import os
import json
import pickle
import shutil
import numpy as np
from joblib import Parallel, delayed
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

import torch
from torch.utils.data import DataLoader
import torchvision.transforms as transforms

from model import Hallucination
from dataloader import HallucinationDataset, ToTensor
from utils import to_numpy, set_axes_equal, draw_ellipsoid, rotanimate
from LfH_main import TrainingParams, AttrDict


def plot_eval_rslts(loc, size, traj, recon_traj, cmd, goal, lin_vel, fname):
    Dy = loc.shape[-1]
    fig = plt.figure()
    if Dy == 2:
        obses = [Ellipse(xy=loc_, width=2 * size_[0], height=2 * size_[1]) for loc_, size_ in zip(loc, size)]
        for obs in obses:
            plt.gca().add_artist(obs)
            obs.set_alpha(0.5)
            obs.set_facecolor(np.random.rand(3))

        pos = traj[0]
        recon_pos = recon_traj[0]
        cmd_vel, cmd_ang_vel = cmd[0], cmd[1]
        cmd_vel = cmd_vel * 0.4
        cmd_ang_vel = cmd_ang_vel * 0.3
        goal = goal * 0.2
        plt.plot(pos[:, 0], pos[:, 1], label="traj")
        plt.plot(recon_pos[:, 0], recon_pos[:, 1], label="recon_traj")
        plt.arrow(0, 0, cmd_vel, 0, width=0.02, label="cmd_vel")
        plt.arrow(0 + cmd[0] / 2.0, 0 - cmd_ang_vel / 2.0, 0, cmd_ang_vel, width=0.02, label="cmd_ang_vel")
        plt.arrow(0, 0, goal[0], goal[1], color="g", width=0.02, label="goal")
        plt.legend()
        plt.gca().axis('equal')
        plt.xlim([-1, 5])
        plt.ylim([-3, 3])
        plt.savefig(fname)
        plt.close()
    else:
        ax = fig.add_subplot(111, projection="3d")
        for loc_, size_ in zip(loc, size):
            draw_ellipsoid(loc_, size_, ax, np.random.rand(3))

        pos = traj[0]
        recon_pos = recon_traj[0]
        goal = goal * 1.0
        lin_vel = lin_vel * 1.0
        ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], label="traj")
        ax.scatter(pos[0, 0], pos[0, 1], pos[0, 2], color="red")
        ax.plot(recon_pos[:, 0], recon_pos[:, 1], recon_pos[:, 2], label="recon_traj")
        ax.plot(*list(zip(pos[0], pos[0] + goal)), color="black", label="goal")
        ax.plot(*list(zip(pos[0], pos[0] + lin_vel)), color="red", label="lin_vel")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.legend()
        set_axes_equal(ax)
        # ax.axis('off')  # remove axes for visual appeal

        angles = np.linspace(0, 360, 20, endpoint=False)  # Take 20 angles between 0 and 360

        # create an animated gif (20ms between frames)
        # rotanimate(ax, angles, fname + '.gif', delay=50, elevation=45, width=8, height=6)
        # plt.show()
        plt.close()


def repeat(input, repeat_time):
    """
    :param array/tensor: (A, B, C)
    :return: (A, A, A, B, B, B, C, C, C) if repeat_time == 3
    """
    if torch.is_tensor(input):
        input = to_numpy(input)
    array = np.stack([input] * repeat_time, axis=1)
    array = array.reshape(tuple((-1, *array.shape[2:])))
    return array


def repeat_tensor(input, repeat_time):
    """
    :param tensor: (A, B, C)
    :return: (A, A, A, B, B, B, C, C, C) if repeat_time == 3
    """
    input = torch.stack([input] * repeat_time, dim=1)
    output = input.view(tuple((-1, *input.size()[2:])))
    return output


def eval(params):
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    params.device = device
    eval_params = params.eval_params
    training_params = params.training_params
    sample_per_traj = eval_params.sample_per_traj
    downsample_traj = eval_params.downsample_traj
    n_traj_in_batch = 32 // sample_per_traj
    batch_size = n_traj_in_batch * downsample_traj

    dataset = HallucinationDataset(params, eval=True, transform=transforms.Compose([ToTensor()]))
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=False, num_workers=4)

    model = Hallucination(params, None).to(device)
    assert os.path.exists(training_params.load_model)
    model.load_state_dict(torch.load(training_params.load_model, map_location=device))
    print(training_params.load_model, "loaded")

    rslts = {"obs_loc": [],
             "obs_size": [],
             "traj": [],
             "goal": []}
    if params.Dy == 2:
        rslts["cmd"] = []
    else:
        rslts["lin_vel"] = []
        rslts["ang_vel"] = []
        rslts["ori_base"] = []

    eval_dir = eval_params.eval_dir
    eval_plots_dir = os.path.join(eval_dir, "plots")
    os.makedirs(eval_plots_dir, exist_ok=True)
    params.pop("device")
    json.dump(params, open(os.path.join(eval_dir, "params.json"), "w"), indent=4)
    shutil.copyfile(training_params.load_model, os.path.join(eval_dir, "model"))
    params.device = device

    for i_batch, sample_batched in enumerate(dataloader):
        for key, val in sample_batched.items():
            val = val[::downsample_traj]
            sample_batched[key] = val.to(device)

        full_traj_tensor = repeat_tensor(sample_batched["full_traj"], sample_per_traj)
        reference_pts_tensor = repeat_tensor(sample_batched["reference_pts"], sample_per_traj)

        reference_pts = to_numpy(reference_pts_tensor)
        trajs = repeat(sample_batched["traj"], sample_per_traj)

        if params.Dy == 2:
            cmds = repeat(sample_batched["cmd"], sample_per_traj)
        else:
            lin_vels = repeat(sample_batched["lin_vel"], sample_per_traj)
            ang_vels = repeat(sample_batched["ang_vel"], sample_per_traj)
            ori_bases = repeat(sample_batched["ori_base"], sample_per_traj)

        n_samples = np.zeros(n_traj_in_batch).astype(np.int)
        print_n_samples = -1
        n_invalid_samples = np.zeros(n_traj_in_batch).astype(np.int)
        while (n_samples < sample_per_traj).any():
            if print_n_samples != n_samples.sum():
                print("{}/{} {}/{}".format(i_batch + 1, len(dataloader),
                                           n_samples.sum(), n_traj_in_batch * sample_per_traj))
                print_n_samples = n_samples.sum()

            # check if stuck for a long time
            need_break = True
            for n_valid, n_invalid in zip(n_samples, n_invalid_samples):
                if n_valid < sample_per_traj and n_invalid < sample_per_traj * eval_params.max_sample_trials:
                    need_break = False
                    break
            if need_break:
                print("give up after {} valid and {} invalid".format(n_samples, n_invalid_samples))
                break

            _, _, loc_tensors, _, _, size_tensors = model(full_traj_tensor, reference_pts_tensor)

            locs = to_numpy(loc_tensors)
            sizes = to_numpy(size_tensors)

            def reference_collision_checker(reference_pts_, loc_, size_):
                # check reference collision
                diff = reference_pts_[3:-3, None] - loc_[None]
                diff_norm = np.linalg.norm(diff, axis=-1)
                diff_dir = diff / diff_norm[..., None]
                radius = 1 / np.sqrt((diff_dir ** 2 / size_[None] ** 2).sum(axis=-1))
                reference_collision = (diff_norm - radius <=
                                       params.optimization_params.clearance * eval_params.clearance_scale).any()
                return reference_collision

            collision_list = Parallel(n_jobs=os.cpu_count())(
                delayed(reference_collision_checker)(reference_pts_, loc_, size_)
                for reference_pts_, loc_, size_ in zip(reference_pts, locs, sizes)
            )

            for j, (col, loc, size, traj) in enumerate(zip(collision_list, locs, sizes, trajs)):
                idx_in_batch = j // sample_per_traj
                if col:
                    n_invalid_samples[idx_in_batch] += 1
                    continue

                if n_samples[idx_in_batch] < sample_per_traj:
                    n_samples[idx_in_batch] += 1

                    rslts["obs_loc"].append(loc)
                    rslts["obs_size"].append(size)
                    rslts["traj"].append(traj)
                    goal = traj[0, -1].copy()
                    goal /= np.linalg.norm(goal)
                    cmd = lin_vel = None
                    if params.Dy == 2:
                        cmd = cmds[j]
                        rslts["goal"].append(goal)
                        rslts["cmd"].append(cmd)
                    else:
                        lin_vel = lin_vels[j]
                        rslts["goal"].append(goal)
                        rslts["lin_vel"].append(lin_vel)
                        rslts["ang_vel"].append(ang_vels[j])
                        rslts["ori_base"].append(ori_bases[j])


                    sample_idx = i_batch * n_traj_in_batch + idx_in_batch
                    if sample_idx % eval_params.plot_freq == 0 and n_samples[idx_in_batch] == 1:
                        fname = os.path.join(eval_plots_dir, str(sample_idx))
                        recon_traj, _ = model.decode(reference_pts_tensor[j:j + 1],
                                                     loc_tensors[j:j + 1], size_tensors[j:j + 1])
                        recon_traj = to_numpy(recon_traj)[0]
                        plot_eval_rslts(loc, size, traj, recon_traj, cmd, goal, lin_vel, fname=fname)

        if i_batch % 100 == 0:
            rslts_ = {key: np.array(val) for key, val in rslts.items()}
            with open(os.path.join(eval_dir, "LfH_eval.p"), "wb") as f:
                pickle.dump(rslts_, f)

    rslts = {key: np.array(val) for key, val in rslts.items()}
    with open(os.path.join(eval_dir, "LfH_eval.p"), "wb") as f:
        pickle.dump(rslts, f)


if __name__ == "__main__":
    load_dir = "2021-07-12-12-02-34"
    model_fname = "model_2000"
    sample_per_traj = 1
    downsample_traj = 4
    plot_freq = 2000
    data_fnames = None
    clearance_scale = 0.6
    max_sample_trials = 10

    repo_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    eval_dir = os.path.join(repo_path, "LfH_eval", "{}_{}".format(load_dir, model_fname))
    load_dir = os.path.join(repo_path, "rslts", "LfH_rslts", load_dir)
    model_fname = os.path.join(load_dir, "trained_models", model_fname)
    params_fname = os.path.join(load_dir, "params.json")

    params = TrainingParams(params_fname, train=False)

    params.training_params.load_model = model_fname
    params.eval_params = eval_params = AttrDict()
    eval_params.eval_dir = eval_dir
    eval_params.params_fname = params_fname
    eval_params.sample_per_traj = sample_per_traj
    eval_params.downsample_traj = downsample_traj
    eval_params.plot_freq = plot_freq
    eval_params.clearance_scale = clearance_scale
    eval_params.max_sample_trials = max_sample_trials
    if data_fnames is not None:
        params.data_fnames = data_fnames

    eval(params)
