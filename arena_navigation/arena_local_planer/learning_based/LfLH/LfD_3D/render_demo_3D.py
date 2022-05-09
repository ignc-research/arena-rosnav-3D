import os
import json
import pickle
import shutil
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from joblib import Parallel, delayed

import torch

import sys
repo_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(os.path.join(repo_path, "LfH"))
import utils


class Params:
    def __init__(self):
        self.LfH_dir = "2021-07-12-12-02-34_model_2000"
        self.n_render_per_sample = 1
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

        # for additional obstable generation
        self.n_pts_to_consider = 5
        self.n_additional_obs = 0
        self.loc_radius = 8.0
        self.loc_span = 80
        self.size_min, self.size_max = 0.2, 0.8
        self.vel_time = 1.0
        self.vel_span = 60

        # for observation rendering
        self.batch_size = 128
        self.image_scale = image_scale = 1.0
        self.image_h = 320
        self.image_v = 240
        self.max_depth = 1000
        self.fh = 282.8       # from ego-planner
        self.fv = 282.7
        self.ch = 160
        self.cv = 120
        self.plot_freq = 200
        self.convert_infi_depth_to_zero = True
        self.proj_to_x = True
        self.render_ground = True

        # post process params
        self.batch_size //= self.n_render_per_sample
        self.loc_span *= np.pi / 180
        self.vel_span *= np.pi / 180

        self.image_h = int(self.image_h / image_scale)
        self.image_v = int(self.image_v / image_scale)
        self.fh /= image_scale
        self.fv /= image_scale
        self.ch /= image_scale
        self.cv /= image_scale
        if self.convert_infi_depth_to_zero:
            self.max_depth = max(self.max_depth, 500.0)

    def to_dict(self):
        import copy
        params_dict = copy.deepcopy(params.__dict__)
        params_dict.pop("device")
        return params_dict


def angle_diff(angle1, angle2):
    diff = np.abs(angle1 - angle2)
    if diff > np.pi:
        diff = 2 * np.pi - diff
    return diff


def find_raycast(pos, dir, loc, size, params, proj_to_x=False):
    """

    :param pos: (Bs1, ..., BsN, 3)
    :param dir: (Bs1, ..., BsN, 3)
    :param loc: (Bs1, ..., BsN, 3)
    :param size: (Bs1, ..., BsN, 3)
    :return: (Bs1, ..., BsN)
    """
    A = (dir ** 2 / size ** 2).sum(axis=-1)
    B = (2 * dir * (pos - loc) / size ** 2).sum(axis=-1)
    C = ((pos - loc) ** 2 / size ** 2).sum(axis=-1) - 1
    delta = B ** 2 - 4 * A * C
    ray = np.full(delta.shape, params.max_depth)
    delta_mask = delta >= 0
    delta = np.maximum(0, delta)
    l1 = (-B - np.sqrt(delta)) / (2 * A)
    l2 = (-B + np.sqrt(delta)) / (2 * A)
    # notice l1 always smaller than l2
    zero_ray_mask = (l1 < 0) & (l2 > 0)
    gt_zero_ray_mask = l1 >= 0
    ray[delta_mask & zero_ray_mask] = 0
    if proj_to_x:
        l1 = l1 * dir[..., 0]
    ray[delta_mask & gt_zero_ray_mask] = l1[delta_mask & gt_zero_ray_mask]
    return ray


def find_raycast_torch(pos, dir, loc, size, params):
    """

    :param pos: (Bs1, ..., BsN, 3)
    :param dir: (Bs1, ..., BsN, 3)
    :param loc: (Bs1, ..., BsN, 3)
    :param size: (Bs1, ..., BsN, 3)
    :return: (Bs1, ..., BsN)
    """
    pos = torch.from_numpy(pos).to(params.device)
    dir = torch.from_numpy(dir).to(params.device)
    loc = torch.from_numpy(loc).to(params.device)
    size = torch.from_numpy(size).to(params.device)

    with torch.no_grad():
        A = (dir ** 2 / size ** 2).sum(dim=-1)
        B = (2 * dir * (pos - loc) / size ** 2).sum(dim=-1)
        C = ((pos - loc) ** 2 / size ** 2).sum(dim=-1) - 1
        delta = B ** 2 - 4 * A * C                              # (Bs1, ..., BsN)
        ray = torch.full(delta.shape, params.max_depth, dtype=torch.float32).to(params.device)
        delta_mask = delta >= 0
        delta = torch.clamp(delta, min=0)
        l1 = (-B - torch.sqrt(delta)) / (2 * A)
        l2 = (-B + torch.sqrt(delta)) / (2 * A)
        # notice l1 always smaller than l2
        zero_ray_mask = (l1 < 0) & (l2 > 0)
        gt_zero_ray_mask = l1 >= 0
        ray[delta_mask & zero_ray_mask] = 0
        if params.proj_to_x:
            l1 = l1 * dir[..., 0]
        ray[delta_mask & gt_zero_ray_mask] = l1[delta_mask & gt_zero_ray_mask]
    return ray.cpu().numpy()


def find_ground_torch(dir, ground_normal, height, params):
    """

    :param dir: (1, image_w, image_h, 3)
    :param ground_normal: (batch_size, 1, 1, 3)
    :param height: (batch_size, 1, 1)
    :return: (batch_size, image_w, image_h)
    """
    dir = torch.from_numpy(dir).to(params.device)
    ground_normal = torch.from_numpy(ground_normal).to(params.device)
    height = torch.from_numpy(height).to(params.device)

    with torch.no_grad():
        cos = (dir * ground_normal).sum(dim=-1)
        ray = torch.full(cos.shape, params.max_depth, dtype=torch.float32).to(params.device)
        gt_zero_ray_mask = cos > 0
        l = height / cos
        if params.proj_to_x:
            l = l * dir[..., 0]
        l = torch.clip(l, max=params.max_depth)
        ray[gt_zero_ray_mask] = l[gt_zero_ray_mask]
    return ray.cpu().numpy()


def is_colliding_w_traj(poses, vels, loc, size, params):
    """
    :param poses: (N, 3) trajectory positions
    :param vels: (N, 3) trajectory velocities
    :param loc: (M, 3) obs location
    :param size: (M, 3) obs size
    :param vel_time, vel_span: colliding if obs within radius = vel * vel_time and angle in vel_span
    :return: (M,) boolean
    """

    vel_time = params.vel_time
    vel_span = params.vel_span
    vel_norm = np.linalg.norm(vels, axis=-1)
    clearance = vel_norm * vel_time

    M = loc.shape[0]
    N = poses.shape[0]

    obs_diff = loc[:, None] - poses[None]                                               # (M, N, 3)
    obs_dir = obs_diff / np.linalg.norm(obs_diff, axis=-1, keepdims=True)               # (M, N, 3)

    vel_dir = vels / vel_norm[:, None]                                                  # (N, 3)
    vel_dir = np.array([vel_dir] * M)                                                   # (M, N, 3)
    cos_ang_diff = (obs_dir * vel_dir).sum(axis=-1, keepdims=True)                      # (M, N, 1)
    normal = np.cross(vel_dir, obs_dir)                                                 # (M, N, 3)
    r = R.from_rotvec(normal.reshape((-1, 3)) * vel_span / 2)
    rotated_dir = r.apply(vel_dir.reshape((-1, 3))).reshape((M, N, 3))                  # (M, N, 3)
    ray_dir = np.where(cos_ang_diff >= np.cos(vel_span / 2), obs_dir, rotated_dir)      # (M, N, 3)

    raycast = find_raycast(poses[None], ray_dir, loc[:, None], size[:, None], params)   # (M, N)
    is_col = (raycast <= clearance).any(axis=-1)                                        # (M)
    return is_col


def generate_additional_obs(traj, params):
    pos, vel = traj
    pt_indexes = np.round(np.linspace(0, len(pos) - 1, params.n_pts_to_consider)).astype(np.int)
    pos, vel = pos[pt_indexes], vel[pt_indexes]

    locs, sizes = [], []
    batch_size = 2 * params.n_additional_obs

    n_valid_obs = 0
    while True:
        rad = np.random.uniform(0, params.loc_radius, batch_size).astype(np.float32)
        ang1 = np.random.uniform(-params.loc_span / 2, params.loc_span / 2, batch_size).astype(np.float32)
        ang2 = np.random.uniform(0, np.pi, batch_size).astype(np.float32)
        loc = np.stack([np.cos(ang1), np.sin(ang1) * np.cos(ang2), np.sin(ang1) * np.cos(ang2)], axis=-1) * rad[:, None]
        size = np.random.uniform(params.size_min, params.size_max, (batch_size, 3)).astype(np.float32)

        is_col =  is_colliding_w_traj(pos, vel, loc, size, params)
        for is_col_, loc_, size_ in zip(is_col, loc, size):
            if is_col_:
                continue
            n_valid_obs += 1
            locs.append(loc_)
            sizes.append(size_)
            if n_valid_obs == params.n_additional_obs:
                break
        if n_valid_obs == params.n_additional_obs:
            break

    return np.array(locs), np.array(sizes)


def render_depth(obs_loc, obs_size, add_obs_loc, add_obs_size, ori_base, params):
    """

    :param obs_loc: (Bs, N, 3)
    :param obs_size: (Bs, N, 3)
    :param add_obs_loc: (Bs, M, 3)
    :param add_obs_size: (Bs, M, 3)
    :param ori_base: (Bs, 4)
    :param params:
    :return:
    """
    image_h = params.image_h
    image_v = params.image_v
    image_scale = params.image_scale
    fh = params.fh
    fv = params.fv
    ch = params.ch
    cv = params.cv

    if params.n_additional_obs > 0:
        locs = np.concatenate([obs_loc, add_obs_loc], axis=1)
        sizes = np.concatenate([obs_size, add_obs_size], axis=1)
    else:
        locs, sizes = obs_loc, obs_size

    batch_size, n_obs, _ = locs.shape

    pos = np.zeros((image_v, image_h, 3)).astype(np.float32)
    h_ratio = (np.flip(np.arange(image_h)) - 0.5 / image_scale - ch) / fh
    v_ratio = (np.flip(np.arange(image_v)) - 0.5 / image_scale - cv) / fv
    h_ratio, v_ratio = h_ratio.astype(np.float32), v_ratio.astype(np.float32)
    h_grid, v_grid = np.meshgrid(h_ratio, v_ratio)
    dir = np.stack([np.ones((image_v, image_h), dtype=np.float32), h_grid, v_grid], axis=-1)
    dir /= np.linalg.norm(dir, axis=-1, keepdims=True)

    pos = pos[None, :, :, None]                                                 # (1, image_v, image_h, 1, 3)
    dir = dir[None, :, :, None]                                                 # (1, image_v, image_h, 1, 3)
    locs = locs[:, None, None]                                                  # (batch_size, image_v, image_h, n_obs, 3)
    sizes = sizes[:, None, None]                                                # (batch_size, image_v, image_h, n_obs, 3)
    depth = find_raycast_torch(pos, dir, locs, sizes, params)                   # (batch_size, image_v, image_h, n_obs)
    depth = depth.min(axis=-1)                                                  # (batch_size, image_v, image_h)
    if params.render_ground:
        r = R.from_quat(ori_base).inv()
        ground_normal = r.apply([0, 0, -1]).astype(np.float32)                  # (batch_size, 3)
        height = np.random.uniform(0.5, 2.0, batch_size).astype(np.float32)     # (batch_size,)
        ground_normal = ground_normal[:, None, None]                            # (batch_size, 1, 1, 3)
        height = height[:, None, None]                                          # (batch_size, 1, 1)
        dir = dir[..., 0, :]                                                    # (1, image_v, image_h, 3)
        ground_depth = find_ground_torch(dir, ground_normal, height, params)
        depth = np.minimum(depth, ground_depth)
    if params.convert_infi_depth_to_zero:
        depth = np.where(depth < 500.0, depth, 0)
    return depth


def plot_render_rslts(obs_loc, obs_size, traj, goal, lin_vel, add_obs_loc, add_obs_size, depth, params, fig_name):
    image_dir = os.path.join(params.demo_dir, "plots")
    os.makedirs(image_dir, exist_ok=True)

    plt.figure(figsize=(5, 5))
    depth = np.clip(depth, 0, 20)
    plt.imshow(depth)
    plt.axis("off")
    plt.tight_layout()
    plt.savefig(os.path.join(image_dir, fig_name + "_depth"))
    plt.close()

    fig = plt.figure(figsize=(5, 5))

    pos = traj[0]
    goal = goal * 1.0
    lin_vel = lin_vel * 1.0
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], label="traj")
    ax.scatter(pos[0, 0], pos[0, 1], pos[0, 2], color="red")
    ax.plot(*list(zip(pos[0], pos[0] + goal)), color="black", label="goal")
    ax.plot(*list(zip(pos[0], pos[0] + lin_vel)), color="red", label="goal")

    for loc_, size_ in zip(obs_loc, obs_size):
        utils.draw_ellipsoid(loc_, size_, ax, color="red")
    if params.n_additional_obs:
        for loc_, size_ in zip(add_obs_loc, add_obs_size):
            utils.draw_ellipsoid(loc_, size_, ax, color="violet")

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.legend()
    utils.set_axes_equal(ax)
    angles = np.linspace(0, 360, 20, endpoint=False)  # Take 20 angles between 0 and 360

    plt.savefig(os.path.join(image_dir, fig_name + "_traj"))
    utils.rotanimate(ax, angles, os.path.join(image_dir, fig_name + ".gif"), delay=50, elevation=45, width=8, height=6)
    plt.close()


def repeat(input, repeat_time):
    """
    :param array/tensor: (A, B, C)
    :return: (A, A, A, B, B, B, C, C, C) if repeat_time == 3
    """
    array = np.stack([input] * repeat_time, axis=1)
    array = array.reshape(tuple((-1, *array.shape[2:])))
    return array


if __name__ == "__main__":
    params = Params()
    params.demo_dir = demo_dir = os.path.join(repo_path, "LfH_demo", params.LfH_dir)
    LfH_dir = os.path.join(repo_path, "LfH_eval", params.LfH_dir)

    os.makedirs(demo_dir, exist_ok=True)
    json.dump(params.to_dict(), open(os.path.join(demo_dir, "demo_params.json"), "w"), indent=4)
    shutil.copyfile(os.path.join(LfH_dir, "params.json"), os.path.join(demo_dir, "eval_params.json"))
    shutil.copyfile(os.path.join(LfH_dir, "model"), os.path.join(demo_dir, "model"))

    with open(os.path.join(LfH_dir, "LfH_eval.p"), "rb") as f:
        data = pickle.load(f)

    obs_locs = data["obs_loc"]
    obs_sizes = data["obs_size"]
    trajs = data["traj"]
    goals = data["goal"]
    lin_vels = data["lin_vel"]
    ang_vels = data["ang_vel"]
    ori_bases = data["ori_base"]

    demo = {"depth": [],
            "goal": [],
            "traj": [],
            "lin_vel": [],
            "ang_vel": []}

    print("Total samples: {}".format(len(goals)))
    plot_freq = params.plot_freq
    batch_size = params.batch_size
    n_render_per_sample = params.n_render_per_sample
    n_render_in_batch = batch_size * n_render_per_sample
    for i in range(0, len(goals), batch_size):
        print("{}/{}".format(i + 1, len(goals)))
        obs_loc_batch = repeat(obs_locs[i:i + batch_size], n_render_per_sample)
        obs_size_batch = repeat(obs_sizes[i:i + batch_size], n_render_per_sample)
        traj_batch = repeat(trajs[i:i + batch_size], n_render_per_sample)
        goal_batch = repeat(goals[i:i + batch_size], n_render_per_sample)
        lin_vel_batch = repeat(lin_vels[i:i + batch_size], n_render_per_sample)
        ang_vel_batch = repeat(ang_vels[i:i + batch_size], n_render_per_sample)
        ori_base_batch = repeat(ori_bases[i:i + batch_size], n_render_per_sample)

        if params.n_additional_obs > 0:
            add_obs_list = Parallel(n_jobs=os.cpu_count())(delayed(generate_additional_obs)(traj, params)
                                                           for traj in traj_batch)
            add_obs_loc_batch = np.array([ele[0] for ele in add_obs_list])
            add_obs_size_batch = np.array([ele[1] for ele in add_obs_list])
        else:
            add_obs_loc_batch = add_obs_size_batch = None

        depth_batch = render_depth(obs_loc_batch, obs_size_batch, add_obs_loc_batch, add_obs_size_batch,
                                   ori_base_batch, params)

        demo["depth"].extend(depth_batch)
        demo["goal"].extend(goal_batch)
        demo["traj"].extend(traj_batch)
        demo["lin_vel"].extend(lin_vel_batch)
        demo["ang_vel"].extend(ang_vel_batch)
        if i % plot_freq == 0:
            obs_loc = obs_loc_batch[0]
            obs_size = obs_size_batch[0]
            traj = traj_batch[0]
            goal = goal_batch[0]
            lin_vel = lin_vel_batch[0]
            add_obs_loc = add_obs_loc_batch[0] if add_obs_loc_batch is not None else None
            add_obs_size = add_obs_size_batch[0] if add_obs_loc_batch is not None else None
            depth = depth_batch[0]
            plot_render_rslts(obs_loc, obs_size, traj, goal, lin_vel, add_obs_loc, add_obs_size, depth, params,
                              fig_name=str(i))

        if i % 200 == 0:
            demo_ = {k: np.array(v).astype(np.float32) for k, v in demo.items()}
            with open(os.path.join(demo_dir, "LfH_demo.p"), "wb") as f:
                pickle.dump(demo_, f)

    demo = {k: np.array(v).astype(np.float32) for k, v in demo.items()}
    with open(os.path.join(demo_dir, "LfH_demo.p"), "wb") as f:
        pickle.dump(demo, f)
