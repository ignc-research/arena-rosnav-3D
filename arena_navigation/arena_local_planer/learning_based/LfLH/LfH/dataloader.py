import os
import torch
import pickle
import numpy as np
import matplotlib.pyplot as plt
from torch.utils.data import Dataset
from scipy.interpolate import BSpline
from scipy.spatial.transform import Rotation as R

import warnings
warnings.filterwarnings("ignore")


class HallucinationDataset(Dataset):
    def __init__(self, params, eval=False, transform=None):
        """
        Assumed data orginazation
        hallucination/
            data/
                # different dimension
                2D/
                    # different data files
                    2020_12_23_12_00_00.npy
                3D/
                    ...
        """
        super(HallucinationDataset, self).__init__()

        self.params = params
        self.eval = eval
        self.transform = transform

        self.Dy = params.Dy
        self.odom_freq = params.odom_freq

        repo_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        data_dir = os.path.join(repo_path, "data", "2D" if self.Dy == 2 else "3D")

        model_params = params.model_params
        reference_pt_timestamp = np.arange(model_params.knot_start + 2, model_params.knot_end - 2)
        reference_pt_timestamp = reference_pt_timestamp * model_params.knot_dt
        # for reference pts
        self.reference_pt_idx = np.round(reference_pt_timestamp * self.odom_freq).astype(np.int32)
        # for full_traj
        full_traj_len = self.reference_pt_idx[-1] - self.reference_pt_idx[0] + 1  # from 0 to reference_pt_idx[-1]
        full_traj_len = full_traj_len
        # for traj (bspline order is 3)
        self.traj_start = self.reference_pt_idx[1] - self.reference_pt_idx[0]
        self.traj_end = self.reference_pt_idx[-2] - self.reference_pt_idx[0] + 1   # +1: for exclusion

        # needed in initializing hallucination model
        model_params.full_traj_len = int(full_traj_len)
        model_params.traj_len = int(self.traj_end - self.traj_start)

        self.trajs = []
        self.traj_mapper = []
        self.odom_mapper = []
        traj_idx = 0
        for fname in params.data_fnames:
            data = pickle.load(open(os.path.join(data_dir, fname), "rb"))
            pos, ori, vel, ang_vel = data["pos"], data["ori"], data["vel"], data["ang_vel"]
            self.trajs.append(data)

            assert len(pos) == len(vel) == len(ori)
            self.traj_mapper.extend([traj_idx] * (len(pos) - full_traj_len + 1))
            self.odom_mapper.extend(np.arange(len(pos) - full_traj_len + 1) - self.reference_pt_idx[0])
            traj_idx += 1

        self.traj_mapper = np.array(self.traj_mapper)
        self.odom_mapper = np.array(self.odom_mapper)

    def __len__(self):
        return len(self.traj_mapper)

    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()

        traj_idx = self.traj_mapper[idx]
        odom_idx = self.odom_mapper[idx]

        traj = self.trajs[traj_idx]
        pos, ori, vel, ang_vel = traj["pos"], traj["ori"], traj["vel"], traj["ang_vel"]

        pos_base, ori_base = pos[odom_idx], ori[odom_idx]
        pos_full_traj = pos[odom_idx + self.reference_pt_idx[0]:odom_idx + self.reference_pt_idx[-1] + 1]
        # ori_full_traj = ori[odom_idx + self.reference_pt_idx[0]:odom_idx + self.reference_pt_idx[-1] + 1]
        vel_full_traj = vel[odom_idx + self.reference_pt_idx[0]:odom_idx + self.reference_pt_idx[-1] + 1]
        # ang_vel_full_traj = ang_vel[odom_idx + self.reference_pt_idx[0]:odom_idx + self.reference_pt_idx[-1] + 1]

        r = R.from_quat(ori_base).inv()
        pos_transformed = r.apply(pos_full_traj - pos_base)
        vel_transformed = r.apply(vel_full_traj)

        if self.Dy == 2:
            pos_transformed = pos_transformed[:, :2]
            vel_transformed = vel_transformed[:, :2]

            if not self.eval and np.random.rand() > 0.5:
                # flipping augmentation
                pos_transformed[:, 1] = -pos_transformed[:, 1]
                vel_transformed[:, 1] = -vel_transformed[:, 1]

        pos_label = pos_transformed[self.traj_start:self.traj_end]
        vel_label = vel_transformed[self.traj_start:self.traj_end]

        data = {"reference_pts": pos_transformed[self.reference_pt_idx - self.reference_pt_idx[0]],
                "full_traj": np.concatenate([pos_transformed, vel_transformed], axis=-1),
                "traj": np.stack([pos_label, vel_label], axis=0)}

        if self.eval:
            if self.Dy == 2:
                cmd_vel, cmd_ang_vel = traj.get("cmd_vel"), traj.get("cmd_ang_vel")
                cmd_vel = cmd_vel[odom_idx + self.reference_pt_idx[0] + self.traj_start]
                cmd_ang_vel = cmd_ang_vel[odom_idx + self.reference_pt_idx[0] + self.traj_start]
                cmd = np.array([cmd_vel, cmd_ang_vel])
                data.update({"cmd": cmd})
            else:
                # goals = traj.get("goal")
                # goal = goals[odom_idx + self.reference_pt_idx[0] + self.traj_start]
                # data.update({"goal": goal})
                lin_vel = vel_label[0]
                ang_vel = ang_vel[odom_idx + self.reference_pt_idx[0] + self.traj_start]
                ang_vel = r.apply(ang_vel)
                data.update({"lin_vel": lin_vel, "ang_vel": ang_vel, "ori_base": ori_base})

        if self.transform:
            data = self.transform(data)

        return data


class FilterKeys(object):
    """Convert ndarrays in sample to Tensors."""

    def __init__(self, filtered_keys):
        self.filtered_keys = filtered_keys

    def __call__(self, data):
        data = {}
        for key in self.filtered_keys:
            data.pop(key, None)
        return data


class ToTensor(object):
    """Convert ndarrays in sample to Tensors."""

    def __init__(self,):
        pass

    def __call__(self, data):
        new_data = {}
        for key, val in data.items():
            if key == "full_traj":
                val = val.transpose((1, 0))  # as torch conv is channel first
            val = val.astype(np.float32)
            new_data[key] = torch.from_numpy(val)
        return new_data
