import os
import torch
import pickle
import shutil
import numpy as np
from skimage import transform

from torch.utils.data import Dataset

import warnings
warnings.filterwarnings("ignore")


class Demo_3D_Dataset(Dataset):
    def __init__(self, params, train=True, transform=None):
        """
        Assumed data orginazation
        hallucination/
            LfH_demo/
                # different demos
                2021-01-01-00-00-00_model_666/
                    LfH_demo.p
                ...
        """
        super(Demo_3D_Dataset, self).__init__()

        self.params = params
        self.train = train
        self.transform = transform
        self.train_prop = params.model_params.train_prop

        self.depth = []
        self.goal = []
        self.traj = []
        self.lin_vel = []
        self.ang_vel = []
        repo_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        demo_params_dir = os.path.join(params.rslts_dir, "demo_params")
        os.makedirs(demo_params_dir, exist_ok=True)
        for demo_dir in params.demo_dirs:
            demo_dir_ = os.path.join(repo_path, "LfH_demo", demo_dir)

            with open(os.path.join(demo_dir_, "LfH_demo.p"), "rb") as f:
                d = pickle.load(f)
            shutil.copyfile(os.path.join(demo_dir_, "eval_params.json"),
                            os.path.join(demo_params_dir, "{}_eval_params.json".format(demo_dir)))
            shutil.copyfile(os.path.join(demo_dir_, "demo_params.json"),
                            os.path.join(demo_params_dir, "{}_demo_params.json".format(demo_dir)))

            depth = d["depth"]
            goal = d["goal"]
            traj = d["traj"]
            lin_vel = d["lin_vel"]
            ang_vel = d["ang_vel"]

            assert len(depth) == len(goal) == len(traj)

            train_len = int(np.round(len(depth) * self.train_prop))
            if self.train:
                self.depth.extend(depth[:train_len].copy())
                self.goal.extend(goal[:train_len].copy())
                self.traj.extend(traj[:train_len].copy())
                self.lin_vel.extend(lin_vel[:train_len].copy())
                self.ang_vel.extend(ang_vel[:train_len].copy())
            else:
                self.depth.extend(depth[train_len:].copy())
                self.goal.extend(goal[train_len:].copy())
                self.traj.extend(traj[train_len:].copy())
                self.lin_vel.extend(lin_vel[train_len:].copy())
                self.ang_vel.extend(ang_vel[train_len:].copy())

        self.depth, self.goal, self.traj = np.array(self.depth), np.array(self.goal), np.array(self.traj)
        self.lin_vel, self.ang_vel = np.array(self.lin_vel), np.array(self.ang_vel)

    def __len__(self):
        return len(self.depth)

    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()

        data = {"depth": self.depth[idx].copy(),
                "goal": self.goal[idx].copy(),
                "traj": self.traj[idx].copy(),
                "lin_vel": self.lin_vel[idx].copy(),
                "ang_vel": self.ang_vel[idx].copy()}

        if self.transform:
            data = self.transform(data)

        return data


class Flip(object):
    """Convert ndarrays in sample to Tensors."""

    def __init__(self,):
        pass

    def __call__(self, data):
        """
        :param depth: (240, 320)
        :param goal: (3,)
        :param traj: (2, T, 3)
        :return:
        """
        flip_vertically = np.random.rand() > 0.5
        flip_horizontally = np.random.rand() > 0.5
        if flip_horizontally:
            data["depth"] = np.flip(data["depth"], axis=1)
            data["goal"][1] *= -1
            data["traj"][:, :, 1] *= -1
        if flip_vertically:
            data["depth"] = np.flip(data["depth"], axis=0)
            data["goal"][2] *= -1
            data["traj"][:, :, 2] *= -1
        return data


class Noise(object):
    """Convert ndarrays in sample to Tensors."""

    def __init__(self, noise_scale=0.05):
        self.noise_scale = noise_scale
        pass

    def __call__(self, data):
        depth_shape = data["depth"].shape
        data["depth"] += np.random.normal(0, self.noise_scale, size=depth_shape)
        return data


class Rescale(object):
    """Rescale the depth in a sample to a given size.

    Args:
        output_size (tuple or int): Desired output size. If tuple, output is
            matched to output_size. If int, smaller of depth edges is matched
            to output_size keeping aspect ratio the same.
    """

    def __init__(self, output_size):
        assert isinstance(output_size, (int, tuple))
        self.output_size = output_size

    def __call__(self, sample):
        depth = sample["depth"]

        if isinstance(self.output_size, int):
            new_h, new_w = self.output_size, self.output_size
        else:
            new_h, new_w = self.output_size

        new_h, new_w = int(new_h), int(new_w)

        depth = transform.resize(depth, (new_h, new_w))
        sample.update({"depth": depth})

        return sample


class ClipNormalization(object):
    def __init__(self, clip_scale):
        self.clip_scale = float(clip_scale)

    def __call__(self, sample):
        depth = sample["depth"]
        depth = np.clip(depth, 0, self.clip_scale)
        depth /= self.clip_scale
        sample.update({"depth": depth})

        return sample


class ToTensor(object):
    """Convert ndarrays in sample to Tensors."""

    def __init__(self,):
        pass

    def __call__(self, data):
        new_data = {}
        for key, val in data.items():
            if key == "depth":
                val = val[None]
            val = val.astype(np.float32)
            new_data[key] = torch.from_numpy(val)
        return new_data
