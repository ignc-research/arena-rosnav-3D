import os
import sys
import time
import json
import shutil
import numpy as np
from collections import OrderedDict
from scipy.interpolate import BSpline

import torch
from torch.utils.data import DataLoader
import torchvision.transforms as transforms
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.tensorboard import SummaryWriter

from dataloader import Demo_3D_Dataset, Flip, Noise, Rescale, ClipNormalization, ToTensor
from LfD_utils import plot_AE


class AttrDict(dict):
    def __init__(self, *args, **kwargs):
        super(AttrDict, self).__init__(*args, **kwargs)
        self.__dict__ = self


class TrainingParams:
    def __init__(self, training_params_fname="params.json", train=True):
        repo_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        training_params_fname = os.path.join(repo_path, "LfD_3D", training_params_fname)
        config = json.load(open(training_params_fname))
        for k, v in config.items():
            self.__dict__[k] = v
        self.__dict__ = self._clean_dict(self.__dict__)

        AE_load_model = None
        if self.training_params.AE_load_dir is not None and self.training_params.AE_model_fname:
            AE_load_model = os.path.join(repo_path, "rslts", "AE_rslts", self.training_params.AE_load_dir,
                                         "trained_models", self.training_params.AE_model_fname)
        self.training_params.AE_load_model = AE_load_model

        if train:
            self.rslts_dir = os.path.join(repo_path, "rslts", "LfD_3D_rslts", time.strftime("%Y-%m-%d-%H-%M-%S"))
            os.makedirs(self.rslts_dir)
            shutil.copyfile(training_params_fname, os.path.join(self.rslts_dir, "params.json"))

    def _clean_dict(self, _dict):
        for k, v in _dict.items():
            if v == "":  # encode empty string as None
                v = None
            if isinstance(v, dict):
                v = AttrDict(self._clean_dict(v))
            _dict[k] = v
        return _dict


class Encoder(nn.Module):
    def __init__(self, params):
        super(Encoder, self).__init__()
        self.conv1 = nn.Conv2d(in_channels=1, out_channels=32, kernel_size=3, stride=2)
        self.conv2 = nn.Conv2d(in_channels=32, out_channels=32, kernel_size=3, stride=2)
        self.conv3 = nn.Conv2d(in_channels=32, out_channels=32, kernel_size=3, stride=2)
        self.conv4 = nn.Conv2d(in_channels=32, out_channels=32, kernel_size=3, stride=1)
        self.fc = nn.Linear(32 * 13 * 13, params.model_params.image_Dh)
        self.ln = nn.LayerNorm(params.model_params.image_Dh)

    def forward(self, image):
        image_h = F.leaky_relu(self.conv1(image))
        # print("conv1", image_h.size())
        image_h = F.leaky_relu(self.conv2(image_h))
        # print("conv2", image_h.size())
        image_h = F.leaky_relu(self.conv3(image_h))
        # print("conv3", image_h.size())
        image_h = F.leaky_relu(self.conv4(image_h))
        # print("conv4", image_h.size())
        image_h = image_h.view(image_h.size(0), -1)
        image_h = self.fc(image_h)
        image_h = self.ln(image_h)
        return image_h


class Decoder(nn.Module):
    def __init__(self, params):
        super(Decoder, self).__init__()
        self.fc = nn.Linear(params.model_params.image_Dh, 32 * 13 * 13)
        self.deconv1 = nn.ConvTranspose2d(in_channels=32, out_channels=32, kernel_size=3, stride=1)
        self.deconv2 = nn.ConvTranspose2d(in_channels=32, out_channels=32, kernel_size=3, stride=2)
        self.deconv3 = nn.ConvTranspose2d(in_channels=32, out_channels=32, kernel_size=3, stride=2)
        self.deconv4 = nn.ConvTranspose2d(in_channels=32, out_channels=1, kernel_size=4, stride=2)

    def forward(self, image_h):
        image_h = F.leaky_relu(self.fc(image_h))
        image_h = image_h.view(-1, 32, 13, 13)
        image_h = F.leaky_relu(self.deconv1(image_h))
        # print("deconv1", image_h.size())
        image_h = F.leaky_relu(self.deconv2(image_h))
        # print("deconv2", image_h.size())
        image_h = F.leaky_relu(self.deconv3(image_h))
        # print("deconv3", image_h.size())
        recon_image = self.deconv4(image_h)
        return recon_image


class Model(nn.Module):
    def __init__(self, params, encoder, decoder):
        super(Model, self).__init__()
        self.params = params
        model_params = self.params.model_params
        self.num_control_pts = model_params.knot_end - model_params.knot_start - 4

        self.encoder = encoder
        self.decoder = decoder
        self.fc_goal = nn.Linear(3, model_params.odom_Dh)
        self.fc_lin_vel = nn.Linear(3, model_params.odom_Dh)
        self.fc_ang_vel = nn.Linear(3, model_params.odom_Dh)
        self.fc1 = nn.Linear(model_params.image_Dh + model_params.odom_Dh * 3, 256)
        self.fc2 = nn.Linear(256, 256)
        self.fc3 = nn.Linear(256, 256)
        self.fc4 = nn.Linear(256, self.num_control_pts * 3)
        # self.W = nn.Parameter(torch.rand(params.image_Dh, params.image_Dh))
        self.coef, self.acc_jerk_coef = self._init_coef()

    def _init_coef(self):
        # Find B-spline coef to convert control points to pos, vel, acc, jerk
        model_params = self.params.model_params
        knots = np.arange(model_params.knot_start, model_params.knot_end, 1) * model_params.knot_dt
        pos_bspline = BSpline(knots, np.eye(self.num_control_pts), 3, extrapolate=False)
        vel_bspline = pos_bspline.derivative(nu=1)
        acc_bspline = pos_bspline.derivative(nu=2)
        jerk_bspline = pos_bspline.derivative(nu=3)
        t = np.linspace(knots[3], knots[-4], model_params.label_t_steps)
        coef = np.array([pos_bspline(t), vel_bspline(t)])  # (4, 201, 3)
        acc_jerk_coef = np.array([acc_bspline(t), jerk_bspline(t)])  # (4, 201, 3)
        assert not np.isnan(coef).any()
        assert not np.isnan(acc_jerk_coef).any()
        # (1, 4, 201, num_control_pts)
        coef = torch.from_numpy(coef.astype(np.float32)[np.newaxis, ...]).to(self.params.device)
        acc_jerk_coef = torch.from_numpy(acc_jerk_coef.astype(np.float32)[np.newaxis, ...]).to(self.params.device)
        return coef, acc_jerk_coef

    def forward(self, image, goal, lin_vel, ang_vel, reconstruct_depth=False):
        image_h = self.encoder(image)
        if reconstruct_depth:
            reconstructed_depth = self.decoder(image_h)
        goal_h = self.fc_goal(goal)
        lin_vel_h = self.fc_lin_vel(lin_vel)
        ang_vel_h = self.fc_ang_vel(ang_vel)

        h = torch.cat([image_h, goal_h, lin_vel_h, ang_vel_h], dim=-1)
        h = F.leaky_relu(self.fc1(h))
        h = F.leaky_relu(self.fc2(h))
        h = F.leaky_relu(self.fc3(h))
        outputs = self.fc4(h)
        outputs = outputs.view(-1, self.num_control_pts, 3)
        if reconstruct_depth:
            return outputs, reconstructed_depth
        return outputs

    def BC_loss(self, control_pts, traj, lin_vel, ang_vel):
        # vel = control_pts[:, 1:] - control_pts[:, :-1]
        # acc = vel[:, 1:] - vel[:, :-1]
        # jerk = vel[:, 1:] - vel[:, :-1]
        # smoothness_loss = torch.mean(torch.sum(acc ** 2, dim=(1, 2)) + torch.sum(jerk ** 2, dim=(1, 2)))

        control_pts = control_pts.view(-1, 1, self.num_control_pts, 3)      # (batch_size, 1, num_control_pts, 3)
        reconst_traj = torch.matmul(self.coef, control_pts)                 # (batch_size, 2, T, 3)
        reconst_loss = torch.mean(torch.sum((reconst_traj - traj) ** 2, dim=(1, 2, 3)))

        acc_jerk_traj = torch.matmul(self.acc_jerk_coef, control_pts)        # (batch_size, 2, T, 3)
        acc, jerk = torch.unbind(acc_jerk_traj, dim=1)
        smoothness_loss = torch.mean(torch.sum(acc ** 2, dim=(1, 2)) + torch.sum(jerk ** 2, dim=(1, 2)))
        smoothness_loss *= self.params.model_params.lambda_smoothness

        initial_pos, initial_vel = reconst_traj[:, 0, 0], reconst_traj[:, 1, 0]     # (batch_size, 3)
        initial_smoothness_loss = (initial_pos ** 2).sum(dim=-1).mean() + 10 * ((initial_vel - lin_vel) ** 2).sum(dim=-1).mean()
        initial_smoothness_loss *= self.params.model_params.lambda_initial_smoothness

        BC_loss = reconst_loss + smoothness_loss + initial_smoothness_loss
        return BC_loss, (reconst_loss, smoothness_loss, initial_smoothness_loss)

    def reconstruct(self, image):
        image_h = self.encoder(image)
        reconstr_image = self.decoder(image_h)
        return reconstr_image

    def train(self, training=True):
        self.training = training
        self.encoder.train(training)
        if self.decoder is not None:
            self.decoder.train(training)


def train(params):
    # params.device = device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    params.device = device = torch.device("cpu")
    model_params = params.model_params
    training_params = params.training_params

    encoder = Encoder(params).to(device)
    decoder = None
    
    use_AE = params.use_AE
    use_pretrained_AE = False
    if use_AE:
        use_pretrained_AE = training_params.AE_load_model is not None
        decoder = Decoder(params).to(device)
        if use_pretrained_AE:
            AE_state_dict = torch.load(training_params.AE_load_model, map_location=device)
            encoder_state_dict = OrderedDict([(k.split("encoder.")[1], v) for k, v in AE_state_dict.items()
                                              if "encoder" in k])
            decoder_state_dict = OrderedDict([(k.split("decoder.")[1], v) for k, v in AE_state_dict.items()
                                              if "decoder" in k])
            encoder.load_state_dict(encoder_state_dict)
            decoder.load_state_dict(decoder_state_dict)

    model = Model(params, encoder, decoder).to(device)
    optimizer = optim.Adam(model.parameters(), lr=training_params.lr)

    train_dataset = Demo_3D_Dataset(params, train=True,
                                    transform=transforms.Compose([Rescale(model_params.image_size),
                                                                  Noise(model_params.noise_scale),
                                                                  ClipNormalization(model_params.max_depth),
                                                                  Flip(),
                                                                  ToTensor()]))
    test_dataset = Demo_3D_Dataset(params, train=False,
                                   transform=transforms.Compose([Rescale(model_params.image_size),
                                                                 ClipNormalization(model_params.max_depth),
                                                                 ToTensor()]))
    train_dataloader = DataLoader(train_dataset, batch_size=training_params.batch_size, shuffle=True, num_workers=4)
    test_dataloader = DataLoader(test_dataset, batch_size=training_params.batch_size, shuffle=False, num_workers=4)

    writer = SummaryWriter(os.path.join(params.rslts_dir, "tensorboard"))

    # model saving
    model_dir = os.path.join(params.rslts_dir, "trained_models")
    if not os.path.exists(model_dir):
        os.makedirs(model_dir)

    for epoch in range(training_params.epochs):
        train_BC_loss, test_BC_loss = [], []
        train_smooth_loss, test_smooth_loss = [], []
        train_init_loss, test_init_loss = [], []
        train_AE_loss, test_AE_loss = [], []
        model.train(training=True)
        for i_batch, sample_batched in enumerate(train_dataloader):
            # get the inputs; data is a list of [inputs, labels]
            for key, val in sample_batched.items():
                sample_batched[key] = val.to(device)
            depth = sample_batched["depth"]
            goal = sample_batched["goal"]
            traj = sample_batched["traj"]           # (batch_size, 2, T, 3)
            lin_vel = sample_batched["lin_vel"]
            ang_vel = sample_batched["ang_vel"]

            optimizer.zero_grad()
            control_pts = model(depth, goal, lin_vel, ang_vel, reconstruct_depth=use_AE)
            if use_AE:
                control_pts, reconstructed_depth = control_pts

            BC_loss, (reconst_loss, smoothness_loss, init_smoothness_loss) = \
                model.BC_loss(control_pts, traj, lin_vel, ang_vel)
            BC_loss.backward(retain_graph=use_AE)
            if use_AE and use_pretrained_AE and epoch < params.AE_frozen_epoch:
                encoder.zero_grad()

            if use_AE:
                AE_loss = torch.mean(torch.sum((depth - reconstructed_depth) ** 2, dim=(1, 2, 3)))
                if not use_pretrained_AE or epoch >= params.AE_frozen_epoch:
                    AE_loss.backward()
                train_AE_loss.append(AE_loss.item())
                train_depth, train_reconstructed_depth = depth, reconstructed_depth

            optimizer.step()

            train_BC_loss.append(reconst_loss.item())
            train_smooth_loss.append(smoothness_loss.item())
            train_init_loss.append(init_smoothness_loss.item())

            print("{}/{}, {}/{}".format(epoch + 1, training_params.epochs,
                                        i_batch + 1, training_params.batch_per_epoch))
            if i_batch + 1 == training_params.batch_per_epoch:
                break

        model.train(training=False)
        for i_batch, sample_batched in enumerate(test_dataloader):
            for key, val in sample_batched.items():
                sample_batched[key] = val.to(device)
            depth = sample_batched["depth"]
            goal = sample_batched["goal"]
            traj = sample_batched["traj"]           # (batch_size, 2, T, 3)
            lin_vel = sample_batched["lin_vel"]
            ang_vel = sample_batched["ang_vel"]

            with torch.no_grad():
                control_pts = model(depth, goal, lin_vel, ang_vel, reconstruct_depth=use_AE)
                if use_AE:
                    control_pts, reconstructed_depth = control_pts
                BC_loss, (reconst_loss, smoothness_loss, init_smoothness_loss) = \
                    model.BC_loss(control_pts, traj, lin_vel, ang_vel)
                test_BC_loss.append(reconst_loss.item())
                test_smooth_loss.append(smoothness_loss.item())
                test_init_loss.append(init_smoothness_loss.item())

                if use_AE:
                    AE_loss = torch.mean(torch.sum((depth - reconstructed_depth) ** 2, dim=(1, 2, 3)))
                    test_AE_loss.append(AE_loss.item())

                    test_depth, test_reconstructed_depth = depth, reconstructed_depth

        if writer is not None:
            writer.add_scalar("train/BC loss", np.mean(train_BC_loss), epoch)
            writer.add_scalar("test/BC loss", np.mean(test_BC_loss), epoch)
            writer.add_scalar("train/smoothness loss", np.mean(train_smooth_loss), epoch)
            writer.add_scalar("test/smoothness loss", np.mean(test_smooth_loss), epoch)
            writer.add_scalar("train/init smoothness loss", np.mean(train_init_loss), epoch)
            writer.add_scalar("test/init smoothness loss", np.mean(test_init_loss), epoch)
            if use_AE:
                writer.add_scalar("train/AE loss", np.mean(train_AE_loss), epoch)
                writer.add_scalar("test/AE loss", np.mean(test_AE_loss), epoch)
                if epoch % 5 == 0:
                    writer.add_figure("train/AE", plot_AE(train_depth, train_reconstructed_depth), epoch)
                    writer.add_figure("test/AE", plot_AE(test_depth, test_reconstructed_depth), epoch)

        if (epoch + 1) % training_params.saving_freq == 0:
            torch.save(model.state_dict(), os.path.join(model_dir, "model_{0}".format(epoch + 1)),
                       pickle_protocol=2, _use_new_zipfile_serialization=False)


if __name__ == "__main__":
    params = TrainingParams()
    train(params)
