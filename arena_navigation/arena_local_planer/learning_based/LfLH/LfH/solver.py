import numpy as np

import torch
import torch.nn as nn
import torch.nn.functional as F

from torchdiffeq import odeint, odeint_adjoint, odeint_event

import cvxpy as cp
from cvxpylayers.torch import CvxpyLayer
# from custom_cvxpyLayer import CustomCvxpyLayer

EPS = 1e-6


class OptimizationFunc(nn.Module):
    def __init__(self, params):
        super(OptimizationFunc, self).__init__()
        self.obs_loc = None
        self.obs_size = None
        self.params = params
        self.reference_pts = None
        self.direction = None
        self.intersection = None
        self.SECOND_DERIVATIVE_CONTINOUS = False
        self.prev_loss = None

    def update(self, obs_loc, obs_size, reference_pts, init_control_pts):
        """
        :param obs_loc: (B, num_obs, Dy) tensor
        :param obs_size: (B, num_obs, Dy) tensor
        :param reference_pts:  (B, num_control_pts, Dy) tensor
            pos on collected trajectory, used as the guidance for control points to get out of obstacles
        :param init_control_pts:  (B, num_control_pts, Dy) tensor
            the straight line connecting the start and the end of reference_pts
        :return:
        """
        self.obs_loc = obs_loc
        self.obs_size = obs_size
        self.reference_pts = reference_pts

        # direction: normalized, from obs_loc to reference_pts, (B, num_control_pts, num_obs, Dy)
        batch_size, _, Dy = list(reference_pts.size())
        diff = reference_pts.view(batch_size, -1, 1, Dy) - obs_loc.view(batch_size, 1, -1, Dy)
        self.direction = diff / torch.linalg.norm(diff, dim=-1, keepdims=True)

        # intersection = obs_loc + t * direction. denote direction = (x, y, z) and obs_size = (a, b, c)
        # then (t * x)^2 / a^2 + (t * y)^2 / b^2 + (t * z)^2 / c^2 = 1
        obs_size = obs_size.view(batch_size, 1, -1, Dy)
        t = 1 / torch.sqrt(torch.sum(self.direction ** 2 / obs_size ** 2, dim=-1, keepdim=True))
        self.intersection = obs_loc[:, None] + t * self.direction       # (B, num_control_pts, num_obs, Dy)

        dist = init_control_pts[:, :, None] - self.obs_loc[:, None, :]          # (B, num_control_pts, num_obs, Dy)
        dist_len = torch.norm(dist, dim=-1)                                     # (B, num_control_pts, num_obs)
        dist_direction = dist / torch.norm(dist, dim=-1, keepdim=True)          # (B, num_control_pts, num_obs, Dy)
        radius_along_dir = 1 / torch.sqrt(torch.sum(dist_direction ** 2 / self.obs_size[:, None, :] ** 2, dim=-1))
                                                                                # (B, num_control_pts, num_obs)
        self.in_collision = dist_len <= radius_along_dir + self.params.optimization_params.clearance

        self.prev_loss = None

    def loss(self, control_pts):
        params = self.params
        model_params = params.model_params
        optimization_params = params.optimization_params
        batch_size = control_pts.size(0)
        assert self.obs_loc is not None and self.obs_size is not None

        control_pts = control_pts.view(batch_size, -1, params.Dy)  # (B, num_control_pts, Dy)

        # smoothness
        # for some reason, ego-planner doesn't divide dt when calculating vel and acc, so keep the same here
        # https://github.com/ZJU-FAST-Lab/ego-planner/blob/master/src/planner/bspline_opt/src/bspline_optimizer.cpp#L413-L452
        vel = (control_pts[:, 1:] - control_pts[:, :-1])                    # (B, num_control_pts - 1, Dy)
        acc = (vel[:, 1:] - vel[:, :-1])                                    # (B, num_control_pts - 2, Dy)
        jerk = (acc[:, 1:] - acc[:, :-1])                                   # (B, num_control_pts - 2, Dy)

        smoothness_loss = torch.mean(torch.sum(jerk ** 2, dim=(1, 2)))

        # collision
        # not used, but may be a good future reference
        # https://math.stackexchange.com/questions/3722553/find-intersection-between-line-and-ellipsoid

        control_pts_ = control_pts.view(batch_size, -1, 1, params.Dy)

        # dist = (control_pts - intersection_pts).dot(direction to reference on traj), (B, num_control_pts, num_obs)
        dist = torch.sum((control_pts_ - self.intersection) * self.direction, dim=-1)
        clearance = optimization_params.clearance
        dist_error = clearance - dist
        gt_0_le_clearance_mask = (dist_error > 0) & (dist_error <= clearance) & self.in_collision
        gt_clearance_mask = (dist_error > clearance) & self.in_collision
        dist_gt_0_le_clearance = dist_error[gt_0_le_clearance_mask]
        dist_gt_clearance = dist_error[gt_clearance_mask]

        # see https://arxiv.org/pdf/2008.08835.pdf Eq 5
        a, b, c = 3 * clearance, -3 * clearance ** 2, clearance ** 3
        collision_loss = torch.sum(dist_gt_0_le_clearance ** 3) + \
            torch.sum(a * dist_gt_clearance ** 2 + b * dist_gt_clearance + c)
        collision_loss /= batch_size

        # feasibility, see https://arxiv.org/pdf/2008.08835.pdf Eq 10 and
        # https://github.com/ZJU-FAST-Lab/ego-planner/blob/master/src/planner/bspline_opt/src/bspline_optimizer.cpp#L462-L577

        knot_dt = model_params.knot_dt
        demarcation = optimization_params.demarcation
        max_vel = optimization_params.max_vel
        max_acc = optimization_params.max_acc

        vel /= knot_dt
        acc /= knot_dt ** 2

        if self.SECOND_DERIVATIVE_CONTINOUS:
            a, b, c = 3 * demarcation, -3 * demarcation ** 2, demarcation ** 3
            raise NotImplementedError("wait for ego_planner response")
        else:
            vel = torch.abs(vel)
            ge_max_vel_mask = vel >= max_vel
            vel_ge_max_vel = vel[ge_max_vel_mask]
            vel_feasibility_loss = (vel_ge_max_vel - max_vel) ** 2

            acc = torch.abs(acc)
            ge_max_acc_mask = acc >= max_acc
            acc_ge_max_acc = acc[ge_max_acc_mask]
            acc_feasibility_loss = (acc_ge_max_acc - max_acc) ** 2

            # extra "/ knot_dt ** 2": from ego_planner, to make vel and acc have similar magnitudes
            feasibility_loss = torch.sum(vel_feasibility_loss) / knot_dt ** 2 + torch.sum(acc_feasibility_loss)
            feasibility_loss /= batch_size

        loss = optimization_params.lambda_smoothness * smoothness_loss + \
               optimization_params.lambda_collision * collision_loss + \
               optimization_params.lambda_feasibility * feasibility_loss

        return loss

    def forward(self, t, control_pts):
        params = self.params
        device = params.device
        model_params = params.model_params
        optimization_params = params.optimization_params
        batch_size = control_pts.size(0)
        assert self.obs_loc is not None and self.obs_size is not None

        control_pts = control_pts.view(batch_size, -1, params.Dy)  # (B, num_control_pts, Dy)

        # smoothness
        # for some reason, ego-planner doesn't divide dt when calculating vel and acc, so keep the same here
        # https://github.com/ZJU-FAST-Lab/ego-planner/blob/master/src/planner/bspline_opt/src/bspline_optimizer.cpp#L413-L452
        vel = (control_pts[:, 1:] - control_pts[:, :-1])                    # (B, num_control_pts - 1, Dy)
        acc = (vel[:, 1:] - vel[:, :-1])                                    # (B, num_control_pts - 2, Dy)
        jerk = (acc[:, 1:] - acc[:, :-1])                                   # (B, num_control_pts - 2, Dy)

        smoothness_grad = torch.zeros_like(control_pts)
        smoothness_grad[:, :-3] += -1 * 2 * jerk
        smoothness_grad[:, 1:-2] += 3 * 2 * jerk
        smoothness_grad[:, 2:-1] += -3 * 2 * jerk
        smoothness_grad[:, 3:] += 1 * 2 * jerk
        smoothness_grad /= batch_size

        # collision
        # not used, but may be a good future reference
        # https://math.stackexchange.com/questions/3722553/find-intersection-between-line-and-ellipsoid

        control_pts_ = control_pts.view(batch_size, -1, 1, params.Dy)

        # dist = (control_pts - intersection_pts).dot(direction to reference on traj), (B, num_control_pts, num_obs)
        dist = torch.sum((control_pts_ - self.intersection) * self.direction, dim=-1)
        clearance = optimization_params.clearance
        dist_error = clearance - dist
        gt_0_le_clearance_mask = (dist_error > 0) & (dist_error <= clearance) & self.in_collision
        gt_clearance_mask = (dist_error > clearance) & self.in_collision
        dist_gt_0_le_clearance = dist_error[gt_0_le_clearance_mask]
        dist_gt_clearance = dist_error[gt_clearance_mask]

        # see https://arxiv.org/pdf/2008.08835.pdf Eq 5
        a, b, c = 3 * clearance, -3 * clearance ** 2, clearance ** 3

        collision_grad = torch.zeros(batch_size, model_params.num_control_pts, model_params.num_obs, params.Dy).to(device)
        collision_grad[gt_0_le_clearance_mask] += -3 * torch.unsqueeze(dist_gt_0_le_clearance ** 2, dim=-1) \
                                                  * self.direction[gt_0_le_clearance_mask]
        collision_grad[gt_clearance_mask] += -torch.unsqueeze(2 * a * dist_gt_clearance + b, dim=-1) \
                                             * self.direction[gt_clearance_mask]
        collision_grad = torch.sum(collision_grad, dim=2)
        collision_grad /= batch_size

        # feasibility, see https://arxiv.org/pdf/2008.08835.pdf Eq 10 and
        # https://github.com/ZJU-FAST-Lab/ego-planner/blob/master/src/planner/bspline_opt/src/bspline_optimizer.cpp#L462-L577

        knot_dt = model_params.knot_dt
        demarcation = optimization_params.demarcation
        max_vel = optimization_params.max_vel
        max_acc = optimization_params.max_acc

        vel /= knot_dt
        acc /= knot_dt ** 2

        if self.SECOND_DERIVATIVE_CONTINOUS:
            a, b, c = 3 * demarcation, -3 * demarcation ** 2, demarcation ** 3
            raise NotImplementedError("wait for ego_planner response")
            # # max_vel < v < vj
            # ge_max_vel_lt_vj_mask = (vel >= max_vel) & (vel < max_vel + params.demarcation)
            # # v >= vj
            # ge_vj_mask = vel >= max_vel + params.demarcation
            # # -vj < v <= -max_vel
            # le_neg_max_vel_gt_neg_vj_mask = (vel <= -max_vel) & (vel > -(max_vel + params.demarcation))
            # # v < -vj
            # le_neg_vj_mask = vel <= -(max_vel + params.demarcation)
            #
            # vel_ge_max_vel_lt_vj = vel[ge_max_vel_lt_vj_mask]
            # vel_ge_vj = vel[ge_vj_mask]
            # vel_le_neg_max_vel_gt_neg_vj = vel[le_neg_max_vel_gt_neg_vj_mask]
            # vel_le_neg_vj = vel[le_neg_vj_mask]
            #
            # feasibility_loss += torch.sum((vel_ge_max_vel_lt_vj - max_vel) ** 3) \
            #                     - torch.sum((vel_le_neg_max_vel_gt_neg_vj + max_vel) ** 3) \
            #                     + torch.sum(a * (vel_ge_vj - max_vel) ** 2 + b * (vel_ge_vj - max_vel) + c) \
            #                     + torch.sum(a * (vel_le_neg_vj + max_vel) ** 2 + b * (vel_le_neg_vj + max_vel) + c)
            # feasibility_loss /= knot_dt ** 3  # from ego_planner: to make vel and acc have similar magnitudes
            #
            # acc = torch.abs(acc)
            # ge_max_acc_lt_vj_mask = (acc > max_acc) & (acc < max_acc + params.demarcation)
            # ge_vj_mask = acc >= max_acc + params.demarcation
            # acc_ge_max_acc_lt_vj = acc[ge_max_acc_lt_vj_mask]
            # acc_ge_vj = acc[ge_vj_mask]
            # feasibility_loss += torch.sum((acc_ge_max_acc_lt_vj - max_acc) ** 3) + \
            #                     torch.sum(a * acc_ge_vj ** 2 + b * acc_ge_vj + c)
            # feasibility_loss /= batch_size
        else:
            vel = torch.abs(vel)
            ge_max_vel_mask = vel >= max_vel
            vel_ge_max_vel = vel[ge_max_vel_mask]

            vel_feasibility_grad = torch.zeros_like(control_pts).to(device)
            vel_feasibility_grad[:, :-1][ge_max_vel_mask] += -2 * (vel_ge_max_vel - max_vel) / knot_dt
            vel_feasibility_grad[:, 1:][ge_max_vel_mask] += 2 * (vel_ge_max_vel - max_vel) / knot_dt

            acc = torch.abs(acc)
            ge_max_acc_mask = acc >= max_acc
            acc_ge_max_acc = acc[ge_max_acc_mask]

            acc_feasibility_grad = torch.zeros_like(control_pts).to(device)
            acc_feasibility_grad[:, :-2][ge_max_acc_mask] += 2 * (acc_ge_max_acc - max_acc) / knot_dt ** 2
            acc_feasibility_grad[:, 1:-1][ge_max_acc_mask] += -4 * (acc_ge_max_acc - max_acc) / knot_dt ** 2
            acc_feasibility_grad[:, 2:][ge_max_acc_mask] += 2 * (acc_ge_max_acc - max_acc) / knot_dt ** 2

            # extra "/ knot_dt ** 2": from ego_planner, to make vel and acc have similar magnitudes
            feasibility_grad = vel_feasibility_grad / knot_dt ** 2 + acc_feasibility_grad

        grad = optimization_params.lambda_smoothness * smoothness_grad + \
               optimization_params.lambda_collision * collision_grad + \
               optimization_params.lambda_feasibility * feasibility_grad
        grad = grad.view(batch_size, -1)

        # gradient descent
        grad *= -1
        grad *= optimization_params.optimization_lr

        return grad

    def loss_decrease(self, t, control_pts):
        loss = self.loss(control_pts)

        prev_loss = self.prev_loss
        self.prev_loss = loss
        if prev_loss is not None:
            print(loss, prev_loss - loss, prev_loss >= loss)
            return prev_loss >= loss
        else:
            return loss


class Neural_ODE_Decoder(nn.Module):
    def __init__(self, params):
        super(Neural_ODE_Decoder, self).__init__()
        self.device = params.device
        self.opt_func = OptimizationFunc(params).to(params.device)
        opt_params = params.optimization_params
        self.ode_num_timestamps = opt_params.ode_num_timestamps
        self.t = np.linspace(0, opt_params.ode_t_end, opt_params.ode_num_timestamps).astype(np.float32)
        self.t = torch.from_numpy(self.t).to(params.device)
        self.t0 = self.t[0]

    def forward(self, init_control_pts, obs_loc, obs_size, reference_pts):
        self.opt_func.update(obs_loc, obs_size, reference_pts, init_control_pts)
        batch_size, num_control_pts, Dy = init_control_pts.size()
        init_control_pts = init_control_pts.view(init_control_pts.size(0), -1)

        # recon_control_points.size(): (ode_num_timestamps, batch_size, num_control_pts * Dy)
        # recon_control_points = odeint(self.opt_func, init_control_pts, self.t, atol=1e-7, rtol=1e-7).to(self.device)
        recon_control_points = odeint(self.opt_func, init_control_pts, self.t,
                                      method='rk4', options=dict(step_size=1.0)).to(self.device)
        # _, recon_control_points = odeint_event(self.opt_func, init_control_pts, self.t0,
        #                                        event_fn=self.opt_func.loss_decrease, odeint_interface=odeint,
        #                                        atol=1e-8, rtol=1e-8)

        recon_control_points = recon_control_points.view(self.ode_num_timestamps, batch_size, num_control_pts, Dy)
        return recon_control_points


class CVX_Decoder(nn.Module):
    def __init__(self, params):
        super(CVX_Decoder, self).__init__()
        self.params = params
        self.device = params.device
        self.SECOND_DERIVATIVE_CONTINOUS = False
        self._init_cvx_problem()

    def _init_cvx_problem(self):
        Dy = self.params.Dy
        num_obs = self.params.model_params.num_obs
        num_control_pts = self.params.model_params.num_control_pts

        control_pts = cp.Variable((num_control_pts, Dy))
        direction = [cp.Parameter((num_control_pts, Dy)) for _ in range(num_obs)]
        intersection_projection = [cp.Parameter(num_control_pts) for _ in range(num_obs)]
        clearance = [cp.Parameter(num_control_pts) for _ in range(num_obs)]
        reference_pts = cp.Parameter((num_control_pts, Dy))
        constraints = [control_pts[0] == reference_pts[0], control_pts[-1] == reference_pts[-1]]

        vel = (control_pts[1:] - control_pts[:-1])  # (num_control_pts - 1, Dy)
        acc = (vel[1:] - vel[:-1])                  # (num_control_pts - 2, Dy)
        jerk = (acc[1:] - acc[:-1])                 # (num_control_pts - 3, Dy)
        smoothness_loss = cp.sum(acc ** 2) + cp.sum(jerk ** 2)

        collision_loss = 0
        for dir, intersec_proj, clr in zip(direction, intersection_projection, clearance):
            # dist = (control_pts - intersection_pts).dot(direction to reference on traj), shape: (num_control_pts,)
            dist = cp.sum(cp.multiply(control_pts, dir), axis=1) - intersec_proj
            dist_error = clr - dist
            dist_loss = cp.maximum(0, dist_error) ** 2
            collision_loss += cp.sum(dist_loss)

        knot_dt = self.params.model_params.knot_dt
        demarcation = self.params.optimization_params.demarcation
        max_vel = self.params.optimization_params.max_vel
        max_acc = self.params.optimization_params.max_acc

        vel /= knot_dt
        acc /= knot_dt ** 2

        if self.SECOND_DERIVATIVE_CONTINOUS:
            a, b, c = 3 * demarcation, -3 * demarcation ** 2, demarcation ** 3
            raise NotImplementedError("wait for ego_planner response")
        else:
            vel = cp.abs(vel)
            vel_feasibility_loss = cp.sum((cp.maximum(0, vel - max_vel)) ** 2)

            acc = cp.abs(acc)
            acc_feasibility_loss = cp.sum((cp.maximum(0, acc - max_acc)) ** 2)

            # extra "/ knot_dt ** 2": from ego_planner, to make vel and acc have similar magnitudes
            feasibility_loss = vel_feasibility_loss / knot_dt ** 2 + acc_feasibility_loss

        lambda_smooth = self.params.optimization_params.lambda_smoothness
        lambda_coll = self.params.optimization_params.lambda_collision
        lambda_feas = self.params.optimization_params.lambda_feasibility
        loss = lambda_smooth * smoothness_loss + lambda_coll * collision_loss + lambda_feas * feasibility_loss

        objective = cp.Minimize(loss)
        problem = cp.Problem(objective, constraints)
        assert problem.is_dpp()

        parameters = direction + intersection_projection + clearance + [reference_pts]
        self.cvxpylayer = CvxpyLayer(problem, parameters=parameters, variables=[control_pts])

    def forward(self, init_control_pts, obs_loc, obs_size, reference_pts):
        # direction: normalized, from obs_loc to reference_pts
        # shape: (B, num_obs, num_control_pts, Dy)
        batch_size, num_control_pts, Dy = list(reference_pts.size())
        diff = reference_pts.view(batch_size, 1, -1, Dy) - obs_loc.view(batch_size, -1, 1, Dy)
        diff_norm = torch.clamp(torch.linalg.norm(diff, dim=-1, keepdims=True), min=EPS)
        direction = diff / diff_norm

        # intersection = obs_loc + t * direction. denote direction = (x, y, z) and obs_size = (a, b, c)
        # then (t * x)^2 / a^2 + (t * y)^2 / b^2 + (t * z)^2 / c^2 = 1
        # shape: (B, num_obs, num_control_pts)
        obs_size = obs_size.view(batch_size, -1, 1, Dy)
        t_inv = torch.sqrt(torch.sum(direction ** 2 / obs_size ** 2, dim=-1))
        t_inv = torch.clamp(t_inv, min=EPS)
        t = 1 / t_inv
        # intersection_projection = <intersection, direction>
        intersection_projection = torch.sum(obs_loc[:, :, None] * direction, dim=-1) + t

        dist = init_control_pts[:, None] - obs_loc[:, :, None]                  # (B, num_obs, num_control_pts, Dy)
        dist_len = torch.norm(dist, dim=-1)                                     # (B, num_obs, num_control_pts)
        dist_len_clamped = torch.clamp(dist_len[..., None], min=EPS)            # (B, num_obs, num_control_pts, 1)
        dist_direction = dist / dist_len_clamped                                # (B, num_obs, num_control_pts, Dy)
        radius_along_dir_inv = torch.sqrt(torch.sum(dist_direction ** 2 / obs_size ** 2, dim=-1))
        radius_along_dir_inv = torch.clamp(radius_along_dir_inv, min=EPS)       # (B, num_obs, num_control_pts)
        radius_along_dir = 1 / radius_along_dir_inv

        in_collision = dist_len <= radius_along_dir + self.params.optimization_params.clearance
        # in_collision = torch.ones_like(intersection_projection)

        direction = direction * in_collision[..., None]
        intersection_projection = intersection_projection * in_collision
        clearance = self.params.optimization_params.clearance
        clearance = in_collision * clearance                                    # (B, num_obs, num_control_pts)

        direction = torch.unbind(direction, dim=1)
        intersection_projection = torch.unbind(intersection_projection, dim=1)
        clearance = torch.unbind(clearance, dim=1)
        parameters = direction + intersection_projection + clearance + (reference_pts,)

        recon_control_points, = self.cvxpylayer(*parameters)

        return recon_control_points[None, ...]

    def loss(self, init_control_pts, obs_loc, obs_size, reference_pts, final_control_pts):
        # ============================================ set up ============================================ #
        # direction: normalized, from obs_loc to reference_pts
        # shape: (B, num_obs, num_control_pts, Dy)
        batch_size, num_control_pts, Dy = list(reference_pts.size())
        diff = reference_pts.view(batch_size, 1, -1, Dy) - obs_loc.view(batch_size, -1, 1, Dy)
        diff_norm = torch.clamp(torch.linalg.norm(diff, dim=-1, keepdims=True), min=EPS)
        direction = diff / diff_norm

        # intersection = obs_loc + t * direction. denote direction = (x, y, z) and obs_size = (a, b, c)
        # then (t * x)^2 / a^2 + (t * y)^2 / b^2 + (t * z)^2 / c^2 = 1
        # shape: (B, num_obs, num_control_pts)
        obs_size = obs_size.view(batch_size, -1, 1, Dy)
        t_inv = torch.sqrt(torch.sum(direction ** 2 / obs_size ** 2, dim=-1))
        t_inv = torch.clamp(t_inv, min=EPS)
        t = 1 / t_inv
        # intersection_projection = <intersection, direction>
        intersection_projection = torch.sum(obs_loc[:, :, None] * direction, dim=-1) + t

        dist = init_control_pts[:, None] - obs_loc[:, :, None]                  # (B, num_obs, num_control_pts, Dy)
        dist_len = torch.norm(dist, dim=-1)                                     # (B, num_obs, num_control_pts)
        dist_len_clamped = torch.clamp(dist_len[..., None], min=EPS)            # (B, num_obs, num_control_pts, 1)
        dist_direction = dist / dist_len_clamped                                # (B, num_obs, num_control_pts, Dy)
        radius_along_dir_inv = torch.sqrt(torch.sum(dist_direction ** 2 / obs_size ** 2, dim=-1))
        radius_along_dir_inv = torch.clamp(radius_along_dir_inv, min=EPS)       # (B, num_obs, num_control_pts)
        radius_along_dir = 1 / radius_along_dir_inv

        in_collision = dist_len <= radius_along_dir + self.params.optimization_params.clearance
        # in_collision = torch.ones_like(intersection_projection)

        direction = direction * in_collision[..., None]
        intersection_projection = intersection_projection * in_collision
        clearance = self.params.optimization_params.clearance
        clearance = in_collision * clearance                                    # (B, num_obs, num_control_pts)

        direction = torch.unbind(direction, dim=1)
        intersection_projection = torch.unbind(intersection_projection, dim=1)
        clearance = torch.unbind(clearance, dim=1)

        # ============================================ loss ============================================ #

        vel = (final_control_pts[:, 1:] - final_control_pts[:, :-1])    # (B, num_control_pts - 1, Dy)
        acc = (vel[:, 1:] - vel[:, :-1])                                # (B, num_control_pts - 2, Dy)
        jerk = (acc[:, 1:] - acc[:, :-1])                               # (B, num_control_pts - 3, Dy)
        smoothness_loss = torch.sum(acc ** 2, dim=(1, 2)) + torch.sum(jerk ** 2, dim=(1, 2))

        collision_loss = torch.zeros(batch_size, dtype=torch.float32).to(self.params.device)
        for dir, intersec_proj, clr in zip(direction, intersection_projection, clearance):
            # dist = (control_pts - intersection_pts).dot(direction to reference on traj), shape: (B, num_control_pts)
            dist = torch.sum(final_control_pts * dir, dim=-1) - intersec_proj
            dist_error = clr - dist
            dist_loss = torch.clamp(dist_error, min=0) ** 2
            collision_loss += torch.sum(dist_loss, dim=-1)

        knot_dt = self.params.model_params.knot_dt
        demarcation = self.params.optimization_params.demarcation
        max_vel = self.params.optimization_params.max_vel
        max_acc = self.params.optimization_params.max_acc

        vel /= knot_dt
        acc /= knot_dt ** 2

        if self.SECOND_DERIVATIVE_CONTINOUS:
            a, b, c = 3 * demarcation, -3 * demarcation ** 2, demarcation ** 3
            raise NotImplementedError("wait for ego_planner response")
        else:
            vel = torch.abs(vel)
            vel_feasibility_loss = torch.sum(torch.clamp(vel - max_vel, min=0) ** 2, dim=(1, 2))

            acc = torch.abs(acc)
            acc_feasibility_loss = torch.sum(torch.clamp(acc - max_acc, min=0) ** 2, dim=(1, 2))

            # extra "/ knot_dt ** 2": from ego_planner, to make vel and acc have similar magnitudes
            feasibility_loss = vel_feasibility_loss / knot_dt ** 2 + acc_feasibility_loss

        lambda_smooth = self.params.optimization_params.lambda_smoothness
        lambda_coll = self.params.optimization_params.lambda_collision
        lambda_feas = self.params.optimization_params.lambda_feasibility
        loss = lambda_smooth * smoothness_loss + lambda_coll * collision_loss + lambda_feas * feasibility_loss

        return loss, (lambda_smooth * smoothness_loss, lambda_coll * collision_loss, lambda_feas * feasibility_loss)
