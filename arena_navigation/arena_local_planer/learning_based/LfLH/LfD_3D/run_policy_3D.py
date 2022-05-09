#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from cv_bridge import CvBridge
from LfH.msg import Bspline
from scipy.interpolate import BSpline as scipy_BSpline

import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from skimage import transform

import torch
from LfD_main import TrainingParams, Encoder, Decoder, Model

model_path = "2021-03-02-03-27-39"
model_fname = "model_1000"
bspline_update_dt = 0.1

# MPC params
collision_check_dt = 0.04
clearance = 0.01

knot_dt_coef_max = 2.0
knot_dt_coef_delta = 0.05
knot_dt_coef_decrease_threshold = 1.8
knot_dt_coef_increase_threshold = 1.5
emergency_time = 1.0

plot_reconstruction = False


class Predictor:
    def __init__(self, model, params):
        self.model = model
        self.bridge = CvBridge()
        self.image_size = params.model_params.image_size
        assert isinstance(self.image_size, (int, tuple))
        if isinstance(self.image_size, int):
            self.image_size = (self.image_size, self.image_size)
        self.max_depth = params.model_params.max_depth
        self.need_update_bspline = False
        self.device = params.device

        model.train(training=False)
        self.depth = None
        self.goal = None
        self.pos = None
        self.ori = None
        self.lin_vel = None
        self.ang_vel = None
        self.depth_pos = None

        model_params = params.model_params
        self.num_control_pts = model_params.knot_end - model_params.knot_start - 3 - 1
        self.knots_base = np.arange(model_params.knot_start, model_params.knot_end, 1) * model_params.knot_dt
        self.knots_base = self.knots_base.astype(np.float64)
        self.knot_dt_coef = knot_dt_coef_max
        self.depth_quat = None

        self.fx = 387.229248046875
        self.fy = 387.229248046875
        self.cx = 321.04638671875
        self.cy = 243.44969177246094
        y_grid, z_grid = np.meshgrid(np.linspace(-clearance, clearance, 5), np.linspace(-clearance, clearance, 5))
        x_grid = np.zeros_like(y_grid)
        self.surface = np.stack([x_grid, y_grid, z_grid], axis=-1).reshape((-1, 3))

        self.in_emergency = False
        self.in_emergency_stop = False          # emergency stop
        self.in_emergency_rotate = False        # rotate in place
        self.out_of_danger_count = 0
        self.out_of_danger_count_threshold = 7
        self.curr_out_of_danger_count_threshold = self.out_of_danger_count_threshold

        self.n_plot = 0                         # for debug

    def update_depth(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth = np.asarray(cv_image)
        self.notify_bspline_update()

    def notify_bspline_update(self):
        if np.any([ele is None for ele in [self.depth, self.goal, self.ori, self.lin_vel, self.ang_vel]]):
            return
        self.need_update_bspline = True

    def update_goal(self, msg):
        goal = msg.poses[0].pose.position
        self.goal = np.array([goal.x, goal.y, goal.z])
        self.notify_bspline_update()

    def update_odom(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.pos = np.array([pos.x, pos.y, pos.z])
        self.ori = np.array([ori.x, ori.y, ori.z, ori.w])

        twist = msg.twist.twist
        self.lin_vel = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
        self.ang_vel = np.array([twist.angular.x, twist.angular.y, twist.angular.z])
        self.notify_bspline_update()

    def update_bspline(self):
        if self.in_emergency:
            control_pts_local, knots = self.handle_emeragency()
        else:
            control_pts_local, knots = self.get_LfD_bspline()
            in_danger, control_pts_local, knots = self.check_bspline_safety(control_pts_local, knots)
            if in_danger:
                print "[INFO] Emergency Stop"
                self.enter_emergency()
                control_pts_local, knots = self.handle_emeragency()

        r_inv = R.from_quat(self.ori)
        control_pts = r_inv.apply(control_pts_local) + self.pos
        knots = list(knots)
        return control_pts, knots

    def handle_emeragency(self):
        assert self.in_emergency_stop or self.in_emergency_rotate
        control_pts_local, knots = self.get_LfD_bspline()
        in_danger, control_pts_local, knots = self.check_bspline_safety(control_pts_local, knots)
        if not in_danger:
            self.out_of_danger_count += 1
            if self.out_of_danger_count >= self.curr_out_of_danger_count_threshold:
                self.exit_emergency()
        else:
            self.out_of_danger_count = 0

        if self.in_emergency:
            if self.in_emergency_stop:
                print "[INFO] In Stopping, current vel", np.linalg.norm(self.lin_vel)
                if np.linalg.norm(self.lin_vel) < 0.02:
                    # transit to emergency rotate
                    self.in_emergency_stop = False
                    self.in_emergency_rotate = True
                    self.curr_out_of_danger_count_threshold = 1
                return np.zeros((6, 3)), np.arange(-3, 7)
            elif self.in_emergency_rotate:
                # handle emergency rotate
                r = R.from_quat(self.ori).inv()
                goal_local = r.apply(self.goal - self.pos)
                turn_left = goal_local[1] > 0
                # not safe yet, rotate in small circle
                for turn_dir in [turn_left, not turn_left]:
                    for radius in np.linspace(0.5, 0.0, 10, endpoint=False):
                        # turn on a circular arc
                        n_control_pts = 6
                        theta = np.linspace(-np.pi * 2 / 3, np.pi / 6, n_control_pts)
                        y_center = radius
                        if not turn_left:
                            theta *= -1
                            y_center *= -1
                        x = np.cos(theta) * radius
                        y = np.sin(theta) * radius + y_center
                        z = np.zeros(n_control_pts)
                        control_pts_local = np.stack([x, y, z], axis=-1)
                        knots = np.arange(-3, 1 + n_control_pts) * 0.5

                        in_danger, _, _ = self.check_bspline_safety(control_pts_local, knots)
                        if not in_danger:
                            break
                    if not in_danger:
                        break
                if in_danger:
                    print "[INFO] Rotate in place fail"
        return control_pts_local, knots

    def get_LfD_bspline(self):
        r = R.from_quat(self.ori).inv()
        goal_local = r.apply(self.goal - self.pos)
        goal_local /= np.linalg.norm(goal_local)
        goal_local = torch.from_numpy(goal_local[None].astype(np.float32)).to(self.device)

        lin_vel = r.apply(self.lin_vel)
        ang_vel = r.apply(self.ang_vel)
        lin_vel = torch.from_numpy(lin_vel[None].astype(np.float32)).to(self.device)
        ang_vel = torch.from_numpy(ang_vel[None].astype(np.float32)).to(self.device)

        depth = transform.resize(self.depth, self.image_size)

        depth = np.clip(depth, 0, self.max_depth)
        depth /= self.max_depth
        depth_4_plot = depth
        depth = torch.from_numpy(depth[None, None].astype(np.float32),).to(self.device)

        with torch.no_grad():
            # control_pts_local = self.model(depth, goal_local, lin_vel, ang_vel)
            control_pts_local  = self.model(depth, goal_local, lin_vel, ang_vel, reconstruct_depth=plot_reconstruction)
            if plot_reconstruction:
                control_pts_local, reconstructed_depth = control_pts_local
                reconstructed_depth = reconstructed_depth[0, 0].cpu().numpy()
                plt.figure(figsize=(6, 3))
                plt.imshow(np.concatenate([depth_4_plot, reconstructed_depth], axis=1))
                plt.title("AE loss: {0:.3f}".format(((depth_4_plot - reconstructed_depth) ** 2).sum()))
                plt.colorbar()
                plot_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "plots")
                if not os.path.exists(plot_path):
                    os.makedirs(plot_path)
                plt.savefig(os.path.join(plot_path, str(self.n_plot)))
                self.n_plot += 1
                plt.close()
        control_pts_local = control_pts_local[0].cpu().numpy()  # get rid of the batch size dim
        control_pts_local = control_pts_local[:self.num_control_pts]
        knots = self.knots_base * self.knot_dt_coef
        return control_pts_local, knots

    def check_bspline_safety(self, control_pts, knots):
        bspline = scipy_BSpline(knots, control_pts, 3)
        traj_end_t = knots[-4]
        t_to_check = np.arange(0, traj_end_t * 2 / 3, collision_check_dt)
        positions = bspline(t_to_check)
        in_collision = self.check_collision(positions, self.depth)
        collision_idxes = np.where(in_collision)[0]
        if len(collision_idxes):
            t_col = t_to_check[collision_idxes[0]]
        else:
            t_col = t_to_check[-1]

        # adjust self.knot_dt_coef
        if not self.in_emergency:
            prev_knot_dt_coef = self.knot_dt_coef
            if t_col >= knot_dt_coef_decrease_threshold:
                self.knot_dt_coef -= knot_dt_coef_delta
                status = "decreased"
            if t_col <= knot_dt_coef_increase_threshold:
                self.knot_dt_coef += knot_dt_coef_delta
                status = "increased"
            self.knot_dt_coef = np.clip(self.knot_dt_coef, 1.0, knot_dt_coef_max)
            if prev_knot_dt_coef != self.knot_dt_coef:
                print "[INFO] Knot coef {0} to {1:.2f} due to t_col = {2:.2f}".format(status, self.knot_dt_coef, t_col)

        if t_col <= emergency_time:
            return True, None, None
        else:
            return False, control_pts, self.knots_base * self.knot_dt_coef

    def check_collision(self, positions, depth):
        positions = positions[:, None] + self.surface                                   # (T, N_surface, 3)
        xs, ys, zs = positions[:, :, 0], positions[:, :, 1], positions[:, :, 2]
        projected_xs = (-ys / xs * self.fx + self.cx + 0.5).astype(np.intp)             # (T, N_surface)
        projected_ys = (-zs / xs * self.fy + self.cy + 0.5).astype(np.intp)             # (T, N_surface)
        invalid_mask = (projected_xs < 0) | (projected_xs >= 640) | (projected_ys < 0) | (projected_ys >= 480)
        projected_xs = np.clip(projected_xs, 0, 640 - 1)
        projected_ys = np.clip(projected_ys, 0, 480 - 1)
        distance = depth[projected_ys, projected_xs]                                    # (T, N_surface)
        distance[invalid_mask] = np.inf
        # mask for ego-planner
        distance[distance == 0] = np.inf
        in_collision = distance < xs
        in_collision = in_collision.any(axis=-1)                                        # (T,)
        return in_collision

    def enter_emergency(self):
        self.knot_dt_coef = knot_dt_coef_max
        self.in_emergency = True
        self.in_emergency_stop = True
        self.in_emergency_rotate = False
        self.out_of_danger_count = 0
        self.curr_out_of_danger_count_threshold = self.out_of_danger_count_threshold

    def exit_emergency(self):
        self.in_emergency = False
        self.in_emergency_stop = False
        self.in_emergency_rotate = False
        self.out_of_danger_count = 0
        self.curr_out_of_danger_count_threshold = self.out_of_danger_count_threshold


if __name__ == '__main__':
    repo_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    params_path = os.path.join(repo_path, "rslts", "LfD_3D_rslts", model_path, "params.json")
    model_path = os.path.join(repo_path, "rslts", "LfD_3D_rslts", model_path, "trained_models", model_fname)

    params = TrainingParams(params_path, train=False)
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    params.device = device

    encoder = Encoder(params).to(device)
    decoder = Decoder(params).to(device)
    model = Model(params, encoder, decoder).to(device)
    assert os.path.exists(model_path)
    model.load_state_dict(torch.load(model_path, map_location=device))
    print "[INFO] LfH policy loaded"

    predictor = Predictor(model, params)
    rospy.init_node('LfH_policy', anonymous=True)

    sub_goal = rospy.Subscriber("/waypoint_generator/waypoints", Path, predictor.update_goal)
    sub_depth = rospy.Subscriber("/pcl_render_node/depth", Image, predictor.update_depth, queue_size=1)
    sub_scan = rospy.Subscriber("/visual_slam/odom", Odometry, predictor.update_odom, queue_size=1)
    bspline_pub = rospy.Publisher('/planning/bspline', Bspline, queue_size=1)

    traj_id = 0
    prev_msg_time = None
    print_time = rospy.get_time()
    while not rospy.is_shutdown():
        try:
            now = rospy.get_time()
            if (prev_msg_time is None or now - prev_msg_time >= bspline_update_dt) and predictor.need_update_bspline:
                traj_id += 1
                bspline_msg = Bspline()
                bspline_msg.start_time = rospy.Time.now()
                bspline_msg.order = 3
                bspline_msg.traj_id = traj_id
                pos_pts = []
                control_pts, knots = predictor.update_bspline()
                bspline_msg.knots = knots
                for control_pt in control_pts:
                    pt = Point()
                    pt.x, pt.y, pt.z = control_pt.astype(np.float64)
                    pos_pts.append(pt)
                bspline_msg.pos_pts = pos_pts
                bspline_pub.publish(bspline_msg)

                if prev_msg_time is not None and now - print_time > 5:
                    print_time = now
                    print "[INFO] Update bspline, frequency = %d Hz" % int(1.0 / (now - prev_msg_time))
                predictor.need_update_bspline = False
                prev_msg_time = now
        except rospy.exceptions.ROSInterruptException:
            break
