import os
import pickle
import shutil
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from joblib import Parallel, delayed


class Params:
    LfH_dir = "2021-02-24-14-38-51_model_1000"
    n_render_per_hallucination = 1

    # for additional obstable generation
    n_pts_to_consider = 10
    n_additional_obs = 5
    loc_radius = 3.0
    loc_span = 270
    size_min, size_max = 0.2, 0.5
    vel_time = 1.0
    vel_span = 120

    # for observation rendering
    laser_max_range = 2.5
    laser_span = 270
    n_scan = 720
    plot_freq = 50

    # post process params
    loc_span = loc_span * np.pi / 180
    vel_span = vel_span * np.pi / 180
    laser_span = laser_span * np.pi / 180


def angle_diff(angle1, angle2):
    diff = np.abs(angle1 - angle2)
    if diff > np.pi:
        diff = 2 * np.pi - diff
    return diff


def find_raycast(pos, ang, loc, size):
    x, y = pos
    c, s = np.cos(ang), np.sin(ang)
    h, v = loc
    a, b = size
    A = c ** 2 / a ** 2 + s ** 2 / b ** 2
    B = (2 * c * (x - h) / a ** 2 + 2 * s * (y - v) / b ** 2)
    C = (x - h) ** 2 / a ** 2 + (y - v) ** 2 / b ** 2 - 1
    delta = B ** 2 - 4 * A * C
    if delta >= 0:
        l1 = (-B - np.sqrt(delta)) / (2 * A)
        l2 = (-B + np.sqrt(delta)) / (2 * A)
        if l1 < 0 and l2 < 0:
            return None
        if l1 > 0 and l2 > 0:
            return np.minimum(l1, l2)
        return np.maximum(l1, l2)
    else:
        return None


def is_colliding_w_traj(poses, vels, loc, size, vel_time, vel_span):
    """

    :param poses: (N, 2) trajectory positions
    :param vels: (N, 2) trajectory velocities
    :param loc: (2,) obs location
    :param size: (2,) obs size
    :param vel_time, vel_span: colliding if obs within radius = vel * vel_time and angle in vel_span
    :return:
    """
    loc_x, loc_y = loc
    a, b = size
    for pos, vel in zip(poses, vels):
        pos_x, pos_y = pos
        if (loc_x - pos_x) ** 2 / a ** 2 + (loc_y - pos_y) ** 2 / b ** 2 <= 1:
            return True

        vel_norm = np.linalg.norm(vel)
        clearance = vel_norm * vel_time

        vel_ang = np.arctan2(vel[1], vel[0])
        loc_dir = np.arctan2(loc_y - pos_y, loc_x - pos_x)

        if angle_diff(loc_dir, vel_ang) < vel_span / 2:
            loc_dis = np.linalg.norm(loc - pos)
            loc_dir = np.array([np.cos(loc_dir), np.sin(loc_dir)])
            rad = 1 / np.sqrt((loc_dir ** 2 / size ** 2).sum())
            if loc_dis - rad <= clearance:
                return True
        else:
            # check if left and right side of the circular sector are colliding
            left_ang, right_ang = vel_ang + vel_span / 2, vel_ang - vel_span / 2
            left_l = find_raycast(pos, left_ang, loc, size)
            if left_l is not None and left_l <= clearance:
                return True
            right_l = find_raycast(pos, right_ang, loc, size)
            if right_l is not None and right_l <= clearance:
                return True
    return False


def generate_additional_obs_2D(traj, params):
    pos, vel = traj
    pt_indexes = np.round(np.linspace(0, len(pos) - 1, params.n_pts_to_consider)).astype(np.int)
    pos, vel = pos[pt_indexes], vel[pt_indexes]

    locs, sizes = [], []

    n_valid_obs = 0
    while n_valid_obs < params.n_additional_obs:
        rad, ang = np.random.uniform(0, params.loc_radius), np.random.uniform(-params.loc_span / 2, params.loc_span / 2)
        loc = np.array([np.cos(ang), np.sin(ang)]) * rad
        size = np.random.uniform(params.size_min, params.size_max, 2)

        if is_colliding_w_traj(pos, vel, loc, size, params.vel_time, params.vel_span):
            continue

        n_valid_obs += 1
        locs.append(loc)
        sizes.append(size)

    return np.array(locs), np.array(sizes)


def render_laser_scan(obs_loc, obs_size, add_obs_loc, add_obs_size, params):
    """

    :param obs_loc: (N, 2)
    :param obs_size: (N, 2)
    :param add_obs_loc: (M, 2)
    :param add_obs_size: (M, 2)
    :param params:
    :return:
    """
    laser_max_range = params.laser_max_range
    laser_span = params.laser_span
    n_scan = params.n_scan

    if params.n_additional_obs > 0:
        locs = np.concatenate([obs_loc, add_obs_loc])
        sizes = np.concatenate([obs_size, add_obs_size])
    else:
        locs, sizes = obs_loc, obs_size

    scans = []
    for theta in np.linspace(-laser_span / 2, laser_span / 2, n_scan):
        scan = laser_max_range
        for loc, size in zip(locs, sizes):
            # solve (c * l - x)^2 / a^2 + (s * l - y)^2 / b^2 = 1
            l = find_raycast([0, 0], theta, loc, size)
            if l is not None:
                scan = np.minimum(scan, l)
        scans.append(scan)

    return np.array(scans)


def plot_render_rslts(obs_loc, obs_size, traj, goal, cmd, add_obs_loc, add_obs_size, laser_scan, params, fig_name):
    plt.figure()

    # plot collision detection
    pos, vel = traj
    vel_span = params.vel_span
    vel_time = params.vel_time
    pt_indexes = np.round(np.linspace(0, len(pos) - 1, params.n_pts_to_consider)).astype(np.int)
    poses, vels = pos[pt_indexes], vel[pt_indexes]
    for pos, vel in zip(poses, vels):
        vel_norm = np.linalg.norm(vel)
        clearance = vel_norm * vel_time
        vel_ang = np.arctan2(vel[1], vel[0])
        vel_arc = np.linspace(vel_ang - vel_span / 2, vel_ang + vel_span / 2, 20)
        sector = pos + clearance * np.stack([np.cos(vel_arc), np.sin(vel_arc)], axis=1)   # (20, 2)
        sector = np.concatenate([pos[None], sector, pos[None]], axis=0)                # (22, 2)
        plt.plot(sector[:, 0], sector[:, 1], color="orange", linestyle="--")

    # plot laser scan
    laser_span = params.laser_span
    laser_angles = np.linspace(-laser_span / 2, laser_span / 2, len(laser_scan))
    for scan, theta in zip(laser_scan[::10], laser_angles[::10]):
        plt.plot([0, scan * np.cos(theta)], [0, scan * np.sin(theta)], color="cyan", alpha=0.5)

    # plot traj
    pos = traj[0]
    plt.plot(pos[:, 0], pos[:, 1], label="traj")
    cmd_vel, cmd_ang_vel = cmd[0], cmd[1]
    cmd_vel = cmd_vel * 0.4
    cmd_ang_vel = cmd_ang_vel * 0.3
    goal = goal * 0.2
    plt.arrow(0, 0, cmd_vel, 0, color="b", width=0.02, label="cmd_vel")
    plt.arrow(cmd[0] / 2.0, -cmd_ang_vel / 2.0, 0, cmd_ang_vel, color="b", width=0.02, label="cmd_ang_vel")
    plt.arrow(0, 0, goal[0], goal[1], color="g", width=0.02, label="goal")

    # plot obstacles
    obses = [Ellipse(xy=loc, width=2 * size[0], height=2 * size[1]) for loc, size in zip(obs_loc, obs_size)]
    for obs in obses:
        plt.gca().add_artist(obs)
        obs.set_alpha(0.5)
        obs.set_facecolor("red")

    add_obses = [Ellipse(xy=loc, width=2 * size[0], height=2 * size[1]) for loc, size in zip(add_obs_loc, add_obs_size)]
    for obs in add_obses:
        plt.gca().add_artist(obs)
        obs.set_alpha(0.5)
        obs.set_facecolor("violet")

    plt.legend(loc="upper left")
    plt.gca().axis("equal")
    plt.xlim([-params.laser_max_range * 1.5, params.laser_max_range * 1.5])
    plt.ylim([-params.laser_max_range * 1.5, params.laser_max_range * 1.5])

    image_dir = os.path.join(params.demo_dir, "plots")
    os.makedirs(image_dir, exist_ok=True)
    plt.savefig(os.path.join(image_dir, fig_name))
    plt.close()


if __name__ == "__main__":
    params = Params()

    repo_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    params.demo_dir = demo_dir = os.path.join(repo_path, "LfH_demo", params.LfH_dir)
    LfH_dir = os.path.join(repo_path, "LfH_eval", params.LfH_dir)

    os.makedirs(demo_dir, exist_ok=True)
    shutil.copyfile(os.path.join(LfH_dir, "params.json"), os.path.join(demo_dir, "params.json"))
    shutil.copyfile(os.path.join(LfH_dir, "LfH_eval.p"), os.path.join(demo_dir, "LfH_eval.p"))
    shutil.copyfile(os.path.join(LfH_dir, "model"), os.path.join(demo_dir, "model"))

    with open(os.path.join(LfH_dir, "LfH_eval.p"), "rb") as f:
        data = pickle.load(f)

    obs_locs = data["obs_loc"]
    obs_sizes = data["obs_size"]
    trajs = data["traj"]
    goals = data["goal"]
    cmds = data["cmd"]

    print("Total samples: {}".format(len(goals)))

    def demo_helper(i, obs_loc, obs_size, traj, goal, cmd):
        demo_ = {"laser": [],
                 "goal": [],
                 "cmd": []}
        for j in range(params.n_render_per_hallucination):
            add_obs_loc, add_obs_size = generate_additional_obs_2D(traj, params)
            laser_scan = render_laser_scan(obs_loc, obs_size, add_obs_loc, add_obs_size, params)
            if i % params.plot_freq == 0 and j == 0:
                plot_render_rslts(obs_loc, obs_size, traj, goal, cmd, add_obs_loc, add_obs_size, laser_scan, params,
                                  fig_name="{}_{}".format(i, j))
            demo_["laser"] = laser_scan
            demo_["goal"] = goal
            demo_["cmd"] = cmd
        return demo_

    demo_list = Parallel(n_jobs=os.cpu_count())(
        delayed(demo_helper)(i, obs_loc, obs_size, traj, goal, cmd)
        for i, (obs_loc, obs_size, traj, goal, cmd) in enumerate(zip(obs_locs, obs_sizes, trajs, goals, cmds))
    )

    demo = {k: np.array([dic[k] for dic in demo_list]).astype(np.float32) for k in demo_list[0]}
    with open(os.path.join(demo_dir, "LfH_demo.p"), "wb") as f:
        pickle.dump(demo, f)
