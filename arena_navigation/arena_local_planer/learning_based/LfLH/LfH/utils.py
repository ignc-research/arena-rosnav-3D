import os
import sys
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


def to_numpy(tensor):
    return tensor.cpu().detach().numpy()


def set_axes_equal(ax):
    """Set 3D plot axes to equal scale.

    Make axes of 3D plot have equal scale so that spheres appear as
    spheres and cubes as cubes.  Required since `ax.axis('equal')`
    and `ax.set_aspect('equal')` don't work on 3D.
    """
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])
    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    _set_axes_radius(ax, origin, radius)


def _set_axes_radius(ax, origin, radius):
    x, y, z = origin
    ax.set_xlim3d([x - radius, x + radius])
    ax.set_ylim3d([y - radius, y + radius])
    ax.set_zlim3d([z - radius, z + radius])


def draw_ellipsoid(loc, size, ax, color, alpha=0.2):
    x, y, z = loc
    rx, ry, rz = size

    # Set of all spherical angles:
    u = np.linspace(0, 2 * np.pi, 50)
    v = np.linspace(0, np.pi, 50)

    # Cartesian coordinates that correspond to the spherical angles:
    # (this is the equation of an ellipsoid):
    sx = x + rx * np.outer(np.cos(u), np.sin(v))
    sy = y + ry * np.outer(np.sin(u), np.sin(v))
    sz = z + rz * np.outer(np.ones_like(u), np.cos(v))

    # Plot:
    ax.plot_surface(sx, sy, sz, rstride=4, cstride=4, color=color, alpha=alpha)


# ---------------------------------- TO CREATE A SERIES OF PICTURES ---------------------------------- #
# from https://zulko.wordpress.com/2012/09/29/animate-your-3d-plots-with-pythons-matplotlib/

def make_views(ax, angles, elevation=None, width=4, height=3,
               prefix='tmprot_', **kwargs):
    """
    Makes jpeg pictures of the given 3d ax, with different angles.
    Args:
        ax (3D axis): te ax
        angles (list): the list of angles (in degree) under which to
                       take the picture.
        width,height (float): size, in inches, of the output images.
        prefix (str): prefix for the files created.

    Returns: the list of files created (for later removal)
    """

    files = []
    ax.figure.set_size_inches(width, height)

    for i, angle in enumerate(angles):
        ax.view_init(elev=elevation, azim=angle)
        fname = '%s%03d.jpeg' % (prefix, i)
        ax.figure.savefig(fname)
        files.append(fname)

    return files


# ----------------------- TO TRANSFORM THE SERIES OF PICTURE INTO AN ANIMATION ----------------------- #

def make_movie(files, output, fps=10, bitrate=1800, **kwargs):
    """
    Uses mencoder, produces a .mp4/.ogv/... movie from a list of
    picture files.
    """

    output_name, output_ext = os.path.splitext(output)
    command = {'.mp4': 'mencoder "mf://%s" -mf fps=%d -o %s.mp4 -ovc lavc\
                         -lavcopts vcodec=msmpeg4v2:vbitrate=%d'
                       % (",".join(files), fps, output_name, bitrate)}

    command['.ogv'] = command['.mp4'] + '; ffmpeg -i %s.mp4 -r %d %s' % (output_name, fps, output)

    print(command[output_ext])
    output_ext = os.path.splitext(output)[1]
    os.system(command[output_ext])


def make_gif(files, output, delay=100, repeat=True, **kwargs):
    """
    Uses imageMagick to produce an animated .gif from a list of
    picture files.
    """

    loop = -1 if repeat else 0
    os.system('convert -delay %d -loop %d %s %s' % (delay, loop, " ".join(files), output))


def make_strip(files, output, **kwargs):
    """
    Uses imageMagick to produce a .jpeg strip from a list of
    picture files.
    """

    os.system('montage -tile 1x -geometry +0+0 %s %s' % (" ".join(files), output))


##### MAIN FUNCTION

def rotanimate(ax, angles, output, **kwargs):
    """
    Produces an animation (.mp4,.ogv,.gif,.jpeg,.png) from a 3D plot on
    a 3D ax

    Args:
        ax (3D axis): the ax containing the plot of interest
        angles (list): the list of angles (in degree) under which to
                       show the plot.
        output : name of the output file. The extension determines the
                 kind of animation used.
        **kwargs:
            - width : in inches
            - heigth: in inches
            - framerate : frames per second
            - delay : delay between frames in milliseconds
            - repeat : True or False (.gif only)
    """

    output_ext = os.path.splitext(output)[1]

    files = make_views(ax, angles, **kwargs)

    D = {'.mp4': make_movie,
         '.ogv': make_movie,
         '.gif': make_gif,
         '.jpeg': make_strip,
         '.png': make_strip}

    D[output_ext](files, output, **kwargs)

    for f in files:
        os.remove(f)


def plot_opt(writer, reference_pts, recon_control_points, loc, size, epoch, is_bug=False, num_sample=3):
    batch_size, num_obstacle, Dy = loc.size()
    if Dy not in [2, 3]:
        pass

    if batch_size <= num_sample:
        idx = 0
        num_sample = min(batch_size, num_sample)
    else:
        idx = np.random.randint(batch_size - num_sample)

    reference_pts = reference_pts[idx:idx + num_sample]
    recon_control_points = recon_control_points[:, idx:idx + num_sample]
    loc = loc[idx:idx + num_sample]
    size = size[idx:idx + num_sample]

    loc = to_numpy(loc)
    size = to_numpy(size)
    recon_control_points = to_numpy(recon_control_points)
    reference_pts = to_numpy(reference_pts)

    fig = plt.figure(figsize=(5 * num_sample, 5))
    colors = sns.color_palette('husl', n_colors=num_obstacle + 1)
    for i in range(num_sample):
        if Dy == 2:
            ax = fig.add_subplot(1, num_sample, i + 1)
            ax.plot(reference_pts[i, :, 0], reference_pts[i, :, 1], label="reference")
            ax.scatter(reference_pts[i, :, 0], reference_pts[i, :, 1])

            obses = [Ellipse(xy=loc_, width=2 * size_[0], height=2 * size_[1]) for loc_, size_ in zip(loc[i], size[i])]
            for j, obs in enumerate(obses):
                ax.add_artist(obs)
                obs.set_alpha(0.5)
                obs.set_facecolor(colors[j])

            ode_num_timestamps = recon_control_points.shape[0]
            for j in range(ode_num_timestamps):
                ax.plot(recon_control_points[j, i, :, 0], recon_control_points[j, i, :, 1], label="opt_{}".format(j))
                ax.scatter(recon_control_points[j, i, :, 0], recon_control_points[j, i, :, 1])

            x_min = np.minimum(np.min(reference_pts[i, :, 0]), np.min(recon_control_points[:, i, :, 0]))
            x_max = np.maximum(np.max(reference_pts[i, :, 0]), np.max(recon_control_points[:, i, :, 0]))
            y_min = np.minimum(np.min(reference_pts[i, :, 1]), np.min(recon_control_points[:, i, :, 1]))
            y_max = np.maximum(np.max(reference_pts[i, :, 1]), np.max(recon_control_points[:, i, :, 1]))
            x_mid, y_mid = (x_max + x_min) / 2, (y_max + y_min) / 2
            x_range, y_range = x_max - x_min, y_max - y_min
            x_min, x_max = x_mid - 1.5 * x_range, x_mid + 1.5 * x_range
            y_min, y_max = y_mid - 1.5 * y_range, y_mid + 1.5 * y_range

            ax.axis('equal')
            ax.set_xlabel("x")
            ax.set_ylabel("y")

            ax.set(xlim=[x_min, x_max], ylim=[y_min, y_max])
            ax.legend()
        else:
            ax = fig.add_subplot(1, num_sample, i + 1, projection="3d")
            ax.plot(reference_pts[i, :, 0], reference_pts[i, :, 1], reference_pts[i, :, 2], label="reference")
            ax.scatter(reference_pts[i, :, 0], reference_pts[i, :, 1], reference_pts[i, :, 2])

            for j, (loc_, size_) in enumerate(zip(loc[i], size[i])):
                draw_ellipsoid(loc_, size_, ax, colors[j])

            ode_num_timestamps = recon_control_points.shape[0]
            for j in range(ode_num_timestamps):
                ax.plot(recon_control_points[j, i, :, 0],
                        recon_control_points[j, i, :, 1],
                        recon_control_points[j, i, :, 2],
                        label="opt_{}".format(j))
                ax.scatter(recon_control_points[j, i, :, 0],
                           recon_control_points[j, i, :, 1],
                           recon_control_points[j, i, :, 2])

            x_min = np.minimum(np.min(reference_pts[i, :, 0]), np.min(recon_control_points[:, i, :, 0]))
            x_max = np.maximum(np.max(reference_pts[i, :, 0]), np.max(recon_control_points[:, i, :, 0]))
            y_min = np.minimum(np.min(reference_pts[i, :, 1]), np.min(recon_control_points[:, i, :, 1]))
            y_max = np.maximum(np.max(reference_pts[i, :, 1]), np.max(recon_control_points[:, i, :, 1]))
            z_min = np.minimum(np.min(reference_pts[i, :, 2]), np.min(recon_control_points[:, i, :, 2]))
            z_max = np.maximum(np.max(reference_pts[i, :, 2]), np.max(recon_control_points[:, i, :, 2]))
            x_mid, y_mid, z_mid = (x_max + x_min) / 2, (y_max + y_min) / 2, (z_max + z_min) / 2
            x_range, y_range, z_range = x_max - x_min, y_max - y_min, z_max - z_min
            x_min, x_max = x_mid - 1.0 * x_range, x_mid + 1.0 * x_range
            y_min, y_max = y_mid - 1.0 * y_range, y_mid + 1.0 * y_range
            z_min, z_max = z_mid - 1.0 * z_range, z_mid + 1.0 * z_range

            ax.set_xlabel("x")
            ax.set_ylabel("y")
            ax.set_zlabel("z")

            ax.set(xlim=[x_min, x_max], ylim=[y_min, y_max], zlim=[z_min, z_max])
            ax.legend()
            set_axes_equal(ax)

    fig.tight_layout()
    writer.add_figure("sample" if not is_bug else "opt_error_logging", fig, epoch)
    plt.close("all")


def plot_obs_dist(writer, params, full_traj, loc_mu, loc_log_var, size_mu, size_log_var, epoch, num_sample=3):
    batch_size, num_obstacle, Dy = loc_mu.size()
    if Dy not in [2, 3]:
        pass
    if batch_size <= num_sample:
        idx = 0
        num_sample = min(batch_size, num_sample)
    else:
        idx = np.random.randint(batch_size - num_sample)

    full_traj = to_numpy(full_traj[idx:idx + num_sample, :Dy])
    loc_prior_mu = np.mean(full_traj, axis=-1)
    loc_prior_var = np.maximum(np.var(full_traj, axis=-1), params.model_params.min_obs_loc_prior_var)
    loc_prior_var *= params.model_params.obs_loc_prior_var_coef
    loc_prior_std = np.sqrt(loc_prior_var)
    loc_mu = to_numpy(loc_mu[idx:idx + num_sample])
    loc_log_var = to_numpy(loc_log_var[idx:idx + num_sample])
    loc_std = np.exp(0.5 * loc_log_var)
    size_mu = to_numpy(size_mu[idx:idx + num_sample])
    size_log_var = to_numpy(size_log_var[idx:idx + num_sample])
    size_std = np.exp(0.5 * size_log_var)

    fig = plt.figure(figsize=(5 * num_sample, 5))

    def softplus(a):
        return np.log(1 + np.exp(a))

    colors = sns.color_palette('husl', n_colors=num_obstacle + 1)
    for i in range(num_sample):
        if Dy == 2:
            ax = fig.add_subplot(1, num_sample, i + 1)

            obs_loc_prior = Ellipse(xy=loc_prior_mu[i], width=2 * loc_prior_std[i, 0], height=2 * loc_prior_std[i, 1],
                                    facecolor='none')
            edge_c = colors[-1]
            ax.add_artist(obs_loc_prior)
            obs_loc_prior.set_edgecolor(edge_c)

            obs_loc = [Ellipse(xy=loc_, width=2 * size_[0], height=2 * size_[1], facecolor='none')
                       for loc_, size_ in zip(loc_mu[i], loc_std[i])]
            obs_size_mu = [Ellipse(xy=loc_, width=2 * size_[0], height=2 * size_[1], facecolor='none')
                           for loc_, size_ in zip(loc_mu[i], softplus(size_mu[i]))]
            obs_size_s = [Ellipse(xy=loc_, width=2 * size_[0], height=2 * size_[1], facecolor='none')
                          for loc_, size_ in zip(loc_mu[i], softplus(size_mu[i] - size_std[i]))]
            obs_size_l = [Ellipse(xy=loc_, width=2 * size_[0], height=2 * size_[1], facecolor='none')
                          for loc_, size_ in zip(loc_mu[i], softplus(size_mu[i] + size_std[i]))]
            for j, (loc_, size_mu_, size_s, size_l) in enumerate(zip(obs_loc, obs_size_mu, obs_size_s, obs_size_l)):
                edge_c = colors[j]
                ax.add_artist(loc_)
                ax.add_artist(size_mu_)
                ax.add_artist(size_s)
                ax.add_artist(size_l)
                loc_.set_edgecolor(edge_c)
                size_mu_.set_edgecolor(edge_c)
                size_s.set_edgecolor(edge_c)
                size_l.set_edgecolor(edge_c)
                loc_.set_linestyle('--')

            x_min = np.min([loc_prior_mu[i, 0] - loc_prior_std[i, 0],
                            np.min(loc_mu[i, :, 0] - loc_std[i, :, 0]),
                            np.min(loc_mu[i, :, 0] - softplus(size_mu[i, :, 0] + size_std[i, :, 0]))])
            x_max = np.max([loc_prior_mu[i, 0] + loc_prior_std[i, 0],
                            np.max(loc_mu[i, :, 0] + loc_std[i, :, 0]),
                            np.max(loc_mu[i, :, 0] + softplus(size_mu[i, :, 0] + size_std[i, :, 0]))])
            y_min = np.min([loc_prior_mu[i, 1] - loc_prior_std[i, 1],
                            np.min(loc_mu[i, :, 1] - loc_std[i, :, 1]),
                            np.min(loc_mu[i, :, 1] - softplus(size_mu[i, :, 1] + size_std[i, :, 1]))])
            y_max = np.max([loc_prior_mu[i, 1] + loc_prior_std[i, 1],
                            np.max(loc_mu[i, :, 1] + loc_std[i, :, 1]),
                            np.max(loc_mu[i, :, 1] + softplus(size_mu[i, :, 1] + size_std[i, :, 1]))])
            x_mid, y_mid = (x_max + x_min) / 2, (y_max + y_min) / 2
            x_range, y_range = x_max - x_min, y_max - y_min
            x_min, x_max = x_mid - x_range * 0.75, x_mid + x_range * 0.75
            y_min, y_max = y_mid - y_range * 0.75, y_mid + y_range * 0.75

            ax.axis('equal')
            ax.set(xlim=[x_min, x_max], ylim=[y_min, y_max])
        else:
            ax = fig.add_subplot(1, num_sample, i + 1, projection="3d")
            draw_ellipsoid(loc_prior_mu[i], loc_prior_std[i], ax, colors[-1], alpha=0.6)

            for j, (loc_, size_) in enumerate(zip(loc_mu[i], loc_std[i])):
                draw_ellipsoid(loc_, size_, ax, colors[j])
            for j, (loc_, size_) in enumerate(zip(loc_mu[i], softplus(size_mu[i]))):
                draw_ellipsoid(loc_, size_, ax, colors[j])

            x_min = np.min([loc_prior_mu[i, 0] - loc_prior_std[i, 0],
                            np.min(loc_mu[i, :, 0] - loc_std[i, :, 0]),
                            np.min(loc_mu[i, :, 0] - softplus(size_mu[i, :, 0]))])
            x_max = np.max([loc_prior_mu[i, 0] + loc_prior_std[i, 0],
                            np.max(loc_mu[i, :, 0] + loc_std[i, :, 0]),
                            np.max(loc_mu[i, :, 0] + softplus(size_mu[i, :, 0]))])
            y_min = np.min([loc_prior_mu[i, 1] - loc_prior_std[i, 1],
                            np.min(loc_mu[i, :, 1] - loc_std[i, :, 1]),
                            np.min(loc_mu[i, :, 1] - softplus(size_mu[i, :, 1]))])
            y_max = np.max([loc_prior_mu[i, 1] + loc_prior_std[i, 1],
                            np.max(loc_mu[i, :, 1] + loc_std[i, :, 1]),
                            np.max(loc_mu[i, :, 1] + softplus(size_mu[i, :, 1]))])
            z_min = np.min([loc_prior_mu[i, 2] - loc_prior_std[i, 2],
                            np.min(loc_mu[i, :, 2] - loc_std[i, :, 2]),
                            np.min(loc_mu[i, :, 2] - softplus(size_mu[i, :, 2]))])
            z_max = np.max([loc_prior_mu[i, 2] + loc_prior_std[i, 2],
                            np.max(loc_mu[i, :, 2] + loc_std[i, :, 2]),
                            np.max(loc_mu[i, :, 2] + softplus(size_mu[i, :, 2]))])
            x_mid, y_mid, z_mid = (x_max + x_min) / 2, (y_max + y_min) / 2, (z_max + z_min) / 2
            x_range, y_range, z_range = x_max - x_min, y_max - y_min, z_max - z_min
            x_min, x_max = x_mid - 0.75 * x_range, x_mid + 0.75 * x_range
            y_min, y_max = y_mid - 0.75 * y_range, y_mid + 0.75 * y_range
            z_min, z_max = z_mid - 0.75 * z_range, z_mid + 0.75 * z_range

            ax.set_xlabel("x")
            ax.set_ylabel("y")
            ax.set_zlabel("z")

            ax.set(xlim=[x_min, x_max], ylim=[y_min, y_max], zlim=[z_min, z_max])
            set_axes_equal(ax)

    fig.tight_layout()
    writer.add_figure("distribution", fig, epoch)
    plt.close("all")
