import numpy as np
import matplotlib.pyplot as plt


def soft_update_params(net, target_net, tau):
    for param, target_param in zip(net.parameters(), target_net.parameters()):
        target_param.data.copy_(
            tau * param.data + (1 - tau) * target_param.data
        )


def plot_AE(depths, reconstructed_depths):
    assert depths.shape == reconstructed_depths.shape
    assert depths.ndim == 4
    batch_size = depths.shape[0]
    n_depths = 3
    if batch_size <= n_depths:
        idx = 0
    else:
        idx = np.random.randint(batch_size - n_depths)
    depths = depths[idx:idx + n_depths, 0].detach().cpu().numpy()
    reconstructed_depths = reconstructed_depths[idx:idx + 3, 0].detach().cpu().numpy()
    fig = plt.figure(figsize=(3 * len(depths), 6))
    for i, (depth, reconstructed_depth) in enumerate(zip(depths, reconstructed_depths)):
        ax = fig.add_subplot(1, len(depths), i + 1, xticks=[], yticks=[])
        ax.imshow(np.concatenate([depth, reconstructed_depth], axis=0))
        ax.set_title("AE loss: {0:.3f}".format(((depth - reconstructed_depth) ** 2).sum()))
    plt.tight_layout()
    return fig
