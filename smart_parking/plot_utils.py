import os
import matplotlib.pyplot as plt


def plot_vals(vals, save_path="plots/xpos_movement.png"):
    """
    Plot the forward movement position in x.
    """

    if len(vals) == 0:
        return

    os.makedirs(os.path.dirname(save_path), exist_ok=True)

    plt.figure()
    plt.plot(vals)

    plt.xlabel("Sample index")
    plt.ylabel("x [m]")

    plt.grid(True)
    plt.tight_layout()

    plt.savefig(save_path)
    plt.close()