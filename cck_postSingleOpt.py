import pickle as pkl
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import tkinter as tk
from tkinter import filedialog
import matplotlib.colors as mcolors

def pick_folder():
    root = tk.Tk()
    root.withdraw()   # Hide the small tkinter window

    folder_path = filedialog.askdirectory(
        title="Select a folder",
        initialdir="/home/ckeefe/logs",
        mustexist=True
    )

    return folder_path


def load_run_hist(run_dir: str | Path):
    """
    Load NES run history from a directory containing run_hist.pkl.
    """
    run_dir = Path(run_dir)
    with open(run_dir / "run_hist.pkl", "rb") as f:
        run_hist = pkl.load(f)
    return run_hist


   
def plot_population_2d_from_hist(run_hist, iteration: int,
    save_path: str | Path | None = None,
):
    pop_hist = run_hist["pop_hist"]        # (iters, pop_size, num_dims)
    pop_rew_hist = run_hist["pop_rew_hist"]
    opt_params = run_hist["opt_params"]

    iters, pop_size, num_dims = pop_hist.shape
    if iteration < 0 or iteration >= iters:
        raise ValueError(f"iteration must be in [0, {iters-1}]")

    rewards = pop_rew_hist[iteration]

    # Debug: confirm values
    print("Reward range for this iteration:", np.min(rewards), np.max(rewards))

    # Global color scale across all iterations 
    vmin = float(np.min(pop_rew_hist))
    vmax = float(np.max(pop_rew_hist))

    norm = mcolors.Normalize(vmin=vmin, vmax=vmax)

    plt.figure()

    # MAIN SCATTER (this is the mappable for the colorbar)
    sc = plt.scatter(
        pop_hist[iteration, :, 0],
        pop_hist[iteration, :, 1],
        c=rewards,
        cmap="viridis",
        norm=norm,
    )

    # Mark the first individual (no norm, no c)
    plt.scatter(
        pop_hist[iteration, 0, 0],
        pop_hist[iteration, 0, 1],
        color="red",
        marker="X",
        s=60,
    )

    cbar = plt.colorbar(sc, label="Reward")
    # Optional: nicer formatting
    # from matplotlib.ticker import FormatStrFormatter
    # cbar.ax.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))

    plt.xlabel(opt_params.param_mask_keys[0])
    plt.ylabel(opt_params.param_mask_keys[1])
    plt.xlim(opt_params.param_min[0] * 0.9, opt_params.param_max[0] * 1.1)
    plt.ylim(opt_params.param_min[1] * 0.9, opt_params.param_max[1] * 1.1)
    plt.title(f"Population at Iteration {iteration}")
    plt.grid(True)

    if save_path is not None:
        plt.savefig(save_path, bbox_inches="tight")
    else:
        plt.show()

    plt.close()
    
def plot_full_population_2d(run_hist, save_path=None):
    import matplotlib.colors as mcolors
    import matplotlib.pyplot as plt
    import numpy as np
    from pathlib import Path

    pop_hist = run_hist["pop_hist"]        # (iters, pop_size, num_dims)
    pop_rew_hist = run_hist["pop_rew_hist"]
    opt_params = run_hist["opt_params"]

    iters, pop_size, num_dims = pop_hist.shape

    # Flatten population history
    all_x = pop_hist[:, :, 0].flatten()
    all_y = pop_hist[:, :, 1].flatten()
    all_rewards = pop_rew_hist.flatten()

    # Global color range
    vmin = float(np.min(all_rewards))
    vmax = float(np.max(all_rewards))
    norm = mcolors.Normalize(vmin=vmin, vmax=vmax)

    plt.figure(figsize=(8, 6))

    sc = plt.scatter(
        all_x,
        all_y,
        c=all_rewards,
        cmap="viridis",
        norm=norm,
        s=10,
        alpha=0.8
    )

    plt.colorbar(sc, label="Reward")
    plt.xlabel(opt_params.param_mask_keys[0])
    plt.ylabel(opt_params.param_mask_keys[1])
    plt.title("All NES Populations Across All Iterations")
    plt.grid(True)

    plt.xlim(opt_params.param_min[0] * 0.9, opt_params.param_max[0] * 1.1)
    plt.ylim(opt_params.param_min[1] * 0.9, opt_params.param_max[1] * 1.1)

    if save_path:
        plt.savefig(save_path, bbox_inches="tight")
    else:
        plt.show()

    plt.close()


if __name__ == "__main__":
    optimizationFolder = pick_folder()
    run_hist = load_run_hist(optimizationFolder)
    #plot_population_2d_from_hist(run_hist, 4)
    #test_version(run_hist, 4)
    plot_full_population_2d(run_hist, optimizationFolder+ "/full_display")
    #run_NES_parallel(config)
