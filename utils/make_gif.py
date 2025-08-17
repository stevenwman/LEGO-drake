from PIL import Image
import matplotlib.pyplot as plt
import pickle as pkl
import numpy as np
from utils.optim import OptimParams
import glob
import os 

def plot_NES_2d_series(
    log_path: str
) -> None:
    # plot the two variables at current iteration
    # first element is x because it's the nominal params, all other are circles
    # all dots are color-coded
    # x y lims are from opt_params

    with open(f"{log_path}/run_hist.pkl", 'rb') as f:
        run_hist = pkl.load(f)

    os.makedirs(f"{log_path}/gif", exist_ok=True)

    pop_hist: np.ndarray = run_hist['pop_hist']
    pop_rew_hist: np.ndarray = run_hist['pop_rew_hist']
    opt_params: OptimParams = run_hist['opt_params']

    # Determine the global min and max rewards across all iterations for consistent color mapping
    min_rew = np.min(pop_rew_hist)
    max_rew = np.max(pop_rew_hist)

    for iteration in range(pop_hist.shape[0]):
        plt.figure()
        # Use vmin and vmax to ensure the color scale is consistent across all plots
        sc = plt.scatter(pop_hist[iteration, :, 0], pop_hist[iteration, :, 1], c=pop_rew_hist[iteration], cmap='viridis', vmin=min_rew, vmax=max_rew)
        plt.scatter(pop_hist[iteration, 0, 0], pop_hist[iteration, 0, 1], color='red', marker='o', s=10, label='Nominal') # Increased size and added label for clarity
        plt.colorbar(sc, label='Reward')
        plt.xlabel(opt_params.param_mask_keys[0])
        plt.ylabel(opt_params.param_mask_keys[1])
        plt.xlim(opt_params.param_min[0] * 0.9, opt_params.param_max[0] * 1.1)
        plt.ylim(opt_params.param_min[1] * 0.9, opt_params.param_max[1] * 1.1)
        plt.title(f'Population at Iteration {iteration}')
        plt.grid(True)
        plt.savefig(f"{log_path}/gif/{iteration}.png", )
        plt.close()


def create_gif_from_folder_glob(folder_path, output_gif_path, duration=500, loop=0):
    """
    Creates a GIF from all PNG images in a folder, sorted alphabetically, using glob.

    Args:
    folder_path: The path to the folder containing the PNG images.
    output_gif_path: The desired path and filename for the output GIF.
    duration: The duration each frame is displayed in milliseconds (default: 500).
    loop: The number of times the GIF should loop (0 for infinite loop) (default: 0).
    """

    # Find all PNG files in the folder and sort them alphabetically
    # The ** operator allows recursive searching in subdirectories, 
    # but here we're only looking in the immediate folder.
    # We use os.path.join to construct full paths to the files for better cross-platform compatibility.
    image_paths = sorted(glob.glob(os.path.join(folder_path, "*.png")))

    if not image_paths:
        print(f"No PNG files found in the specified folder: {folder_path}")
        return

    # Open all images and store them in a list
    images = [Image.open(image_path) for image_path in image_paths]

    # Save the first image, appending the rest as frames
    images[0].save(
        output_gif_path,
        save_all=True,
        append_images=images[1:],
        duration=duration,
        loop=loop
    )

    print(f"GIF created and saved at {output_gif_path}")