from dataclasses import dataclass
from typing import ClassVar
import numpy as np
from utils.sample import compute_reward, sample_parameter
from utils.optim import *
from stickbots.config import StickbotParams, FeetVars
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor, as_completed
import os
from copy import deepcopy
import matplotlib.pyplot as plt  # noqa: WPS433 (local import by design)
from matplotlib.ticker import MaxNLocator  # noqa: WPS433
import pickle as pkl
import datetime
import tyro
import csv
from pathlib import Path
from itertools import product
from tqdm import tqdm


from cckTest import test

np.random.seed(69420)  # For reproducibility
total_sims = 0

@dataclass
class NES_Config:
    pop_size: int = 25
    iterations: int = 10
    sigma: float = 0.1  #originally .2, swapping to .1 for feet testing
    alpha: float = 0.05 
    max_workers: int | None = None
    duration_override: float = 1
    greedFunction: int = 0

    #Class level constants 
    NO_GREED: ClassVar[int] = 0
    GREEDY: ClassVar[int] = 1
    EVERY_OTHER_GREED: ClassVar[int] = 2
    

   

def normalize_params(params, pmin, pmax):
    " Normalize to [-1, 1] range "
    return 2.0 * (params - pmin) / (pmax - pmin) - 1.0

def denormalize_params(z, pmin, pmax):
    " Convert normalized [-1, 1] back to real domain. "
    return pmin + 0.5 * (z + 1.0) * (pmax - pmin)
    
    
    
def run_NES_FootSweep(cfg: NES_Config,log_path,X,Y,box_x,box_y):
    sim_config = SimConfig()
    urdf_params = StickbotParams()
    act_params = ActuationParams()
    #feet_params = FeetVars()
    feet_params = FeetVars(X=X,Y=Y,box_x=box_x,box_y=box_y)
    
    #print(f"[NES Start] X={feet_params.X:.4f}, Y={feet_params.Y:.4f}, box_x={feet_params.box_x:.4f}, box_y={feet_params.box_y:.4f}")
    param_range = ParamRange() #Change this class to change what parameters are optimized 

    opt_params = OptimParams(
        urdf_params=urdf_params,
        act_params=act_params,
        feet_params=feet_params,
        param_range=param_range,
        sim_config=sim_config
    )
    

    num_dims = len(opt_params.attr2vec())
    #print(f"{num_dims} optimization parameters")
    
    

    curr_params = opt_params.attr2vec()
    pmin = opt_params.param_min
    pmax = opt_params.param_max
    
    # Normalize parameters ( Convert real parameters → normalized [-1,1] )
    z_curr = normalize_params(curr_params, pmin, pmax)

    params_hist: np.ndarray = np.zeros((cfg.iterations, num_dims))
    pop_hist: np.ndarray = np.zeros((cfg.iterations, cfg.pop_size, num_dims))
    pop_rew_hist: np.ndarray = np.zeros((cfg.iterations, cfg.pop_size))
    rew_hist: np.ndarray = np.zeros(cfg.iterations)

    if cfg.max_workers is None:
        cfg.max_workers = os.cpu_count() or 1
    #print(f"{cfg.max_workers} max workers")

    for iter in range(cfg.iterations):
        #Convert center back to real parameters from normalized
        curr_params = denormalize_params(z_curr, pmin, pmax)
        #print(f"Iteration {iter + 1}/{cfg.iterations} with parameters: {curr_params}")

        #Generate gaussian, noise in popSize*optimized variables
        noise = np.random.randn(cfg.pop_size-1, num_dims)
        
        #Add a point without noise (aka zero noise per variable)
        noise = np.insert(noise, 0, np.zeros((1, num_dims)), axis=0)
        #candidate_params = curr_params + cfg.sigma * noise
                    #candidate_params = curr_params*(1+ cfg.sigma * noise)  #Multiplicative
        z_candidates = z_curr + cfg.sigma * noise    # additive noise in normalized space
        z_candidates = np.clip(z_candidates, -1.0, 1.0) #Hopefully safer clipping
        
        #Convert back to real parameters to run simulation
        candidate_params = denormalize_params(z_candidates, pmin, pmax)
        
        #candidate_params = np.clip(candidate_params, opt_params.param_min, opt_params.param_max)  #Adjust this, responsible for clipping
   
        pop_hist[iter] = candidate_params

        # Evaluate the population in parallel, tracking start/end of each sim
        with ProcessPoolExecutor(max_workers=cfg.max_workers, 
                                 mp_context=mp.get_context("spawn")) as executor:
            global total_sims
            rewards: np.ndarray = np.zeros(cfg.pop_size, dtype=float)
            future_to_info = {}
            for idx, params in enumerate(candidate_params):
                sim_id = total_sims + 1
                total_sims += 1

                temp_param_obj = deepcopy(opt_params)
                temp_param_obj.vec2attr(params)
                assert (temp_param_obj.attr2vec() == params).all()

                candidate_params[idx] = params.astype(float)
                #print(f"sim #{sim_id} started with params: {params}")
                fut = executor.submit(sample_parameter, deepcopy(temp_param_obj), cfg.duration_override)
                future_to_info[fut] = (sim_id, idx)

            for fut in as_completed(future_to_info):
                sim_id, idx = future_to_info[fut]
                try:
                    result = fut.result() #having errors at this step
                except Exception as exc:  # re-raise after logging which sim ended
                    print(f"sim #{sim_id} ended with error: {exc}")
                    raise
                rewards[idx] = float(result)
                #print(f"sim #{sim_id} ended with reward {rewards[idx]}")

        rewards_array = np.array(rewards, dtype=float)
        mean_reward = float(np.mean(rewards_array))
        max_reward = float(np.max(rewards_array))
        pop_rew_hist[iter] = rewards_array
        rew_hist[iter] = mean_reward
        #print("Mean reward:", mean_reward, "Max reward:", max_reward)

        #Edge case protection if all rewards are zero
        adv_std = np.std(rewards_array)
        if adv_std == 0:
            advantages = np.zeros_like(rewards_array)
        else:
            advantages = (rewards_array - np.mean(rewards_array)) / adv_std
        
        #advantages = (rewards_array - np.mean(rewards_array)) / np.std(rewards_array)

        match cfg.greedFunction:
            case NES_Config.NO_GREED:
                z_curr = (
                        z_curr + (cfg.alpha / (cfg.pop_size * cfg.sigma)) * np.dot(noise.T, advantages)
                )
                # clip to normalized bounds
                z_curr = np.clip(z_curr, -1.0, 1.0)
                
            case NES_Config.GREEDY:
                best_idx = np.argmax(rewards_array)
                z_curr = z_candidates[best_idx]
                
            case _:
                print("Error - incorrect greedFunction Value")  
   
        #Denormalize values after updating them     
        curr_params = denormalize_params(z_curr, pmin, pmax)
        #print(f"Updated parameters: {curr_params}")
            
       
        #Report best values
        if iter ==cfg.iterations-1:
            best_idx = np.argmax(rewards_array)
            best_params = pop_hist[iter, best_idx]
            #print(f"Best Final Value found: {best_params}")
        
        
       
        

        params_hist[iter] = curr_params[:]
        #plot_NES(iter, opt_params, rew_hist, params_hist, log_path)
        #plot_NES_2d(iter, opt_params, pop_hist, pop_rew_hist, log_path)
    
    x = opt_params.feet_params.X
    y = opt_params.feet_params.Y
    boxX = opt_params.feet_params.box_x
    boxY = opt_params.feet_params.box_y
    
    pickledFileName = (
    f"X={x:.4f}_Y={y:.4f}_boxX={boxX:.4f}_boxY={boxY:.4f}_run_hist.pkl"
    )
    
    full_path = Path(log_path) / "allRuns" / pickledFileName
    with open(full_path, "wb") as f:
        run_hist = {
            'pop_hist': pop_hist,
            'pop_rew_hist': pop_rew_hist,
            'opt_params': opt_params
        }
        pkl.dump(run_hist, f)
        
    return run_hist,pickledFileName


def plot_NES_2d(
        iter: int, 
        opt_params: OptimParams,
        pop_hist: np.ndarray, 
        pop_rew_hist: np.ndarray,
        log_path: str
) -> None:
    # plot the two variables at current iteration
    # first element is x because it's the nominal params, all other are circles
    # all dots are color-coded
    # x y lims are from opt_params
    plt.figure()
    plt.scatter(pop_hist[iter, :, 0], pop_hist[iter, :, 1], c=pop_rew_hist[iter], cmap='viridis')
    plt.scatter(pop_hist[iter, 0, 0], pop_hist[iter, 0, 1], color='red', marker='X', s=10)
    plt.colorbar(label='Reward')
    plt.xlabel(opt_params.param_mask_keys[0])
    plt.ylabel(opt_params.param_mask_keys[1])
    plt.xlim(opt_params.param_min[0] * 0.9, opt_params.param_max[0] * 1.1)
    plt.ylim(opt_params.param_min[1] * 0.9, opt_params.param_max[1] * 1.1)
    plt.title(f'Population at Iteration {iter}')
    plt.grid(True)
    plt.savefig(f"{log_path}/live_plots/population_2d_iter_{iter}.png")
    plt.close()


def plot_NES(
        iter: int,
        opt_params: OptimParams, 
        rew_hist: np.ndarray, 
        params_hist: np.ndarray,
        log_path: str
) -> None:

    num_plots = len(opt_params.param_mask_keys) + 1  # add third subplot for average return
    fig, axes = plt.subplots(num_plots, 1, figsize=(10, 8), sharex=True)

    # Ensure axes is iterable even when num_plots == 1
    if num_plots == 1:
        axes = [axes]

    # X values aligned to iterations (exclude initial params row for alignment)
    num_iters_done = iter + 1
    x_iter = list(range(num_iters_done))

    # Parameter traces (aligned to iteration indices)
    for i, key in enumerate(opt_params.param_mask_keys):
        ax = axes[i]
        ax.plot(x_iter, params_hist[x_iter, i], label=key)
        ax.set_ylabel(f"{key}")
        ax.legend()
        ax.grid(True)

    # Average return subplot
    ax_ret = axes[-1]
    ax_ret.plot(x_iter, rew_hist[x_iter], label="avg_return")
    ax_ret.set_xlabel("Iteration")
    ax_ret.set_ylabel("Average return")
    ax_ret.legend()
    ax_ret.grid(True)

    # Match x-axis ticks/limits across subplots and use integer ticks
    ax_ret.set_xlim(min(x_iter), max(x_iter))
    ax_ret.xaxis.set_major_locator(MaxNLocator(integer=True))

    fig.tight_layout()
    fig.savefig(f"{log_path}/live_plots/parameter_evolution_parallel.png")




def append_to_index(run_hist: dict, file_name: str, file_location: Path):
    opt_params = run_hist["opt_params"]
    fp = opt_params.feet_params

   
    row = {
        "file": file_name,                 # points to the pickle

        # geometry parameters
        "X": fp.X,
        "Y": fp.Y,
        
        "box_x": fp.box_x,
        "box_y": fp.box_y,
    }

    file_exists = file_location.exists()

    with open(file_location, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=row.keys())

        if not file_exists:
            writer.writeheader()   # first time only

        writer.writerow(row)       # always append one row



def footSweepWrapper(cfg: NES_Config,points):
    now = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    log_dir = Path("logs") / f"footSweep_{now}"
    log_dir.mkdir(parents=True, exist_ok=True)

    (log_dir / "allRuns").mkdir(parents=True, exist_ok=True)

    index_path = log_dir / "index.csv"

    fv = FeetVars()
    
    
    
    X_values = np.linspace(0.9 * fv.X, 1.1 * fv.X, points)
    Y_values = np.linspace(0.9 * fv.Y, 1.1 * fv.Y, points)
    boxX_values = np.linspace(0.9 * fv.box_x, 1.1 * fv.box_x, points)
    boxY_values = np.linspace(0.9 * fv.box_y, 1.1 * fv.box_y, points)
    
    total_points = points**4
    
    index_grid = product(range(points), range(points), range(points), range(points))

    for i, j, k, l in tqdm(index_grid, total=total_points, desc="[LEGO NES] 4D sweep"):
        X    = X_values[i]
        Y    = Y_values[j]
        boxX = boxX_values[k]
        boxY = boxY_values[l]

        
        # print(f"[LEGO NES] Point (i={i+1}, j={j+1}, k={k+1}, l={l+1}) / {points}")

        run_hist, pickleFileName = run_NES_FootSweep(cfg, log_dir, X, Y, boxX, boxY)
        append_to_index(run_hist, pickleFileName, index_path)
                
    

    



if __name__ == "__main__":
    config = tyro.cli(NES_Config)
    #file_location = RESULTS_DIR / "index.csv"
    
    points = 10 #Number of points in 4D grid space betwen 90% and 110% of default value
    
    footSweepWrapper(config,points)
    
    #run_NES_FootSweep(config,.23,.25,.1,.06)
    #run_NES_parallel(config)