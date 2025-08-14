from dataclasses import dataclass
from utils.sample import compute_reward, sample_parameter
from utils.optim import *
from stickbots.config import StickbotParams, FeetVars
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor, as_completed
import os
from copy import deepcopy

np.random.seed(69420)  # For reproducibility
total_sims = 0

@dataclass
class NES_Config:
    pop_size: int = 5
    iterations: int = 5
    sigma: float = 0.2
    alpha: float = 0.05
    max_workers: int | None = None
    duration_override: float = 20


def run_NES_parallel(cfg: NES_Config):
    sim_config = SimConfig()
    urdf_params = StickbotParams()
    act_params = ActuationParams()
    feet_params = FeetVars()
    param_range = ParamRange()

    opt_params = OptimParams(
        urdf_params=urdf_params,
        act_params=act_params,
        feet_params=feet_params,
        param_range=param_range,
        sim_config=sim_config
    )

    num_dims = len(opt_params.attr2vec())
    print(f"{num_dims} optimization parameters")

    curr_params = opt_params.attr2vec()
    params_hist: np.ndarray = np.zeros((cfg.iterations, num_dims))
    params_hist[0] = curr_params[:]
    pop_hist: np.ndarray = np.zeros((cfg.iterations, cfg.pop_size, num_dims))
    pop_rew_hist: np.ndarray = np.zeros((cfg.iterations, cfg.pop_size))
    rew_hist: np.ndarray = np.zeros(cfg.iterations)

    if cfg.max_workers is None:
        cfg.max_workers = os.cpu_count() or 1
    print(f"{cfg.max_workers} max workers")

    for iter in range(cfg.iterations):
        print(f"Iteration {iter + 1}/{cfg.iterations} with parameters: {curr_params}")

        noise = np.random.randn(cfg.pop_size-1, num_dims)
        noise = np.insert(noise, 0, np.zeros((1, num_dims)), axis=0)
        candidate_params = curr_params + cfg.sigma * noise
        candidate_params = np.clip(candidate_params, opt_params.param_min, opt_params.param_max)
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
                print(f"sim #{sim_id} started with params: {params}")
                fut = executor.submit(sample_parameter, deepcopy(temp_param_obj), cfg.duration_override)
                future_to_info[fut] = (sim_id, idx)

            for fut in as_completed(future_to_info):
                sim_id, idx = future_to_info[fut]
                try:
                    result = fut.result()
                except Exception as exc:  # re-raise after logging which sim ended
                    print(f"sim #{sim_id} ended with error: {exc}")
                    raise
                rewards[idx] = float(result)
                print(f"sim #{sim_id} ended with reward {rewards[idx]}")

        rewards_array = np.array(rewards, dtype=float)
        mean_reward = float(np.mean(rewards_array))
        max_reward = float(np.max(rewards_array))
        pop_rew_hist[iter] = rewards_array
        rew_hist[iter] = mean_reward
        print("Mean reward:", mean_reward, "Max reward:", max_reward)

        advantages = (rewards_array - np.mean(rewards_array)) / np.std(rewards_array)
        curr_params = (
            curr_params
            + (cfg.alpha / (cfg.pop_size * cfg.sigma)) * np.dot(noise.T, advantages)
        )
        print(f"Updated parameters: {curr_params}")

        params_hist[iter + 1] = curr_params[:]
        history_array = np.array(params_hist)

        # Import for plotting only in the main process to reduce worker overhead
        import matplotlib.pyplot as plt  # noqa: WPS433 (local import by design)
        from matplotlib.ticker import MaxNLocator  # noqa: WPS433

        num_plots = len(opt_params.param_mask_keys) + 1  # add third subplot for average return
        fig, axes = plt.subplots(num_plots, 1, figsize=(10, 8), sharex=True)

        # Ensure axes is iterable even when num_plots == 1
        if num_plots == 1:
            axes = [axes]

        # X values aligned to iterations (exclude initial params row for alignment)
        num_iters_done = len(rew_hist)
        x_iter = list(range(1, num_iters_done + 1))

        # Parameter traces (aligned to iteration indices)
        for i, key in enumerate(opt_params.param_mask_keys):
            ax = axes[i]
            if num_iters_done > 0:
                ax.plot(x_iter, history_array[1:, i], label=key)
            ax.set_ylabel(f"{key}")
            ax.legend()
            ax.grid(True)

        # Average return subplot
        ax_ret = axes[-1]
        if num_iters_done > 0:
            ax_ret.plot(x_iter, rew_hist, label="avg_return")
        ax_ret.set_xlabel("Iteration")
        ax_ret.set_ylabel("Average return")
        ax_ret.legend()
        ax_ret.grid(True)

        # Match x-axis ticks/limits across subplots and use integer ticks
        ax_ret.set_xlim(1, max(1, num_iters_done))
        ax_ret.xaxis.set_major_locator(MaxNLocator(integer=True))

        fig.tight_layout()
        fig.savefig("parameter_evolution_parallel.png")

if __name__ == "__main__":
    config = NES_Config()
    run_NES_parallel(config)