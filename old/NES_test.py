import os
import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor, as_completed
import argparse
import numpy as np
from typing import Optional, List
from dataclasses import dataclass, field, make_dataclass

from utils.sim_funcs import run_sim

total_sims = 0


"""Standalone config and helpers copied from NES.py so this file is self-contained."""
sim_config = {
    "urdf_filename": "stickbot/stick_bot_generated.urdf",
    "duration": 15.0,
    "sim_time_step": 0.001,
    "calib_time_step": 0.01,
    "save_data": False,
    "ground_friction": 0.9,
    "feet_friction": 0.9,
    "control_period": 0.001,
    "wait_time": 0,
    "start_height": 0.16,
    "hip_kp": 1.5,
    "hip_kd": 3e-2,
}

opt_params = {
    "frequency": (1.5, 1.2, 2.2),
    # amplitude (deg) converted to radians
    "amplitude": tuple(a * np.pi / 180 for a in (35, 20, 45)),
}


@dataclass
class NES_Config:
    pop_size: int = 25
    sigma: float = 0.2
    alpha: float = 0.05


def sanitize_key(key: str) -> str:
    sanitized = key.replace("-", "_").replace(" ", "_")
    if sanitized and sanitized[0].isdigit():
        sanitized = "_" + sanitized
    return sanitized


def dict_to_empty_dataclass(data_dict: dict):
    dc_fields = []
    for key in data_dict.keys():
        sanitized_key = sanitize_key(key)
        dc_fields.append((sanitized_key, type(None), field(default=data_dict[key])))
    DynamicDataclass = make_dataclass("DynamicDataclass", dc_fields)
    return DynamicDataclass()


def clip_params(param_vector: np.ndarray) -> np.ndarray:
    for i, key in enumerate(opt_params.keys()):
        param_vector[i] = np.clip(param_vector[i], opt_params[key][1], opt_params[key][2])
    return param_vector


def compute_reward(
    com_xyz: np.ndarray,
    sim_time_step: float,
    duration: float,
    z_min: float = 0.0,
    z_max: float = 0.25,
    displacement_weight: float = 10.0,
) -> float:
    """Compute the NES reward from the COM trajectory.

    - Alive time: counts steps where z is within bounds [z_min, z_max]
    - Terminal penalty if last step is out of bounds
    - Displacement bonus scaled by ``displacement_weight``
    """
    com_displacement = np.linalg.norm(com_xyz[-1] - com_xyz[0], axis=0)
    com_zs = com_xyz[:, 2]
    alive = (com_zs > z_min) & (com_zs < z_max)

    final_reward = float(np.sum(alive) * sim_time_step)
    if not bool(alive[-1]):
        final_reward -= float(duration)
    else:
        final_reward += float(com_displacement * displacement_weight)
    return float(final_reward)


def _evaluate_one(param_vector: np.ndarray, duration_override: Optional[float] = None) -> float:
    """Evaluate a single parameter vector by running the Drake simulation.

    Mirrors the logic in `rew_fn` from `NES.py`, but without printing.
    """
    # Avoid mutating the shared `sim_config` by copying before modifications
    local_config = sim_config.copy()

    if duration_override is not None:
        local_config["duration"] = float(duration_override)

    for param_value, key in zip(param_vector, opt_params.keys()):
        local_config[key] = float(param_value)

    config_obj = dict_to_empty_dataclass(local_config)
    com_xyz = run_sim(config_obj)[7]
    return compute_reward(com_xyz, config_obj.sim_time_step, config_obj.duration)


# (Removed) _evaluate_one_with_override: unified into _evaluate_one with optional override


def run_NES_parallel(
    iters: int,
    max_workers: Optional[int] = None,
    pop_size: Optional[int] = None,
    duration_override: Optional[float] = None,
) -> None:
    """Run NES where each candidate evaluation is executed in parallel processes."""
    config = NES_Config()
    if pop_size is not None:
        config.pop_size = int(pop_size)

    num_dims = len(opt_params.keys())
    current_params = np.zeros(num_dims)
    for idx, key in enumerate(opt_params.keys()):
        current_params[idx] = opt_params[key][0]

    params_history: List[np.ndarray] = [current_params.copy()]
    reward_history: List[float] = []

    if max_workers is None:
        max_workers = os.cpu_count() or 1

    for iteration_index in range(iters):
        print(f"Iteration {iteration_index + 1}/{iters} with parameters: {current_params}")

        noise = np.random.randn(config.pop_size, num_dims)
        candidate_params: List[np.ndarray] = []
        for j in range(config.pop_size):
            trial = current_params + config.sigma * noise[j]
            trial = clip_params(trial)
            candidate_params.append(trial.astype(float))

        # Evaluate the population in parallel, tracking start/end of each sim
        with ProcessPoolExecutor(max_workers=max_workers, mp_context=mp.get_context("spawn")) as executor:
            global total_sims
            rewards: List[float] = [0.0] * len(candidate_params)
            future_to_info = {}
            for idx, params in enumerate(candidate_params):
                sim_id = total_sims + 1
                total_sims += 1
                print(f"sim #{sim_id} started")
                fut = executor.submit(_evaluate_one, params, duration_override)
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
        reward_history.append(mean_reward)
        print("Mean reward:", mean_reward, "Max reward:", max_reward)

        advantages = (rewards_array - np.mean(rewards_array)) / np.std(rewards_array)
        current_params = (
            current_params
            + (config.alpha / (config.pop_size * config.sigma)) * np.dot(noise.T, advantages)
        )
        current_params = clip_params(current_params)
        print(f"Updated parameters: {current_params}")

        params_history.append(current_params.copy())
        history_array = np.array(params_history)

        # Import for plotting only in the main process to reduce worker overhead
        import matplotlib.pyplot as plt  # noqa: WPS433 (local import by design)
        from matplotlib.ticker import MaxNLocator  # noqa: WPS433

        num_plots = len(opt_params.keys()) + 1  # add third subplot for average return
        fig, axes = plt.subplots(num_plots, 1, figsize=(10, 8), sharex=True)

        # Ensure axes is iterable even when num_plots == 1
        if num_plots == 1:
            axes = [axes]

        # X values aligned to iterations (exclude initial params row for alignment)
        num_iters_done = len(reward_history)
        x_iter = list(range(1, num_iters_done + 1))

        # Parameter traces (aligned to iteration indices)
        for i, key in enumerate(opt_params.keys()):
            ax = axes[i]
            if num_iters_done > 0:
                ax.plot(x_iter, history_array[1:, i], label=sanitize_key(key))
            ax.set_ylabel(f"{sanitize_key(key)}")
            ax.legend()
            ax.grid(True)

        # Average return subplot
        ax_ret = axes[-1]
        if num_iters_done > 0:
            ax_ret.plot(x_iter, reward_history, label="avg_return")
        ax_ret.set_xlabel("Iteration")
        ax_ret.set_ylabel("Average return")
        ax_ret.legend()
        ax_ret.grid(True)

        # Match x-axis ticks/limits across subplots and use integer ticks
        ax_ret.set_xlim(1, max(1, num_iters_done))
        ax_ret.xaxis.set_major_locator(MaxNLocator(integer=True))

        fig.tight_layout()
        fig.savefig("parameter_evolution_parallel.png")

    print("Final parameters:", current_params)
    print("Final reward:", _evaluate_one(current_params))


def main() -> None:
    parser = argparse.ArgumentParser(description="Run NES with parallel Drake simulations.")
    parser.add_argument("-i", "--iters", type=int, default=30, help="Number of NES iterations")
    parser.add_argument("-w", "--workers", type=int, default=None, help="Max parallel workers (defaults to CPU count)")
    parser.add_argument("-p", "--pop-size", type=int, default=None, help="Population size (overrides NES_Config.pop_size)")
    parser.add_argument("-d", "--duration", type=float, default=None, help="Override simulation duration (seconds) for faster tests")
    args = parser.parse_args()
    run_NES_parallel(
        iters=args.iters,
        max_workers=args.workers,
        pop_size=args.pop_size,
        duration_override=args.duration,
    )


if __name__ == "__main__":
    main()


