import numpy as np
from utils.sim_funcs import run_sim
from utils.optim import OptimParams, DrakeParams
from typing import Optional
from stickbots.stickbot import build_stickbot


def compute_reward(
    com_xyz: np.ndarray,
    sim_params: DrakeParams,
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

    final_reward = float(np.sum(alive) * sim_params.sim_time_step)
    if not bool(alive[-1]):
        final_reward -= float(sim_params.duration)
    else:
        final_reward += float(com_displacement * displacement_weight)
    return float(final_reward)


def sample_parameter(optim_params: OptimParams, duration_override: Optional[float] = None) -> float:
    """Evaluate a single parameter vector by running the Drake simulation.

    Mirrors the logic in `rew_fn` from `NES.py`, but without printing.
    """
    urdf_path = build_stickbot(optim_params.urdf_params, optim_params.feet_params).urdf_path
    optim_params.sim_config.urdf_filename = urdf_path
    if duration_override is None:
        optim_params.sim_config.duration = float(duration_override)
    optim_params.update_drake_parms()
    com_xyz = run_sim(optim_params.combined_params)[7]
    return compute_reward(com_xyz, optim_params.combined_params)