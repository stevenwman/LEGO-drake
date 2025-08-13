from dataclasses import dataclass, asdict
from stickbots.config import StickbotParams, FeetVars
import numpy as np
from copy import deepcopy


@dataclass
class NES_Config:
    pop_size: int = 25
    sigma: float = 0.2
    alpha: float = 0.05


@dataclass
class SimConfig:
    """
    Static simulation params
    """
    duration: float = 15.0
    sim_time_step: float = 0.001
    calib_time_step: float = 0.01
    save_data: bool = False
    ground_friction: float = 0.9
    feet_friction: float = 0.9
    control_period: float = 0.001
    wait_time: float = 0
    start_height: float = 0.16
    

@dataclass
class ActuationParams:
    hip_kp: float = 1.5
    hip_kd: float = 3e-2
    amplitude: float = 35 * np.pi / 180  # Convert degrees to radians
    frequency: float = 1.5  # Example frequency in Hz 


@dataclass
class ParamRange:
    X: float = (0.24, 0.24 * 0.9, 0.24 * 1.1)
    Y: float = (0.24, 0.24 * 0.9, 0.24 * 1.1)
    # Z: float = (0.24, 0.24 * 0.9, 0.24 * 1.1)
    # box_x: float = (0.101, 0.101 * 0.9, 0.101 * 1.1)
    # box_y: float = (0.0527, 0.0527 * 0.9, 0.0527 * 1.1)
    # fn: int = 100 # mesh resolution

    # gap_ft: float = (0.032, 0.032 * 0.9, 0.032 * 1.1)
    # w_arm: float = (0.0625, 0.0625 * 0.9, 0.0625 * 1.1)
    # l_arm: float = (0.104, 0.104 * 0.9, 0.104 * 1.1)
    # l_leg: float = (0.153, 0.153 * 0.9, 0.153 * 1.1)
    # hip_offset: float = (-0.01, -0.01 * 0.9, -0.01 * 1.1)
    # leg_mass: float = (0.1, 0.1 * 0.9, 0.1 * 1.1)
    # feet_mass: float = (0.13, 0.13 * 0.9, 0.13 * 1.1)
    # hand_mass: float = (0.15, 0.15 * 0.9, 0.15 * 1.1)

    # hip_kp: float = (1.5, 1.5 * 0.9, 1.5 * 1.1)
    # hip_kd: float = (3e-2, 3e-2 * 0.9, 3e-2 * 1.1)
    amplitude: float = tuple(a * np.pi / 180 for a in (35, 20, 45))
    frequency: float = (1.5, 1.2, 2.2)


class OptimParams:
    """
    Handles parameter formatting and update for simulation
    """
    def __init__(
            self, 
            urdf_params: StickbotParams, 
            act_params: ActuationParams,
            feet_params: FeetVars,
            param_range: ParamRange, 
        ) -> None:

        self.starter = {
            "urdf_params"   : deepcopy(urdf_params),
            "act_params"    : deepcopy(act_params),
            "feet_params"   : deepcopy(feet_params),
            "param_range"   : deepcopy(param_range)
        }

        self.urdf_params    = urdf_params
        self.act_params     = act_params
        self.feet_params    = feet_params

        param_range_dict = asdict(param_range)
        self.merge_params()
        param_mask = np.zeros(len(self.merged_dict.keys()))
        self.param_mask_keys = []
        self.param_min, self.param_max = np.array([]), np.array([])

        for i, k in enumerate(self.merged_dict.keys()):
            if k in param_range_dict:
                param_mask[i] = 1
                self.param_min = np.append(self.param_min, param_range_dict[k][1])
                self.param_max = np.append(self.param_max, param_range_dict[k][2])
                self.param_mask_keys.append(k)
        self.masked_idx = np.where(param_mask == 1)[0]

    def merge_params(self) -> None:
        self.merged_dict = asdict(self.urdf_params) | \
                            asdict(self.act_params) | \
                            asdict(self.feet_params)

    def attr2vec(self) -> np.array:
        self.merge_params()
        return np.array(list(self.merged_dict.values()))[self.masked_idx]

    def vec2attr(self, vec: np.array) -> None:
        vec = np.clip(vec, self.param_min, self.param_max)
        for k, v in zip(self.param_mask_keys, vec):
            for param in [self.urdf_params, self.act_params, self.feet_params]:
                if hasattr(param, k): setattr(param, k, v)