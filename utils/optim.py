from dataclasses import dataclass
from stickbots.config import StickbotParams, FeetVars
import numpy as np
from copy import deepcopy
from typing import Optional


@dataclass
class SimConfig:
    """
    Static simulation params
    """
    urdf_filename: str = ""
    duration: float = 15.0
    sim_time_step: float = 0.001
    calib_time_step: float = 0.01
    save_data: bool = False
    ground_friction: float = 0.9
    feet_friction: float = 0.9
    control_period: float = 0.001
    wait_time: float = 0
    # TODO automate start_height
    start_height: float = 0.16
    

@dataclass
class ActuationParams:
    hip_kp: float = 1.5
    hip_kd: float = 3e-2
    amplitude: float =0.51777882 #0.49133407#28deg    35 * np.pi / 180  # Convert degrees to radians .20708687# 
    frequency: float = 1.72771918 #1.7496934#    1.5  # Example frequency in Hz 1.89007795# 
                    #Current best vals, 53 Reward


@dataclass
class DrakeParams(SimConfig, ActuationParams):
    pass

@dataclass
class ParamRange:
    #X: float = (0.24, 0.24 * 0.9, 0.24 * 1.1) #X rad Sphere dimensions to cut
    #Y: float = (0.24, 0.24 * 0.9, 0.24 * 1.1) # X Rad
    # Z: float = (0.24, 0.24 * 0.9, 0.24 * 1.1) #Leave along for now
    #box_x: float = (0.101, 0.101 * 0.9, 0.101 * 1.1)  #Sets X space to cut sphere
    #box_y: float = (0.0527, 0.0527 * 0.9, 0.0527 * 1.1) #Sets Y space to cut spehere
    # fn: int = 100 # mesh resolution


    #Hold to same
    # gap_ft: float = (0.032, 0.032 * 0.9, 0.032 * 1.1) #Distance btwn sphere center
    # w_arm: float = (0.0625, 0.0625 * 0.9, 0.0625 * 1.1) #Shoulder to shoulder
    # l_arm: float = (0.104, 0.104 * 0.9, 0.104 * 1.1) #length arm
    # l_leg: float = (0.153, 0.153 * 0.9, 0.153 * 1.1) #Hip to top of foot
    # hip_offset: float = (-0.01, -0.01 * 0.9, -0.01 * 1.1)
    # leg_mass: float = (0.1, 0.1 * 0.9, 0.1 * 1.1)
    # feet_mass: float = (0.13, 0.13 * 0.9, 0.13 * 1.1)
    # hand_mass: float = (0.15, 0.15 * 0.9, 0.15 * 1.1)

    # hip_kp: float = (1.5, 1.5 * 0.9, 1.5 * 1.1)
    # hip_kd: float = (3e-2, 3e-2 * 0.9, 3e-2 * 1.1)
    
    
    #Actuation
    amplitude: float = tuple(a * np.pi / 180 for a in (28, 10, 45)) #Min was 20, decreasing to 10
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
            sim_config: Optional[SimConfig] = SimConfig()
        ) -> None:

        self.starter = {
            "urdf_params"   : deepcopy(urdf_params),
            "act_params"    : deepcopy(act_params),
            "feet_params"   : deepcopy(feet_params),
            "param_range"   : deepcopy(param_range)
        }

        self.urdf_params = urdf_params
        self.act_params = act_params
        self.feet_params = feet_params
        self.sim_config = sim_config
        self.update_drake_parms()

        param_range_dict = param_range.__dict__
        self.merge_params()
        param_mask = np.zeros(len(self.merged_dict.keys()))
        self.param_mask_keys = []
        self.param_min, self.param_max, self.param_range = \
            np.array([]), np.array([]), np.array([])

        for i, k in enumerate(self.merged_dict.keys()):
            if k in param_range_dict:
                param_mask[i] = 1
                self.param_min = np.append(self.param_min, param_range_dict[k][1])
                self.param_max = np.append(self.param_max, param_range_dict[k][2])
                self.param_mask_keys.append(k)
                self.param_range = np.append(self.param_range, 
                                             param_range_dict[k][2] - param_range_dict[k][1])
        self.masked_idx = np.where(param_mask == 1)[0]

    def update_drake_parms(self) -> None:
        self.combined_params = DrakeParams(
            **self.sim_config.__dict__,
            **self.act_params.__dict__,
        )

    def merge_params(self) -> None:
        self.merged_dict = self.urdf_params.__dict__ | \
                            self.act_params.__dict__ | \
                            self.feet_params.__dict__

    def attr2vec(self) -> np.array:
        self.merge_params()
        return np.array(list(self.merged_dict.values()))[self.masked_idx]

    def vec2attr(self, vec: np.array) -> None:
        vec = np.clip(vec, self.param_min, self.param_max)
        for k, v in zip(self.param_mask_keys, vec):
            for param in [self.urdf_params, self.act_params, self.feet_params]:
                if hasattr(param, k): setattr(param, k, v)
        self.update_drake_parms()