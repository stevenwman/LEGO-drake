from stickbot.generate_stickbot import generate_stickbot
from dataclasses import dataclass, fields, make_dataclass, field
import numpy as np
from utils.sim_funcs import run_sim
import matplotlib.pyplot as plt


sim_config ={
    'urdf_filename': "stickbot/stick_bot_generated.urdf",
    'duration': 15.0,
    'sim_time_step': 0.001,
    'calib_time_step': 0.01,
    'save_data': False,
    'ground_friction': 0.9, # set to 0.4 for scale <= 1 or 0.9 for scale > 1
    'feet_friction': 0.9, # set to 0.7 for scale <= 1 or 0.9 for scale > 1
    'control_period': 0.001,
    'wait_time': 0,
    'start_height': 0.16, # Initial height of the robot's COM
    'hip_kp': 1.5,
    'hip_kd': 3e-2,
}

morpho_params: dict = {
    # gap between feets
    'gap_ft': 0.032,
    # width between leg and arm
    'w_arm' : 0.0625,
    # length of arm and leg links
    'l_arm' : 0.104, 'l_leg' : 0.153,
    # hip offset
    # 'hip_offset' : -0.014,
    'hip_offset' : -0.01,
    # masses 
    'leg_mass' : 0.1, 'feet_mass' : 0.13, 'hand_mass' : 0.15,
    # feet geom params
    'feet_vars_dict' : {
        # Ellipsoid diameters
        "X": 0.24, "Y": 0.24, "Z": 0.24,
        # feet box dimensions
        "box_x": 0.101, "box_y": 0.0527,
        "scad_fn": 100,
    },
    # dynamic properties
    'dynamics' : {
        'ground_friction' : 0.9,
        'feet_friction' : 0.9,
        'hydroelastic_modulus' : 5e7,
        'ground_hydroelastic_modulus' : 1e8,
        'ground_mesh_resolution_hint' : 0.01,
        'mesh_resolution_hint' : 0.01,
    }     
}

opt_params = {
    'frequency': (1.5, 1.2, 2.2),
    # 'amp_deg': (35, 20, 45),  # amplitude in degrees
    'amplitude': tuple(a * np.pi / 180 for a in (35, 20, 45)),  # convert to radians
}


def sanitize_key(key):
    sanitized = key.replace('-', '_').replace(' ', '_')
    if sanitized[0].isdigit():
        sanitized = '_' + sanitized
    return sanitized

def dict_to_empty_dataclass(data_dict):
    fields = []
    for key in data_dict.keys():
        sanitized_key = sanitize_key(key)
        fields.append((sanitized_key, type(None), field(default=data_dict[key]))) # Or use a different default empty value

    DynamicDataclass = make_dataclass('DynamicDataclass', fields)

    return DynamicDataclass()

@dataclass
class NES_Config:
    pop_size: int = 10 
    sigma: float = 0.2
    alpha: float = 0.1


def rew_fn(param_vector) -> float:
    """Cost function to evaluate the performance of a given parameter set."""
    config = sim_config

    for param_val, key in zip(param_vector, opt_params.keys()):
        config[key] = param_val

    config = dict_to_empty_dataclass(config)
    print(f"Running simulation with parameters: {param_vector}", end="")
    _,_,_,_,_,_,_, com_xyz, _, _ = run_sim(config)
    com_displacement = np.linalg.norm(com_xyz[-1] - com_xyz[0], axis=0)
    print(f", COM displacement: {com_displacement:.4f}", end="")

    com_zs = com_xyz[:, 2]
    alive = (com_zs > 0) &  (com_zs < 0.25)

    final_reward = np.sum(alive) * config.sim_time_step  # Reward for being alive

    if not alive[-1]:
        print(", COM out of bounds", end="")
        final_reward -= config.duration
    else:
        final_reward += com_displacement * 10

    print(f" -> Final reward: {final_reward:.4f}")

    return final_reward

def clip_params(param_vector):
    """Ensure parameters stay within defined bounds."""
    for i, key in enumerate(opt_params.keys()):
        param_vector[i] = np.clip(param_vector[i], opt_params[key][1], opt_params[key][2])
    return param_vector

def run_NES(iters=None):
    config = NES_Config()

    w = np.zeros(len(opt_params.keys()))
    for i, key in enumerate(opt_params.keys()):
        w[i] = opt_params[key][0]

        w_hist = [w.copy()]

    for _ in range(iters):
        print(f"Iteration {_ + 1}/{iters} with parameters: {w}")
        N = np.random.randn(config.pop_size, len(w))
        R = np.zeros(config.pop_size)
        for j in range(config.pop_size):
            w_try = w + config.sigma*N[j]
            w_try = clip_params(w_try)
            R[j] = rew_fn(w_try)
        print("Mean reward:", np.mean(R), "Max reward:", np.max(R))
        A = (R - np.mean(R)) / np.std(R)
        w = w + config.alpha/(config.pop_size*config.sigma) * np.dot(N.T, A)
        w = clip_params(w)
        print(f"Updated parameters: {w}")

        w_hist.append(w.copy())
        w_hist_array = np.array(w_hist)
        plt.figure(figsize=(10, 6))
        plots = len(opt_params.keys())        
        for i, key in enumerate(opt_params.keys()):
            plt.subplot(plots, 1, i + 1)
            plt.plot(w_hist_array[:, i], label=sanitize_key(key))
            plt.xlabel('Iteration')
            plt.ylabel(f'{sanitize_key(key)}')
            plt.legend()
            plt.grid(True)
        # save figure 
        plt.tight_layout()
        plt.savefig('parameter_evolution.png')

    print("Final parameters:", w)
    print("Final reward:", rew_fn(w))


if __name__ == "__main__":
    run_NES(15)