
import argparse
from dataclasses import dataclass
from old.meshcat_setup import start_meshcat
import datetime
import os
import numpy as np
from pydrake.all import *
from utils.sim_funcs import *

@dataclass
class SimConfig:
    """Container for tunable simulation parameters."""
    urdf_filename: str = "stickbot/stick_bot_generated.urdf"
    # Sim params
    duration: float = 30.0
    sim_time_step: float = 0.001    # Simulation time step in seconds
    calib_time_step: float = 0.01   # Calibration time step in seconds
    controller_period: float = 0.0005
    start_height: float = 0.16  # Initial height of the robot's COM
    save_data: bool = False
    visualize_coms: bool = False
    # Dynamics params
    ground_friction: float = 0.9 # set to 0.4 for scale <= 1 or 0.9 for scale > 1
    feet_friction: float = 0.9 # set to 0.7 for scale <= 1 or 0.9 for scale > 1
    # Controller params
    control_period: float = 0.0005
    hip_kp: float = 8000
    hip_ki: float = 0
    hip_kd: float = 0.1
    threshold_force: float = 0.0001  # in N, To set contact mode
    # Actuation parameters
    frequency = 1.8
    wait_time = 0
    counter = 0
    amplitude = 35 * np.pi / 180


def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Run Mugatu walker simulation.")
    parser.add_argument("--simulate_walker", action="store_true", help="Run the walker sim. (otherwise JointSliders)")
    parser.add_argument("--duration", type=float, default=30.0, help="Simulation duration (default 30).")

    args = parser.parse_args()
    config = SimConfig(
        duration=args.duration,
    )

    print(f"Using URDF file: {config.urdf_filename}")

    meshcat = start_meshcat()

    if not args.simulate_walker:
        print("Opening joint sliders …")
        print(meshcat.web_url())
        run_joint_sliders(config, meshcat)
        return
    
    now = datetime.datetime.now()
    label = now.strftime("%Y-%m-%d_%H-%M-%S")

    base_dir = os.path.join(os.getcwd(), "run_sim_save_data")
    alldata_folder = os.path.join(base_dir, f"data_for_{label}")
    plots_folder = os.path.join(alldata_folder, f"plots_{label}")
    os.makedirs(plots_folder, exist_ok=True)

    csv_base = os.path.join(alldata_folder, f"all_data_{label}")

    run_sim(config, meshcat)


if __name__ == "__main__":
    main()
