
import argparse
from dataclasses import dataclass
import datetime
import os
import numpy as np
from pydrake.all import *
from utils.sim_funcs import *
from utils.helpers import start_meshcat
import matplotlib.pyplot as plt
import os
import numpy as np # Make sure numpy is imported

@dataclass
class SimConfig:
    """Container for tunable simulation parameters."""
    urdf_filename: str = "stickbot/stick_bot_generated.urdf"
    # Sim params
    duration: float = 30.0
    sim_time_step: float = 0.001    # Simulation time step in seconds
    calib_time_step: float = 0.01   # Calibration time step in seconds
    start_height: float = 0.16  # Initial height of the robot's COM
    save_data: bool = False
    visualize_coms: bool = False
    # Dynamics params
    ground_friction: float = 0.9 # set to 0.4 for scale <= 1 or 0.9 for scale > 1
    feet_friction: float = 0.9 # set to 0.7 for scale <= 1 or 0.9 for scale > 1
    # Controller params
    control_period: float = 0.001
    hip_kp: float = 1.5
    hip_ki: float = 0
    hip_kd: float = 3e-2
    # Actuation parameters
    frequency = 1.8
    wait_time = 0
    counter = 0
    amplitude = 35 * np.pi / 180


def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Run Mugatu walker simulation.")
    parser.add_argument("--walk", action="store_true", help="Run the walker sim. (otherwise JointSliders)")
    parser.add_argument("--duration", type=float, default=10.0, help="Simulation duration (default 30).")
    parser.add_argument("--save", action="store_true", help="Save simulation data to CSV files.")

    args = parser.parse_args()
    config = SimConfig(
        duration=args.duration,
    )

    print(f"Using URDF file: {config.urdf_filename}")

    meshcat = start_meshcat()

    if not args.walk:
        print("Opening joint sliders …")
        print(meshcat.web_url())
        run_joint_sliders(config, meshcat)
        return
    
    now = datetime.datetime.now()
    label = now.strftime("%Y-%m-%d_%H-%M-%S")

    base_dir = os.path.join(os.getcwd(), "run_sim_save_data")
    alldata_folder = os.path.join(base_dir, f"data_for_{label}")
    plots_folder = os.path.join(alldata_folder, f"plots_{label}")
    if args.save:
        os.makedirs(plots_folder, exist_ok=True)

    csv_base = os.path.join(alldata_folder, f"all_data_{label}")

    # Run the simulation and unpack results
    (states, hip_real_torque, desired_hip_angle, left_contact_forces,
        left_contact_points, right_contact_forces, right_contact_points,
        com_xyz, com_vxyz, time_array) = run_sim(config, meshcat)
    
    # Split the composite COM vectors for your existing plotting code
    com_x, com_y, com_z = com_xyz[:, 0], com_xyz[:, 1], com_xyz[:, 2]
    com_vx, com_vy, com_vz = com_vxyz[:, 0], com_vxyz[:, 1], com_vxyz[:, 2]

    print(f"Simulation completed. Data saved to {base_dir}")

    # --- Plotting for sanity check ---
    print("Generating plots for sanity check...")

    # Calculate magnitudes for contact forces
    left_force_magnitude = np.linalg.norm(left_contact_forces, axis=1)
    right_force_magnitude = np.linalg.norm(right_contact_forces, axis=1)

    # Change the figure size and subplot layout to accommodate the new plot
    plt.figure(figsize=(12, 24)) # Increased height to make space for 5 subplots

    n_plots = 5
    ith_plot = 1

    # Plot 1: Contact Force Magnitudes
    plt.subplot(n_plots, 1, ith_plot) # Changed to 5 rows
    plt.plot(time_array, left_force_magnitude, label='Left Foot Contact Force Magnitude')
    plt.plot(time_array, right_force_magnitude, label='Right Foot Contact Force Magnitude')
    plt.xlabel('Time (s)')
    plt.ylabel('Force Magnitude (N)')
    plt.title('Contact Force Magnitudes Over Time')
    plt.grid(True)
    plt.legend()
    ith_plot += 1

    # Plot 2: Center of Mass Position
    plt.subplot(n_plots, 1, ith_plot) # Changed to 5 rows
    plt.plot(time_array, com_x, label='CoM X')
    plt.plot(time_array, com_y, label='CoM Y')
    plt.plot(time_array, com_z, label='CoM Z')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title('Center of Mass Position Over Time')
    plt.grid(True)
    plt.legend()
    ith_plot += 1

    # Plot 3: Center of Mass Velocity
    plt.subplot(n_plots, 1, ith_plot) # Changed to 5 rows
    plt.plot(time_array, com_vx, label='CoM Vx')
    plt.plot(time_array, com_vy, label='CoM Vy')
    plt.plot(time_array, com_vz, label='CoM Vz')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Center of Mass Velocity Over Time')
    plt.grid(True)
    plt.legend()
    ith_plot += 1


    # Plot 4: Contact Points in XY Plane
    plt.subplot(n_plots, 1, ith_plot) # New subplot for XY contact points
    plt.plot(left_contact_points[:, 0], left_contact_points[:, 1], 'o-', markersize=2, label='Left Foot Contact Points (XY)')
    plt.plot(right_contact_points[:, 0], right_contact_points[:, 1], 'o-', markersize=2, label='Right Foot Contact Points (XY)')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Contact Points in XY Plane')
    plt.grid(True)
    plt.axis('equal') # Ensure equal scaling for X and Y axes
    plt.legend()
    ith_plot += 1

    plt.subplot(n_plots, 1, ith_plot)
    plt.plot(time_array, desired_hip_angle * 180/np.pi, label='Desired Hip Angle', linestyle='--')
    # The actual hip angle is the 8th element (index 7) of the logged state vector
    plt.plot(time_array, states[:, 7] * 180/np.pi, label='Actual Hip Angle') 
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (deg)')
    plt.title('Controller Hip Angle Tracking')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plot_filepath = os.path.join(base_dir, "sanity_check_plots.png")
    plt.savefig(plot_filepath)
    print(f"Sanity check plots saved to: {plot_filepath}")
    plt.close()


if __name__ == "__main__":
    main()
