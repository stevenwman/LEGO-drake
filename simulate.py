
import argparse
from dataclasses import dataclass
from old.meshcat_setup import start_meshcat
import datetime
import os
import numpy as np
from pydrake.all import *
from utils.sim_funcs import *
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
    hip_kp: float = 8
    hip_ki: float = 0
    hip_kd: float = 0.001
    threshold_force: float = 0.0001  # in N, To set contact mode
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
    (
        states,
        hip_real_torque,
        left_contact_forces,
        left_contact_points,
        right_contact_forces,
        right_contact_points,
        com_x,
        com_y,
        com_z,
        com_vx,
        com_vy,
        com_vz,
        total_mass,
        time_array,
        com_per_link,
        frequency,
        wait_time,
    ) = run_sim(config, meshcat)

    print(f"Simulation completed. Data saved to {base_dir}")

    # --- Plotting for sanity check ---
    print("Generating plots for sanity check...")

    # Calculate magnitudes for contact forces
    left_force_magnitude = np.linalg.norm(left_contact_forces, axis=1)
    right_force_magnitude = np.linalg.norm(right_contact_forces, axis=1)

    # Change the figure size and subplot layout to accommodate the new plot
    plt.figure(figsize=(12, 24)) # Increased height to make space for 4 subplots

    # Plot 1: Contact Force Magnitudes
    plt.subplot(4, 1, 1) # Changed to 4 rows
    plt.plot(time_array, left_force_magnitude, label='Left Foot Contact Force Magnitude')
    plt.plot(time_array, right_force_magnitude, label='Right Foot Contact Force Magnitude')
    plt.xlabel('Time (s)')
    plt.ylabel('Force Magnitude (N)')
    plt.title('Contact Force Magnitudes Over Time')
    plt.grid(True)
    plt.legend()

    # Plot 2: Center of Mass Position
    plt.subplot(4, 1, 2) # Changed to 4 rows
    plt.plot(time_array, com_x, label='CoM X')
    plt.plot(time_array, com_y, label='CoM Y')
    plt.plot(time_array, com_z, label='CoM Z')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title('Center of Mass Position Over Time')
    plt.grid(True)
    plt.legend()

    # Plot 3: Center of Mass Velocity
    plt.subplot(4, 1, 3) # Changed to 4 rows
    plt.plot(time_array, com_vx, label='CoM Vx')
    plt.plot(time_array, com_vy, label='CoM Vy')
    plt.plot(time_array, com_vz, label='CoM Vz')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Center of Mass Velocity Over Time')
    plt.grid(True)
    plt.legend()

    # Plot 4: Contact Points in XY Plane
    plt.subplot(4, 1, 4) # New subplot for XY contact points
    plt.plot(left_contact_points[:, 0], left_contact_points[:, 1], 'o-', markersize=2, label='Left Foot Contact Points (XY)')
    plt.plot(right_contact_points[:, 0], right_contact_points[:, 1], 'o-', markersize=2, label='Right Foot Contact Points (XY)')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Contact Points in XY Plane')
    plt.grid(True)
    plt.axis('equal') # Ensure equal scaling for X and Y axes
    plt.legend()

    plt.tight_layout()
    plot_filepath = os.path.join(base_dir, "sanity_check_plots.png")
    plt.savefig(plot_filepath)
    print(f"Sanity check plots saved to: {plot_filepath}")
    plt.close()


    # # If you also want to save the raw data to CSVs
    # np.savetxt(os.path.join(base_dir, "time_array.csv"), time_array, delimiter=",")
    # np.savetxt(os.path.join(base_dir, "states.csv"), states, delimiter=",")
    # np.savetxt(os.path.join(base_dir, "hip_real_torque.csv"), hip_real_torque, delimiter=",")
    # np.savetxt(os.path.join(base_dir, "left_contact_forces.csv"), left_contact_forces, delimiter=",")
    # np.savetxt(os.path.join(base_dir, "right_contact_forces.csv"), right_contact_forces, delimiter=",")
    # np.savetxt(os.path.join(base_dir, "left_contact_points.csv"), left_contact_points, delimiter=",")
    # np.savetxt(os.path.join(base_dir, "right_contact_points.csv"), right_contact_points, delimiter=",")
    # np.savetxt(os.path.join(base_dir, "com_xyz.csv"), np.column_stack((com_x, com_y, com_z)), delimiter=",")
    # np.savetxt(os.path.join(base_dir, "com_vxyz.csv"), np.column_stack((com_vx, com_vy, com_vz)), delimiter=",")
    # print(f"Raw data CSVs saved to: {base_dir}")


if __name__ == "__main__":
    main()
