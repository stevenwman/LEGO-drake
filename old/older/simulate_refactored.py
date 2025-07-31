"""
Refactored simulation script for the Mugatu walker.

This module demonstrates how you could restructure the original
`simulate_mugatu.py` to make it easier to modify and potentially
more efficient.  Key changes include:

* A `SimConfig` dataclass encapsulating all tunable parameters.
* Top‑level functions instead of nested definitions.
* A `calibrate_quaternion` helper to perform the calibration loop.
* Preallocation of NumPy arrays for state and COM storage.
* Reduced printing during the simulation loop.

This file is meant to live on a separate branch or directory so it
doesn't overwrite the original script.  It reuses functions from
`utilities.py`, `plot_utilities.py` and `model_definition.py` in the
same package.  To run a simulation, call `main()` at the bottom of
the file.
"""

import argparse
import datetime
import os
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

# Import helper modules from the package
from utilities import get_amplitude, get_box_inertia, get_sphere_inertia
from plot_utilities import (
    plot_grf,
    plot_com_and_roc,
    plot_link_coms,
    plot_angles,
    plot_position_and_velocity,
    plot_controls,
    plot_power_and_energy,
    plot_stability_analysis,
)
from model_definition import (
    setup_walker_plant,
    get_home_state,
    ContactResultsToArray,
    Controller,
    urdf_file_name,
)
from old.meshcat_setup import start_meshcat

from pydrake.all import (
    MeshcatVisualizer,
    ContactVisualizer,
    ContactVisualizerParams,
    PointCloud,
    RigidTransform,
    RotationMatrix,
    Sphere,
    Rgba,
    ScopedName,
    Simulator,
    JointSliders,
    AddDefaultVisualization,
    
)


@dataclass
class SimConfig:
    """Container for tunable simulation parameters."""
    duration: float = 30.0
    scale: float = 1.0
    ground_friction: float = 0.9
    feet_friction: float = 0.9
    sim_time_step: float = 0.001
    calib_time_step: float = 0.01
    controller_period: float = 0.0005
    visualize_coms: bool = False


def calibrate_quaternion(
    cfg: SimConfig,
    meshcat,
    start_state: Optional[np.ndarray] = None,
) -> np.ndarray:
    """
    Run a brief calibration simulation to average the quaternion.

    Parameters
    ----------
    cfg : SimConfig
        The simulation configuration.
    meshcat : Meshcat
        The meshcat instance used for visualisation.
    start_state : Optional[np.ndarray]
        A starting state for the calibration.  If ``None``, use the
        home state from ``get_home_state``.

    Returns
    -------
    np.ndarray
        A new initial state with calibrated quaternion and final position.
    """
    # Determine number of steps for calibration
    steps = int(3 * (1 / cfg.calib_time_step))
    if start_state is None:
        start_state = get_home_state(cfg.scale)
    # Run a short simulation with calibration flag
    (
        states,
        _,
        _,
        _,
        _,
        _,
        _,
        _,
        _,
        _,
        _,
        _,
        _,
        _,
        _,
        _,
    ) = simulate(
        cfg=cfg,
        meshcat=meshcat,
        N_simulation_steps=steps,
        simulation_time_step=cfg.calib_time_step,
        controller_period=cfg.controller_period,
        start_state=start_state,
        calib=True,
    )
    # Compute average quaternion and final position
    start_idx = int(0.1 * states.shape[0])
    final_pos = states[-1, 4:7]
    quats = states[start_idx:, 0:4]
    mean_quat = np.mean(quats, axis=0)
    mean_quat /= np.linalg.norm(mean_quat)
    new_state = get_home_state(cfg.scale)
    new_state[0:4] = mean_quat
    new_state[4:7] = final_pos
    return new_state


def simulate(
    cfg: SimConfig,
    meshcat,
    N_simulation_steps: int,
    simulation_time_step: float,
    controller_period: float,
    start_state: Optional[np.ndarray] = None,
    calib: bool = False,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, np.ndarray, dict, float, float]:
    """
    Run a Mugatu simulation for a given number of steps.

    This function closely follows the original ``simulate`` but
    preallocates arrays and reduces printing.  It returns a tuple of
    arrays and metadata analogous to the original script.
    """
    # Set up the Drake plant, scene graph and controller if needed
    plant, scene_graph, builder, instance = setup_walker_plant(
        scale=cfg.scale,
        ground_friction=cfg.ground_friction,
        feet_friction=cfg.feet_friction,
        timestep=simulation_time_step,
        filename=urdf_file_name,
    )
    if start_state is None:
        start_state = get_home_state(cfg.scale)
    if not calib:
        controller = builder.AddSystem(
            Controller(
                scale=cfg.scale,
                ground_friction=cfg.ground_friction,
                feet_friction=cfg.feet_friction,
                control_period=controller_period,
                calib=calib,
            )
        )
        builder.Connect(plant.get_state_output_port(), controller.GetInputPort("state"))
        builder.Connect(controller.get_output_port(), plant.get_actuation_input_port())
    # Contact results collector
    collision_pairs = [
        [ScopedName("walker", "left_foot"), ScopedName("walker", "ground")],
        [ScopedName("walker", "right_foot"), ScopedName("walker", "ground")],
    ]
    contact_results_system = builder.AddSystem(
        ContactResultsToArray(plant, scene_graph, collision_pairs)
    )
    builder.Connect(
        plant.get_contact_results_output_port(),
        contact_results_system.GetInputPort("contact_results"),
    )
    # Visualisation
    meshcat.Delete()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    ContactVisualizer.AddToBuilder(
        builder, plant, meshcat, ContactVisualizerParams(radius=0.002 * cfg.scale)
    )
    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    context = simulator.get_mutable_context()
    plant_context = plant.GetMyContextFromRoot(context)
    # Set initial positions and velocities
    plant.SetPositionsAndVelocities(plant_context, start_state)
    total_mass = plant.CalcTotalMass(plant_context, [instance])
    body_indices = plant.GetBodyIndices(instance)
    exclude_names = {"ground", "inclined_plane"}
    link_names = [
        plant.get_body(index).name()
        for index in body_indices
        if plant.get_body(index).name() not in exclude_names
    ]
    # Preallocate storage
    n_states = plant.num_positions() + plant.num_velocities()
    states = np.zeros((N_simulation_steps, n_states))
    hip_real_torque = np.zeros((N_simulation_steps, plant.num_actuators()))
    com_xyz = np.zeros((N_simulation_steps, 3))
    com_vxyz = np.zeros((N_simulation_steps, 3))
    # Per‑link COMs stored in dict of arrays
    com_per_link = {name: np.zeros((N_simulation_steps, 3)) for name in link_names}
    # Contact forces and points stored in arrays
    left_contact_forces = np.zeros((N_simulation_steps, 3))
    right_contact_forces = np.zeros((N_simulation_steps, 3))
    left_contact_points = np.zeros((N_simulation_steps, 3))
    right_contact_points = np.zeros((N_simulation_steps, 3))
    # Save controller frequency and wait time if applicable
    if not calib:
        frequency = controller.frequency
        wait_time = controller.wait_time
        controller_context = diagram.GetSubsystemContext(controller, context)
        controller_output_port = controller.get_output_port(0)
    else:
        frequency = 0.0
        wait_time = 0.0
    # Simulation loop
    for idx in range(N_simulation_steps):
        # Advance simulator
        simulator.AdvanceTo(simulation_time_step * (idx + 1))
        # Record state
        states[idx] = plant.GetPositionsAndVelocities(plant_context)
        if not calib:
            hip_real_torque[idx] = controller.control_signal.copy()
        # Full robot COM and velocity
        com_robot = plant.CalcCenterOfMassPositionInWorld(plant_context, [instance])
        com_xyz[idx] = com_robot
        com_vel = plant.CalcCenterOfMassTranslationalVelocityInWorld(
            plant_context, [instance]
        )
        com_vxyz[idx] = com_vel
        # Individual link COMs
        for name in link_names:
            body = plant.GetBodyByName(name)
            X_WB = plant.EvalBodyPoseInWorld(plant_context, body)
            p_BoBcm_B = body.CalcCenterOfMassInBodyFrame(plant_context)
            p_WBcm = X_WB @ np.append(p_BoBcm_B, 1)
            com_per_link[name][idx] = p_WBcm[:3]
        # Contact forces via ContactResultsToArray
        # Query contact results at this step
        # Note: ContactResultsToArray caches results in dict keyed by string time;
        # here we use direct access to arrays instead of strings for efficiency.
        force_dict, point_dict = contact_results_system.get_forces_and_points()
        # Use current simulation time as key
        t_key = str(context.get_time())
        if t_key in force_dict:
            left_contact_forces[idx] = force_dict[t_key]["left_foot_force"]
            right_contact_forces[idx] = force_dict[t_key]["right_foot_force"]
            left_contact_points[idx] = point_dict[t_key]["left_foot_point"]
            right_contact_points[idx] = point_dict[t_key]["right_foot_point"]
    # Construct time array for caller
    time_array = np.arange(N_simulation_steps) * simulation_time_step
    return (
        states,
        hip_real_torque,
        left_contact_forces,
        left_contact_points,
        right_contact_forces,
        right_contact_points,
        com_xyz[:, 0],
        com_xyz[:, 1],
        com_xyz[:, 2],
        com_vxyz[:, 0],
        com_vxyz[:, 1],
        com_vxyz[:, 2],
        total_mass,
        time_array,
        com_per_link,
        frequency,
        wait_time,
    )


def run_joint_sliders(cfg: SimConfig, meshcat):
    """Launch joint sliders for URDF visualisation."""
    plant, scene_graph, builder, instance = setup_walker_plant(
        scale=cfg.scale,
        ground_friction=cfg.ground_friction,
        feet_friction=cfg.feet_friction,
        filename=urdf_file_name,
    )
    sliders = builder.AddSystem(JointSliders(meshcat, plant))
    meshcat.Delete()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    ContactVisualizer.AddToBuilder(
        builder, plant, meshcat, ContactVisualizerParams(radius=0.001 * cfg.scale)
    )
    AddDefaultVisualization(builder, meshcat)
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    # Compute COM height for information
    com_robot = plant.CalcCenterOfMassPositionInWorld(plant_context, [instance])
    print("COM Height from Hip:", get_home_state(cfg.scale)[6] - com_robot[2], "m")
    start_state = get_home_state(cfg.scale)
    sliders.SetPositions(start_state[0 : plant.num_positions()])
    sliders.Run(diagram, None)


def run_sim(
    cfg: SimConfig,
    meshcat,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, np.ndarray, dict, float, float, float]:
    """Run a full simulation including calibration and return results."""
    # First calibrate quaternion orientation
    start_state = calibrate_quaternion(cfg, meshcat)
    # Determine number of steps based on duration and time step
    N_steps = int(cfg.duration * (1 / cfg.sim_time_step))
    return simulate(
        cfg=cfg,
        meshcat=meshcat,
        N_simulation_steps=N_steps,
        simulation_time_step=cfg.sim_time_step,
        controller_period=cfg.controller_period,
        start_state=start_state,
    )


def run_sim_save_data(
    cfg: SimConfig,
    meshcat,
    plots_folder_path: str,
    csvs_folder_path: str,
    plot_data: bool,
    all_data_path: str,
) -> None:
    """Run a simulation and optionally save data and plots."""
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
    ) = run_sim(cfg, meshcat)
    # Parse state array into individual components
    qw = states[:, 0]
    qx = states[:, 1]
    qy = states[:, 2]
    qz = states[:, 3]
    x = states[:, 4]
    y = states[:, 5]
    z = states[:, 6]
    hip_real_angle = states[:, 7]
    omega_x_rad = states[:, 8]
    omega_y_rad = states[:, 9]
    omega_z_rad = states[:, 10]
    xdot = states[:, 11]
    ydot = states[:, 12]
    zdot = states[:, 13]
    hip_real_angvel = states[:, 14]
    # Determine stabilization period (placeholder: one second)
    stabilization_period = int(1 / cfg.sim_time_step)
    # Only plot if requested
    if plot_data:
        print("Plotting GRF data …")
        left_x, left_y, right_x, right_y, left_fx, left_fy, left_fz, right_fx, right_fy, right_fz = plot_grf(
            time_array,
            plots_folder_path,
            left_contact_forces,
            right_contact_forces,
            left_contact_points,
            right_contact_points,
            total_mass,
            stabilization_period=stabilization_period,
        )
        print("Plotting COM …")
        plot_com_and_roc(
            plots_folder_path,
            com_per_link,
            com_x,
            com_y,
            com_z,
            com_vx,
            com_vy,
            com_vz,
            left_contact_points,
            right_contact_points,
            cfg.sim_time_step,
            time_array,
            total_mass,
        )
        print("Plotting link COMs …")
        plot_link_coms(cfg.sim_time_step, plots_folder_path, com_per_link)
        print("Plotting angles …")
        rolls,
        pitches,
        yaws,
        rolls_degrees,
        pitches_degrees,
        yaws_degrees,
        pitch_rate,
        roll_rate,
        yaw_rate,
        omega_x,
        omega_y,
        omega_z,
        avg_roll_amp,
        avg_pitch_amp,
        avg_yaw_amp = plot_angles(
            time_array,
            plots_folder_path,
            qw,
            qx,
            qy,
            qz,
            omega_x_rad,
            omega_y_rad,
            omega_z_rad,
            stabilization_period,
            int(cfg.duration / cfg.sim_time_step),
            cfg.duration * (1 / cfg.sim_time_step),
            stabilization_period,
        )
        print("Plotting positions and velocities …")
        plot_position_and_velocity(
            time_array,
            plots_folder_path,
            x,
            y,
            z,
            com_x,
            com_y,
            com_z,
            xdot,
            ydot,
            zdot,
            left_x,
            left_y,
            right_x,
            right_y,
        )
        print("Plotting controls …")
        plot_controls(
            time_array,
            plots_folder_path,
            hip_real_torque,
            hip_real_angvel,
            hip_real_angle,
        )
        print("Plotting power …")
        total_power, cumulative_energy = plot_power_and_energy(
            time_array,
            plots_folder_path,
            hip_real_angvel,
            hip_real_torque,
            stabilization_period=stabilization_period,
        )
        print("Plotting stability …")
        plot_stability_analysis(
            time_array,
            plots_folder_path,
            0,
            len(time_array) - 1,
            cfg.duration,
            stabilization_period,
            pitches,
            pitch_rate,
            rolls,
            roll_rate,
            yaws,
            yaw_rate,
            com_x,
            com_y,
            com_z,
            com_vx,
            com_vy,
            com_vz,
        )
        # Write CSV
        data = {
            "scale": cfg.scale,
            "duration": cfg.duration,
            "time": time_array,
            "total_mass": total_mass,
            "ground_friction": cfg.ground_friction,
            "feet_friction": cfg.feet_friction,
            "qw": qw,
            "qx": qx,
            "qy": qy,
            "qz": qz,
            "x": x,
            "y": y,
            "z": z,
            "x_dot": xdot,
            "y_dot": ydot,
            "z_dot": zdot,
            "com_x": com_x,
            "com_y": com_y,
            "com_z": com_z,
            "com_vx": com_vx,
            "com_vy": com_vy,
            "com_vz": com_vz,
            "left_fx": left_fx,
            "left_fy": left_fy,
            "left_fz": -left_fz,
            "right_fx": right_fx,
            "right_fy": right_fy,
            "right_fz": -right_fz,
            "hip_real_angle": hip_real_angle,
            "hip_real_angvel": hip_real_angvel,
            "hip_real_torque": hip_real_torque.squeeze(),
            "total_power": total_power,
            "cumulative_energy": cumulative_energy,
            "contact_times": time_array,
        }
        import pandas as pd

        df = pd.DataFrame(data)
        csv_path = csvs_folder_path + ".csv"
        df.to_csv(csv_path, index=False)
        # Save meshcat HTML
        html_content = meshcat.StaticHtml()
        with open(os.path.join(all_data_path, "meshcat_viz.html"), "w") as f:
            f.write(html_content)
        print(f"Data saved to {csv_path}")


def main() -> None:
    """Parse CLI arguments and run the appropriate action."""
    parser = argparse.ArgumentParser(
        description="Run Mugatu walker simulation (refactored)."
    )
    parser.add_argument(
        "--simulate_walker",
        action="store_true",
        help="Run the walker simulation (otherwise launch joint sliders).",
    )
    parser.add_argument(
        "--save_data",
        action="store_true",
        help="Save output data to file.",
    )
    parser.add_argument(
        "--visualize_coms",
        action="store_true",
        help="Add COM trajectories to MeshCat visualisation.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Simulation duration in seconds (default 30).",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="Scale of the Mugatu walker (default 1).",
    )
    parser.add_argument(
        "--ground_friction",
        type=float,
        default=0.9,
        help="Ground friction coefficient.",
    )
    parser.add_argument(
        "--feet_friction",
        type=float,
        default=0.9,
        help="Feet friction coefficient.",
    )
    args = parser.parse_args()
    cfg = SimConfig(
        duration=args.duration,
        scale=args.scale,
        ground_friction=args.ground_friction,
        feet_friction=args.feet_friction,
        visualize_coms=args.visualize_coms,
    )
    meshcat = start_meshcat()
    if args.simulate_walker and args.save_data:
        # Prepare output paths
        now = datetime.datetime.now()
        label = now.strftime("%Y-%m-%d_%H-%M-%S")
        base_dir = os.path.join(os.getcwd(), "run_sim_save_data")
        alldata_folder = os.path.join(base_dir, f"data_for_{label}")
        plots_folder = os.path.join(alldata_folder, f"plots_{label}")
        os.makedirs(plots_folder, exist_ok=True)
        csv_base = os.path.join(alldata_folder, f"all_data_{label}")
        run_sim_save_data(
            cfg,
            meshcat,
            plots_folder_path=plots_folder,
            csvs_folder_path=csv_base,
            plot_data=True,
            all_data_path=alldata_folder,
        )
    elif args.simulate_walker:
        run_sim(cfg, meshcat)
    else:
        print("Opening joint sliders …")
        print(meshcat.web_url())
        run_joint_sliders(cfg, meshcat)


if __name__ == "__main__":
    main()
