import sys
import os
import argparse

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
sys.path.append(os.path.abspath(os.path.join(current_dir, "..")))

print(sys.version)

from utilities import *
from plot_utilities import *
from animate_utilities import *
from model_definition import *
from meshcat_setup import start_meshcat

import time

# Parse command-line arguments
parser = argparse.ArgumentParser(description="Run Mugatu walker simulation.")
parser.add_argument("--simulate_walker", action="store_true", help="Run the walker simulation (otherwise launch joint sliders for URDF visualization).")
parser.add_argument("--save_data", action="store_true", help="Save output data to file.")
parser.add_argument("--visualize_coms", action="store_true", help="Add COM trajectories to MeshCat visualization.")
parser.add_argument("--duration", type=float, default=30.0, help="Simulation duration in seconds (default is 30s).")
parser.add_argument("--scale", type=float, default=1, help="Scale of Mugatu walker (default is 1x).")
args = parser.parse_args()

duration = args.duration
simulate_walker = args.simulate_walker
save_data = args.save_data
visualize_coms = args.visualize_coms
scale = args.scale

file_name = urdf_file_name
print(f"Using URDF file: {file_name}")

def run_simulation():

    def run_joint_sliders(scale, ground_friction, feet_friction, start_state=None):
        plant, scene_graph, builder, instance = setup_walker_plant(
            scale = scale, 
            ground_friction = ground_friction, 
            feet_friction = feet_friction,
            filename = file_name)
        
        sliders = builder.AddSystem(JointSliders(meshcat,plant))
        # Setup visualization
        meshcat.Delete()
        visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
        ContactVisualizer.AddToBuilder(
            builder, plant, meshcat, ContactVisualizerParams(radius=0.001*scale)
        )
        # print(plant.get_discrete_contact_approximation()) 
        AddDefaultVisualization(builder,meshcat)
        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        
        # Get COM of robot for the entire robot
        plant_context = plant.GetMyContextFromRoot(context)
        com_robot = plant.CalcCenterOfMassPositionInWorld(plant_context, [instance])
        print("_"*120)
        print(f"COM Height from Hip: {get_home_state(scale)[6]-com_robot[2]}m")

        if start_state is None:
            start_state = get_home_state(scale)
        # print("num positions: ",plant.num_positions())
        sliders.SetPositions(start_state[0:plant.num_positions()])
        sliders.Run(diagram,None)

    def simulate( 
        scale,
        ground_friction,
        feet_friction,
        N_simulation_steps, #in miliseconds
        simulation_time_step, #sets timestep 
        controller_period, 
        start_state = get_home_state(scale),
        meshcat = meshcat, 
        calib=False
        ):

        # Setup Drake paramaters, initialize walker
        plant, scene_graph, builder, instance = setup_walker_plant(
            scale = scale, 
            ground_friction = ground_friction, 
            feet_friction = feet_friction,
            timestep = simulation_time_step,
            filename = file_name
            )
        if start_state is None:
            start_state = get_home_state(scale)
        # print("Starting simulation...")
        # print("start state:", start_state)


        # Setup controller
        controller = builder.AddSystem(Controller(scale = scale, 
                                                  hip_kp=1,
                                                  hip_kd=0,
                                                  ground_friction = ground_friction, 
                                                  feet_friction = feet_friction, 
                                                  control_period=controller_period, 
                                                  calib=calib))
        builder.Connect(plant.get_state_output_port(),controller.GetInputPort("state"))
        builder.Connect(controller.get_output_port(), plant.get_actuation_input_port())
        
        # Setup contact results:
        collision_pairs = [
            [ScopedName("walker", "left_foot"), ScopedName("walker", "ground")],
            [ScopedName("walker", "right_foot"), ScopedName("walker", "ground")],
        ]
        contact_results_system = builder.AddSystem(ContactResultsToArray(plant, scene_graph, collision_pairs))
        builder.Connect(
            plant.get_contact_results_output_port(),
            # contact_results_system.get_input_port("contact_results")
            contact_results_system.GetInputPort("contact_results")
        )

        # Setup visualization
        meshcat.Delete()
        visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
        ContactVisualizer.AddToBuilder(
            builder, plant, meshcat, ContactVisualizerParams(radius=0.002*scale)
        )
        diagram = builder.Build()

        # Set up a simulator to run this diagram
        simulator = Simulator(diagram)
        simulator.set_target_realtime_rate(1.0)
        context = simulator.get_mutable_context()
        plant_context = plant.GetMyContextFromRoot(context)

        # rotate walker because it imported incorrectly
        initial_rotation = RotationMatrix.MakeXRotation(-np.pi / 2)
        rotation = RigidTransform(initial_rotation)
        base_body = plant.GetBodyByName("left_leg") 
        plant.SetFreeBodyPose(plant_context, base_body, rotation)

        # set initial robot pose
        plant.SetPositionsAndVelocities(plant_context, start_state)

        # Get COM of robot for the entire robot
        com_robot = plant.CalcCenterOfMassPositionInWorld(plant_context, [instance])
        print("_"*120)
        print(f"COM Height from Hip: {get_home_state(scale)[6]-com_robot[2]}m")
        print("_"*120)
        # print("it's [X,Z,Y] because the COM is calculated in URDF coordinates and Y/Z are flipped for the URDF")
        
        # Simulate
        visualizer.StartRecording(False)
        # print(meshcat.web_url())

        # Initialize arrays to save data from sim
        simulated_states = [] # I'm too lazy to use numpy and count for now
        hip_real_torque = []
        
        left_contact_forces = []
        left_contact_points = []
        right_contact_forces = []
        right_contact_points = []

        # Save square wave frequency data
        frequency = controller.frequency
        wait_time = controller.wait_time

        # Initialize and save COM data
        com_x_positions = []
        com_y_positions = []
        com_z_positions = []
        com_vx = []
        com_vy = []
        com_vz = []

        total_mass = plant.CalcTotalMass(plant_context, [instance])
        # print("Total mass:", total_mass)
        body_indices = plant.GetBodyIndices(instance)
        exclude_names = {"ground", "inclined_plane"}
        link_names = [
            plant.get_body(index).name() 
            for index in body_indices 
            if plant.get_body(index).name() not in exclude_names
        ]
        com_history_dict = {name: [] for name in link_names} # COMs for visualization
        com_per_link = {name: [] for name in link_names} # array to save COMS for all links
        body_com_history = []

        def publish_all_coms():
            # Full-body COM in red
            body_com = plant.CalcCenterOfMassPositionInWorld(plant_context, [instance])
            meshcat.SetObject("COMs/robot_com", Sphere(0.01), rgba=Rgba(1, 0, 0, 1))
            meshcat.SetTransform("COMs/robot_com", RigidTransform(body_com))

            # Full-body COM trail in red
            trail_body = np.array(body_com_history[::2])
            if trail_body.ndim == 2 and trail_body.shape[0] > 0:
                trail_body = trail_body.T
                cloud_body = PointCloud(trail_body.shape[1])
                cloud_body.mutable_xyzs()[:3, :] = trail_body
                meshcat.SetObject("COMs/robot_com_trail", cloud_body, rgba=Rgba(1, 0, 0, 0.5))

        controller_context = diagram.GetSubsystemContext(controller, context)
        controller_output_port = controller.get_output_port(0) # Get the output port of the controller

        # Run simulation for timesteps
        for idx in range(N_simulation_steps):
            timer = context.get_time()
            controller_output = controller_output_port.Eval(controller_context)

            print(f"Simulation time: {timer:.2f}s, Step: {idx+1}/{N_simulation_steps}, Control signal: {controller_output}", end='\r', flush=True)
            print(""*200,end='\r', flush=True)

            simulator.AdvanceTo(simulation_time_step*(idx+1))

            # Save state data
            simulated_states.append(plant.GetPositionsAndVelocities(plant_context))
            hip_real_torque.append(controller.control_signal.copy())

            # Get robot COM
            com_robot = plant.CalcCenterOfMassPositionInWorld(plant_context, [instance])
            body_com_history.append(com_robot)
            com_x_positions.append(com_robot[0])
            com_y_positions.append(com_robot[1])
            com_z_positions.append(com_robot[2])
            com_robot_vel = plant.CalcCenterOfMassTranslationalVelocityInWorld(plant_context, [instance])
            com_vx.append(com_robot_vel[0])
            com_vy.append(com_robot_vel[1])
            com_vz.append(com_robot_vel[2])

            # Get individual link COMs
            for name in link_names:
                body = plant.GetBodyByName(name)
                X_WB = plant.EvalBodyPoseInWorld(plant_context, body)
                p_BoBcm_B = body.CalcCenterOfMassInBodyFrame(plant_context)
                p_WBcm = X_WB @ np.append(p_BoBcm_B, 1)  # homogeneous transform
                com_per_link[name].append(p_WBcm[:3])

            # Update MeshCat visualization with all COMs
            if visualize_coms and idx % 10 == 0:
                publish_all_coms()
            
        for name in com_per_link:
            com_per_link[name] = np.array(com_per_link[name])

        # Format data as np arrays and save contact forces
        simulated_states = np.array(simulated_states)
        hip_real_torque = np.array(hip_real_torque) # same as "simulated_inputs"
        force_dict, points_dict = contact_results_system.get_forces_and_points()
        contact_logging_times_str = np.array(list(force_dict.keys()))
        contact_logging_times = contact_logging_times_str.astype(np.float64)

        for time_key in contact_logging_times_str:
            # Access the forces for the current time step
            forces = force_dict[time_key]
            points = points_dict[time_key]

            # Append the forces and points to the respective lists
            left_contact_forces.append(forces['left_foot_force'])
            right_contact_forces.append(forces['right_foot_force'])
            left_contact_points.append(points['left_foot_point'])
            right_contact_points.append(points['right_foot_point'])
        
        visualizer.PublishRecording()
        xf =plant.GetPositionsAndVelocities(plant_context)
        print()
        print(f"{meshcat.web_url()}/download")

        return simulated_states, hip_real_torque, left_contact_forces, left_contact_points, right_contact_forces, right_contact_points, com_x_positions, com_y_positions, com_z_positions, com_vx, com_vy, com_vz, total_mass, contact_logging_times, com_per_link, frequency, wait_time

    def run_sim( scale, ground_friction, feet_friction, meshcat = meshcat):
        # run sim
        T = 0.001 #timestep
        sim_time = int(duration * (1/T)) #time in seconds

        t_calib = int(3 * (1/T))

        start_state = get_home_state(scale)

        for i in range(2):
            (states, 
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
                contact_logging_times,
                com_per_link,
                frequency, 
                wait_time
            ) = simulate(scale = scale,
                        ground_friction = ground_friction,
                        feet_friction = feet_friction,
                        N_simulation_steps = t_calib,
                        simulation_time_step = T,
                        controller_period = 0.0005,
                        meshcat = meshcat, 
                        start_state=start_state,
                        calib=True)
            
            print("Quaternion calibration done")

            start_idx = int(0.1 * states.shape[0])  # Start after 100ms
            final_pos = states[-1, 4:7]  # Get the final position
            quats = states[start_idx:, 0:4]
            mean_quat = np.mean(quats, axis=0)
            mean_quat /= np.linalg.norm(mean_quat)  # Normalize the quaternion
            
            start_state = get_home_state(scale)
            start_state[0:4] = mean_quat  # Set the calibrated quaternion to the start state
            start_state[4:7] = final_pos  # Set the final position to the start state

        (states, 
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
            contact_logging_times,
            com_per_link,
            frequency, 
            wait_time
        ) = simulate(scale = scale,
                    ground_friction = ground_friction,
                    feet_friction = feet_friction,
                    N_simulation_steps = sim_time,
                    simulation_time_step = T,
                    controller_period = 0.0005,
                    meshcat = meshcat, 
                    start_state=start_state)

        for i in range(10):
            time.sleep(1)  # wait for the recording to finish
            print(f"Waiting for recording to finish... {i+1}/10", end='\r', flush=True)
        
        return states, hip_real_torque, left_contact_forces, left_contact_points, right_contact_forces, right_contact_points, com_x, com_y, com_z, com_vx, com_vy, com_vz, total_mass, contact_logging_times, com_per_link, frequency, wait_time, T 


    def run_sim_save_data(scale, ground_friction, feet_friction, plots_folder_path, csvs_folder_path, plot_data, meshcat = meshcat):
        states, hip_real_torque, left_contact_forces, left_contact_points, right_contact_forces, right_contact_points, com_x, com_y, com_z, com_vx, com_vy, com_vz, total_mass, contact_logging_times, com_per_link, frequency, wait_time, T = run_sim(scale = scale, ground_friction = ground_friction, feet_friction = feet_friction, meshcat = meshcat)

        # get state from sim
        qw = [state[0] for state in states]
        qx = [state[1] for state in states]
        qy = [state[2] for state in states]
        qz = [state[3] for state in states]
        x = [state[4] for state in states]
        y = [state[5] for state in states]
        z = [state[6] for state in states]
        hip_real_angle = [state[7] for state in states]
        omega_x_rad = [state[8] for state in states]
        omega_y_rad = [state[9] for state in states]
        omega_z_rad = [state[10] for state in states]
        roll_rates = [state[8] for state in states]
        yaw_rates = [state[9] for state in states]
        pitch_rates = [state[10] for state in states]
        xdot = [state[11] for state in states]
        ydot = [state[12] for state in states]
        zdot = [state[13] for state in states]
        hip_real_angvel = [state[14] for state in states]


        # other key variables for plots and data saving
        N = len(qw)
        time_array = np.arange(N) * T
        start_time = 2 # Actuation starts after 2 seconds
        period = wait_time * 2
        end_time = start_time + period
        start_idx = np.searchsorted(time_array, start_time)
        end_idx = np.searchsorted(time_array, end_time, side='right')
        # stabilization_period = int(5 * (1/T))
        stabilization_period = 1
        print("Total mass in kg:", total_mass)

        delta_x = np.diff(x)
        delta_y = np.diff(y)
        v = np.sum(np.sqrt(delta_x**2 + delta_y**2))/29.9
        v_dist= np.sum(np.sqrt(delta_x**2 + delta_y**2))
        alt_v = np.square(np.max(xdot[stabilization_period:])**2 + np.max(ydot[stabilization_period:])**2)
        alt_v2_dist = np.square(x[-1]**2 + y[-1]**2)
        alt_v2 = alt_v2_dist/29.9 
        hip_real_angvel = np.array(hip_real_angvel)
        
        if plot_data == True:

            # plot_data
            print("_"*120)
            print("Plotting GRF data...")
            left_x, left_y, right_x, right_y, left_fx, left_fy, left_fz, right_fx, right_fy, right_fz = plot_grf(time_array,
                    plots_folder_path, 
                    left_contact_forces,
                    right_contact_forces,
                    left_contact_points,
                    right_contact_points, 
                    total_mass, 
                    stabilization_period = stabilization_period)
            print("_"*120)

            print("Plotting COM of robot data...")
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
                T,
                time_array,
                total_mass)
            print("_"*120)
            
            print("Plotting COMs of individual links data...")
            plot_link_coms(T, 
                    plots_folder_path, 
                    com_per_link)
            print("_"*120)
            
            print("Plotting RPY data....")
            rolls, pitches, yaws, rolls_degrees, pitches_degrees, yaws_degrees, pitch_rate, roll_rate, yaw_rate, omega_x, omega_y, omega_z, avg_roll_amp, avg_pitch_amp, avg_yaw_amp = plot_angles(time_array,
                                                                            plots_folder_path,
                                                                            qw, 
                                                                            qx, 
                                                                            qy, 
                                                                            qz, 
                                                                            omega_x_rad, 
                                                                            omega_y_rad, 
                                                                            omega_z_rad,
                                                                            start_idx,
                                                                            end_idx,
                                                                            duration * (1/T), 
                                                                            stabilization_period)
            print("_"*120)

            print("Plotting position and velocity data...")
            plot_position_and_velocity(time_array,
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
                                    right_y)
            print("_"*120)

            print("Plotting controls and motor data...")
            plot_controls(time_array,
                        plots_folder_path,
                        hip_real_torque,
                        hip_real_angvel,
                        hip_real_angle)
            print("_"*120)

            print("Plotting power and energy data...")
            total_power,cumulative_energy = plot_power_and_energy(time_array,
                                    plots_folder_path,
                                    hip_real_angvel,
                                    hip_real_torque,
                                    stabilization_period = stabilization_period)
            print("_"*120)

            print("Plotting stability data...")
            plot_stability_analysis(time_array,
                            plots_folder_path,
                            start_idx,
                            end_idx,
                            duration,
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
                            com_vz
                            )
            print("_"*120)

            # save_data
            data = {
                "scale": scale,
                "duration": duration,
                "time_array": time_array,
                "total_mass": total_mass,
                "Ground_mu-dynamic_value": ground_friction,
                "Ground_mu-static value=": ground_friction,
                "Feet_mu-dynamic value": feet_friction,
                "Feet_mu-static value": feet_friction,
                # "Pitch_degrees": pitches_degrees,
                # "Roll_degrees": rolls_degrees,
                # "Yaw_degrees": yaws_degrees,
                # "Pitch_radians": pitches,
                # "Average_pitch_amplitude_after_stable_walking_degrees": avg_pitch_amp,
                # "Roll_radians": rolls,
                # "Average_roll_amplitude_after_stable_walking_degrees": avg_roll_amp,
                # "Yaw_radians": yaws,
                # "Average_yaw_amplitude_after_stable_walking_degrees": avg_yaw_amp,
                "qw": qw,
                "qx": qx,
                "qy":qy,
                "qz": qz, 
                "x": x, 
                "y": y, 
                "z": z, 
                "x_dot": xdot,
                "y_dot": ydot, 
                "z_dot": zdot, 
                "COM_x_position":com_x,
                "COM_y_position":com_y,
                "COM_z_position":com_z,
                "COM_x_velocity":com_vx, 
                "COM_y_velocity":com_vy, 
                "COM_z_velocity":com_vz,
                # "Roll_rate_radpersec": roll_rate,
                # "Pitch_rate_radpersec": pitch_rate, 
                # "Yaw_rate_radpersec": yaw_rate, 
                # "X_euler _rate_radpersec": omega_x,
                # "Y_euler_rate_radpersec": omega_y,
                # "Z_euler_rate_radpersec": omega_z, 
                "Left_foot_fx": left_fx,
                "Left_foot_fy": left_fy,
                "Left_foot_fz": -left_fz,
                "Right_foot_fx": right_fx,
                "Right_foot_fy": right_fy,
                "Right_foot_fz": -right_fz,
                "delta_velocity": v,
                "incremental_velocity": alt_v,
                "displacement_velocity": alt_v2,
                "min_z": np.min(z),
                "alt_v2_dist": alt_v2_dist,
                "v_distance": v_dist,
                "hip_real_angle": hip_real_angle,
                "hip_real_angvel":hip_real_angvel,
                "hip_real_torque": hip_real_torque,
                "total_power": total_power,
                "cumulative_energy": cumulative_energy,
                "contact_logging_times": contact_logging_times
            }
            # df = pd.DataFrame(dict([(k, pd.Series(v)) for k, v in data.items()]))
            data_1d = {k: np.asarray(v).squeeze()   
            for k, v in data.items()}
            df = pd.DataFrame(data_1d) 

            csvs_folder_path +=".csv"
            df.to_csv(csvs_folder_path, index=True)
            print(f"Sim data saved to {csvs_folder_path}")
        else:
            pass

    # Create the folder path
    time_now = datetime.datetime.now()
    label = time_now.strftime("%Y-%m-%d_%H-%M-%S")
    plot_folder_name = f"plots_{label}"
    alldata_output_folder = os.path.join(current_dir, 'run_sim_save_data')
    print("alldata_output_folder:", alldata_output_folder)
    alldata_folder_name = f"data_for_{label}"
    alldata_folder_path = os.path.join(alldata_output_folder, alldata_folder_name)
    plots_folder_path = os.path.join(alldata_folder_path, plot_folder_name)

    os.makedirs(alldata_folder_path, exist_ok=True) # create the new folder if it doesn't exist
    os.makedirs(plots_folder_path, exist_ok=True) # create the new folder if it doesn't exist
    print("_"*120)
    print(f"Created folder: {alldata_folder_path}")
    csvname = f"all_data_{label}"
    print("_"*120)
    csvs_path = os.path.join(alldata_folder_path, csvname)

    # Set parameters
    ground_friction = 0.9 # set to 0.4 for scale <= 1 or 0.9 for scale > 1
    feet_friction = 0.9 # set to 0.7 for scale <= 1 or 0.9 for scale > 1

    # Run sim and save data, run sim, or run visualizer
    if simulate_walker & save_data:
        run_sim_save_data(scale = scale, ground_friction = ground_friction, feet_friction=feet_friction, plots_folder_path = plots_folder_path, csvs_folder_path = csvs_path, plot_data = save_data, meshcat = meshcat)
    elif simulate_walker:
        run_sim(scale = scale, ground_friction = ground_friction, feet_friction=feet_friction, meshcat = meshcat)
    else:
        print("Running joint_sliders...")
        print(f"Go up to where it says Click the link and click the link:")
        print(meshcat.web_url())
        run_joint_sliders(scale = scale, ground_friction = ground_friction, feet_friction=feet_friction)


if __name__ == "__main__":
    print("Starting Mugatu walker simulation...")
    meshcat = start_meshcat()
    run_simulation()
