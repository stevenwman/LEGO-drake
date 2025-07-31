from simulate import SimConfig
from utils.helpers import *

def simulate(
    cfg: SimConfig,
    meshcat: Meshcat,
    N_simulation_steps: int,
    simulation_time_step: float,
    start_state = None,
    calib: bool = False,
) -> tuple:
    """
    Run a Mugatu simulation for a given number of steps.

    This function closely follows the original ``simulate`` but
    preallocates arrays and reduces printing.  It returns a tuple of
    arrays and metadata analogous to the original script.
    """
    # Set up the Drake plant, scene graph and controller if needed
    plant, scene_graph, builder, instance = setup_walker_plant(
        timestep=simulation_time_step,
        filename=cfg.urdf_filename,
    )
    if start_state is None:
        start_state = get_home_state(cfg.start_height)

    if not calib:
        controller = builder.AddSystem(Controller(cfg))
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
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    ContactVisualizer.AddToBuilder(
        builder, plant, meshcat, ContactVisualizerParams(radius=0.002)
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

    # body_com_history = []
    # def publish_all_coms():
    #     # Full-body COM in red
    #     body_com = plant.CalcCenterOfMassPositionInWorld(plant_context, [instance])
    #     meshcat.SetObject("COMs/robot_com", Sphere(0.01), rgba=Rgba(1, 0, 0, 1))
    #     meshcat.SetTransform("COMs/robot_com", RigidTransform(body_com))

    #     # Full-body COM trail in red
    #     trail_body = np.array(body_com_history[::2])
    #     if trail_body.ndim == 2 and trail_body.shape[0] > 0:
    #         trail_body = trail_body.T
    #         cloud_body = PointCloud(trail_body.shape[1])
    #         cloud_body.mutable_xyzs()[:3, :] = trail_body
    #         meshcat.SetObject("COMs/robot_com_trail", cloud_body, rgba=Rgba(1, 0, 0, 0.5))
    
    if not calib:
        controller_context = diagram.GetSubsystemContext(controller, context)
        controller_output_port = controller.get_output_port(0)

    frequency = cfg.frequency
    wait_time = cfg.wait_time

    visualizer.StartRecording(False)

    # Simulation loop
    for idx in range(N_simulation_steps):
        time_array = np.arange(N_simulation_steps) * simulation_time_step
        curr_time = idx * simulation_time_step
        # print time every simulation second
        if calib:
            if idx % int(1 / simulation_time_step) == 0:
                print(f"Simulation time: {curr_time:.2f}s, Step: {idx+1}/{N_simulation_steps}")
        else:
            controller_output = controller_output_port.Eval(controller_context)
            if idx % int(1 / simulation_time_step) == 0:
                print(f"Simulation time: {curr_time:.2f}s, Step: {idx+1}/{N_simulation_steps}, Control signal: {controller_output}", end='\r', flush=True)
            # # Update MeshCat visualization with all COMs
            # if cfg.visualize_coms and idx % 10 == 0:
            #     publish_all_coms()

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
    visualizer.PublishRecording()

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

def calibrate_quaternion(cfg: SimConfig, meshcat, new_state) -> np.ndarray:
    """
    Run a brief calibration simulation to average the quaternion.

    Parameters
    ----------
    cfg : SimConfig
        The simulation configuration.
    meshcat : Meshcat
        The meshcat instance used for visualisation.
    new_state : Optional[np.ndarray]
        A starting state for the calibration.  If ``None``, use the
        home state from ``get_home_state``.

    Returns
    -------
    np.ndarray
        A new initial state with calibrated quaternion and final position.
    """
    # Determine number of steps for calibration
    steps = int(3 * (1 / cfg.calib_time_step))
    # Run a short simulation with calibration flag
    for i in range(2):
        print(f"Calibrating quaternion orientation iter {i + 1}")
        states = simulate(
            cfg=cfg,
            meshcat=meshcat,
            N_simulation_steps=steps,
            simulation_time_step=cfg.calib_time_step,
            start_state=new_state,
            calib=True,
        )[0]
        # Compute average quaternion and final position
        final_pos = states[-1, 4:7]
        quats = states[:, 0:4]
        mean_quat = np.mean(quats, axis=0)
        mean_quat /= np.linalg.norm(mean_quat)
        new_state = get_home_state(cfg.start_height)
        new_state[0:4] = mean_quat
        new_state[4:7] = final_pos
    return new_state

def run_sim(cfg: SimConfig, meshcat) -> tuple:
    """Run a full simulation including calibration and return results."""
    # First calibrate quaternion orientation
    sim_time = int(cfg.duration * (1/cfg.sim_time_step)) #time in seconds
    start_state = get_home_state(cfg.start_height)
    stable_state = calibrate_quaternion(cfg, meshcat, start_state)
    # Determine number of steps based on duration and time step
    return simulate(
        cfg=cfg,
        meshcat=meshcat,
        N_simulation_steps=sim_time,
        simulation_time_step=cfg.sim_time_step,
        start_state=stable_state,
    )

def run_joint_sliders(cfg: SimConfig, meshcat):
    """Launch joint sliders for URDF visualisation."""
    plant, scene_graph, builder, instance = setup_walker_plant(
        timestep=cfg.sim_time_step,
        filename=cfg.urdf_filename,
    )
    sliders = builder.AddSystem(JointSliders(meshcat, plant))
    meshcat.Delete()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    ContactVisualizer.AddToBuilder(
        # builder, plant, meshcat, ContactVisualizerParams(radius=0.001 * cfg.scale)
        builder, plant, meshcat, ContactVisualizerParams(radius=0.001)
    )
    AddDefaultVisualization(builder, meshcat)
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    # Compute COM height for information
    com_robot = plant.CalcCenterOfMassPositionInWorld(plant_context, [instance])
    print("COM Height from Hip:", get_home_state(cfg.start_height)[6] - com_robot[2], "m")
    start_state = get_home_state(cfg.start_height)
    sliders.SetPositions(start_state[0 : plant.num_positions()])
    sliders.Run(diagram, None)