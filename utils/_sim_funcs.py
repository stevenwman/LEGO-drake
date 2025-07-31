from simulate import SimConfig
from utils.helpers import *

def simulate(
    cfg: SimConfig,
    meshcat: Meshcat,
    N_simulation_steps: int, # This parameter will now mainly serve as an initial estimate
    simulation_time_step: float,
    start_state = None,
    calib: bool = False,
) -> tuple:
    """
    Run a Mugatu simulation.

    This function uses Drake's logging capabilities to collect data
    without an explicit simulation loop.
    """
    # Set up the Drake plant, scene graph and controller if needed
    plant, scene_graph, builder, instance = setup_walker_plant(
        timestep=simulation_time_step,
        filename=cfg.urdf_filename,
    )
    if start_state is None:
        start_state = get_home_state(cfg.start_height)

    controller = None # Initialize controller to None
    if not calib:
        controller = builder.AddSystem(Controller(cfg))
        builder.Connect(plant.get_state_output_port(), controller.GetInputPort("state"))
        builder.Connect(controller.get_output_port(), plant.get_actuation_input_port())
        
    # # Contact results collector
    # collision_pairs = [
    #     [ScopedName("walker", "left_foot"), ScopedName("walker", "ground")],
    #     [ScopedName("walker", "right_foot"), ScopedName("walker", "ground")],
    # ]
    # contact_results_system = builder.AddSystem(
    #     ContactResultsToArray(plant, scene_graph, collision_pairs)
    # )
    # builder.Connect(
    #     plant.get_contact_results_output_port(),
    #     contact_results_system.GetInputPort("contact_results"),
    # )

    # Visualisation
    meshcat.Delete()
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    ContactVisualizer.AddToBuilder(
        builder, plant, meshcat, ContactVisualizerParams(radius=0.002)
    )

    # Add loggers for states and control signals
    state_logger = LogVectorOutput(plant.get_state_output_port(), builder)
    state_logger.set_name("state_logger")

    if not calib:
        control_logger = LogVectorOutput(controller.get_output_port(), builder)
        control_logger.set_name("control_logger")

    diagram = builder.Build()
    simulator = Simulator(diagram)
    # simulator.set_target_realtime_rate(1.0)
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

    visualizer.StartRecording(False)

    # Advance the simulator to the end time
    simulator.AdvanceTo(cfg.duration)

    visualizer.PublishRecording()

    # Retrieve data from loggers
    states_log = state_logger.FindLog(context)
    states = states_log.data().transpose() # Transpose to get (N_steps, N_states)
    time_array = states_log.sample_times()
    
    # *** CRITICAL FIX: Determine actual number of simulation steps from logged data ***
    actual_N_simulation_steps = len(time_array)

    hip_real_torque = np.zeros((actual_N_simulation_steps, plant.num_actuators()))
    if not calib:
        control_log = control_logger.FindLog(context)
        # It's good practice to ensure control_log has compatible data
        if control_log.data().shape[1] == actual_N_simulation_steps:
            hip_real_torque = control_log.data().transpose() # Transpose to get (N_steps, N_actuators)
        else:
            print("Warning: Control log sample count mismatch with state log. This might indicate different logging rates or issues.")
            # You might want to resample or interpolate control_log data here if precise alignment is crucial
            # For now, we'll proceed with the available samples, or use the pre-initialized zero array if sizes don't match for safety
            # If control_log.data() is empty or too short, hip_real_torque remains zeros, which is safer than an error.

    # # Retrieve contact forces and points from ContactResultsToArray
    # force_dict, point_dict = contact_results_system.get_forces_and_points()
    force_dict, point_dict = {}, {}
    
    # Initialize arrays for contact forces and points with the actual number of steps
    left_contact_forces = np.zeros((actual_N_simulation_steps, 3))
    right_contact_forces = np.zeros((actual_N_simulation_steps, 3))
    left_contact_points = np.zeros((actual_N_simulation_steps, 3))
    right_contact_points = np.zeros((actual_N_simulation_steps, 3))

    # Populate contact force/point arrays from dictionaries
    # Loop over the actual number of recorded samples
    for i, t in enumerate(time_array):
        t_key = str(t)
        if t_key in force_dict:
            left_contact_forces[i] = force_dict[t_key].get('left_foot_force', np.zeros(3))
            right_contact_forces[i] = force_dict[t_key].get('right_foot_force', np.zeros(3))
            left_contact_points[i] = point_dict[t_key].get('left_foot_point', np.zeros(3))
            right_contact_points[i] = point_dict[t_key].get('right_foot_point', np.zeros(3))

    # Calculate COM data from logged states
    com_xyz = np.zeros((actual_N_simulation_steps, 3))
    com_vxyz = np.zeros((actual_N_simulation_steps, 3))
    com_per_link = {name: np.zeros((actual_N_simulation_steps, 3)) for name in link_names}

    # Create a temporary diagram for offline COM calculations
    com_builder = DiagramBuilder()
    com_plant, com_scene_graph = AddMultibodyPlantSceneGraph(com_builder, time_step=simulation_time_step)
    com_parser = Parser(com_plant)
    com_parser.AddModels(cfg.urdf_filename)
    com_plant.Finalize()
    com_instance = com_plant.GetModelInstanceByName("walker")
    com_diagram = com_builder.Build()
    com_context = com_diagram.CreateDefaultContext()
    com_plant_context = com_plant.GetMyContextFromRoot(com_context)

    # Loop over the actual number of recorded samples for COM calculations
    for idx in range(actual_N_simulation_steps):
        com_plant.SetPositionsAndVelocities(com_plant_context, states[idx])
        com_robot = com_plant.CalcCenterOfMassPositionInWorld(com_plant_context, [com_instance])
        com_xyz[idx] = com_robot
        com_vel = com_plant.CalcCenterOfMassTranslationalVelocityInWorld(com_plant_context, [com_instance])
        com_vxyz[idx] = com_vel
        
        for name in link_names:
            body = com_plant.GetBodyByName(name)
            X_WB = com_plant.EvalBodyPoseInWorld(com_plant_context, body)
            p_BoBcm_B = body.CalcCenterOfMassInBodyFrame(com_plant_context)
            p_WBcm = X_WB @ np.append(p_BoBcm_B, 1)
            com_per_link[name][idx] = p_WBcm[:3]

    frequency = cfg.frequency
    wait_time = cfg.wait_time

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
            N_simulation_steps=steps, # This N_simulation_steps is now mainly for preallocation/return shape
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
    # The N_simulation_steps here is now just for the shape of the output arrays,
    # as the actual simulation advances to cfg.duration
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