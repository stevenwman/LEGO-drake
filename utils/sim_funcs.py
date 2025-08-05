# In files.zip/utils/sim_funcs.py

from simulate import SimConfig
from utils.helpers import *

def simulate(
    cfg: SimConfig,
    simulation_time_step: float,
    meshcat: Meshcat = None,
    start_state = None,
    calib: bool = False,
) -> tuple:
    """
    Run a Mugatu simulation.
    """
    # Set up the Drake plant and scene graph
    plant, scene_graph, builder, instance = setup_walker_plant(
        timestep=simulation_time_step,
        filename=cfg.urdf_filename,
    )
    if start_state is None:
        start_state = get_home_state(cfg.start_height)

    # Add the controller and loggers if not in calibration mode
    if not calib:
        controller = builder.AddSystem(Controller(cfg))
        builder.Connect(plant.get_state_output_port(), controller.GetInputPort("state"))
        builder.Connect(controller.GetOutputPort("control"), plant.get_actuation_input_port())
        
        control_logger = LogVectorOutput(controller.GetOutputPort("control"), builder)
        angle_logger = LogVectorOutput(controller.GetOutputPort("desired_hip_angle"), builder)

    # Add contact results system and logger
    collision_pairs = [[ScopedName("walker", "left_foot"), ScopedName("walker", "ground")],
                       [ScopedName("walker", "right_foot"), ScopedName("walker", "ground")]]
    contact_results_system = builder.AddSystem(ContactResultsToArray(plant, scene_graph, collision_pairs))
    builder.Connect(plant.get_contact_results_output_port(), contact_results_system.GetInputPort("contact_results"))
    contact_logger = LogVectorOutput(contact_results_system.get_output_port(0), builder)

    # Add visualizer and state logger
    if meshcat is not None:
        visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
        ContactVisualizer.AddToBuilder(
            builder, plant, meshcat, ContactVisualizerParams(radius=0.001)
        )
    state_logger = LogVectorOutput(plant.get_state_output_port(), builder)
    
    # Build and run the simulation
    diagram = builder.Build()
    simulator = Simulator(diagram)
    context = simulator.get_mutable_context()
    plant_context = plant.GetMyContextFromRoot(context)
    plant.SetPositionsAndVelocities(plant_context, start_state)
    
    if meshcat is not None:
        meshcat.Delete()
        visualizer.StartRecording(False)
        simulator.AdvanceTo(cfg.duration)
        visualizer.PublishRecording()
    else:
        simulator.AdvanceTo(cfg.duration)

    # --- Post-Processing ---
    # Retrieve logged data
    states_log = state_logger.FindLog(context)
    states = states_log.data().transpose()
    time_array = states_log.sample_times()
    num_steps = len(time_array)

    # Retrieve optional logged data
    hip_real_torque = np.zeros((num_steps, 1))
    desired_hip_angle = np.zeros(num_steps)
    if not calib:
        hip_real_torque = control_logger.FindLog(context).data().transpose()
        desired_hip_angle = angle_logger.FindLog(context).data().flatten()

    contact_data = contact_logger.FindLog(context).data().transpose()
    left_contact_forces = contact_data[:, 0:3]
    right_contact_forces = contact_data[:, 3:6]
    # And add these lines right after it to extract the points:
    left_contact_points = contact_data[:, 6:9]
    right_contact_points = contact_data[:, 9:12]
    
    # Efficiently calculate COM data using the existing simulation plant
    com_xyz = np.zeros((num_steps, 3))
    com_vxyz = np.zeros((num_steps, 3))
    for i in range(num_steps):
        plant.SetPositionsAndVelocities(plant_context, states[i, :])
        com_xyz[i] = plant.CalcCenterOfMassPositionInWorld(plant_context, [instance])
        com_vxyz[i] = plant.CalcCenterOfMassTranslationalVelocityInWorld(plant_context, [instance])
        
    # Further down, update the return statement to include the contact points:
    return (states, hip_real_torque, desired_hip_angle, left_contact_forces, 
            left_contact_points, right_contact_forces, right_contact_points, 
            com_xyz, com_vxyz, time_array)

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
        # print(f"Calibrating quaternion orientation iter {i + 1}")
        states = simulate(
            cfg=cfg,
            meshcat=meshcat,
            # N_simulation_steps=steps, # This N_simulation_steps is now mainly for preallocation/return shape
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

def run_sim(cfg: SimConfig, meshcat: Meshcat = None) -> tuple:
    """Run a full simulation including calibration and return results."""
    # First calibrate quaternion orientation
    # sim_time = int(cfg.duration * (1/cfg.sim_time_step)) #time in seconds
    start_state = get_home_state(cfg.start_height)
    stable_state = calibrate_quaternion(cfg, meshcat, start_state)
    # The N_simulation_steps here is now just for the shape of the output arrays,
    # as the actual simulation advances to cfg.duration
    return simulate(
        cfg=cfg,
        simulation_time_step=cfg.sim_time_step,
        meshcat=meshcat,
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


class Controller(LeafSystem):
    def __init__(self, cfg: SimConfig):
        LeafSystem.__init__(self)
        # Store only the parameters needed for the PD control
        self.hip_kp = cfg.hip_kp
        self.hip_kd = cfg.hip_kd
        self.frequency = cfg.frequency
        self.amplitude = cfg.amplitude

        # Define the size of the state vector and actuator command
        self.num_states = 15
        self.num_actuators = 1

        # Initialize the target state vector for the hip joint
        self.target_state = np.zeros(self.num_states)

        # Declare the input port for the robot's state
        self.DeclareVectorInputPort("state", self.num_states)
        
        # Declare the output port for the control signal (torque)
        self.DeclareVectorOutputPort("control", self.num_actuators, self.SetControlOutput)
        
        # Declare the output port for the desired hip angle for logging
        self.DeclareVectorOutputPort("desired_hip_angle", 1, self.SetDesiredAngleOutput)

    def SetControlOutput(self, context, output):
        """Calculates and sets the PD control output."""
        # Get the current state from the input port
        current_state = self.get_input_port(0).Eval(context)
        
        # Update the desired trajectory based on time
        elapsed_time = context.get_time()
        act_start_time = 1
        if elapsed_time > act_start_time:
            adjusted_time = elapsed_time - act_start_time
            ang_freq = 2 * np.pi * self.frequency
            self.target_state[7] = self.amplitude * np.sin(ang_freq * adjusted_time)
            self.target_state[14] = self.amplitude * ang_freq * np.cos(ang_freq * adjusted_time) # Correct index for hip velocity is 14
        
        # Calculate PD control errors
        pos_error = self.target_state[7] - current_state[7]
        vel_error = self.target_state[14] - current_state[14] # Correct index for hip velocity is 14

        # Compute the feedback torque
        feedback_torque = self.hip_kp * pos_error + self.hip_kd * vel_error
        output.SetFromVector([feedback_torque])

    def SetDesiredAngleOutput(self, context, output):
        """Outputs the desired hip angle for logging."""
        output.SetFromVector([self.target_state[7]])