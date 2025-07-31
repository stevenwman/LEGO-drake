# In files.zip/utils/sim_funcs.py

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
        builder.Connect(controller.get_output_port(0), plant.get_actuation_input_port())
        
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

    # Add loggers for states and control signals
    state_logger = LogVectorOutput(plant.get_state_output_port(), builder)
    state_logger.set_name("state_logger")

    if not calib:
        control_logger = LogVectorOutput(controller.get_output_port(0), builder)
        control_logger.set_name("control_logger")
        # Log only the desired hip angle from its specific port
        angle_logger = LogVectorOutput(controller.GetOutputPort("desired_hip_angle"), builder)
        angle_logger.set_name("angle_logger")

    contact_logger = LogVectorOutput(contact_results_system.get_output_port(0), builder) # Use index 0 for the first output port
    contact_logger.set_name("contact_logger")

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(0.0) # Ensure this is 0.0 for max speed
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
    
    actual_N_simulation_steps = len(time_array)

    hip_real_torque = np.zeros((actual_N_simulation_steps, plant.num_actuators()))
    desired_hip_angle = np.zeros(actual_N_simulation_steps)

    if not calib:
        control_log = control_logger.FindLog(context)
        if control_log.data().shape[1] == actual_N_simulation_steps:
            hip_real_torque = control_log.data().transpose()
        else:
            print("Warning: Control log sample count mismatch with state log. This might indicate different logging rates or issues.")
            # If control_log.data() is empty or too short, hip_real_torque remains zeros, which is safer than an error.
        # Retrieve the desired angle log
        angle_log = angle_logger.FindLog(context)
        if angle_log.data().shape[1] == actual_N_simulation_steps:
            # Flatten the (1, N) data array into a simple 1D array
            desired_hip_angle = angle_log.data().flatten()

    # NEW: Retrieve combined contact data from the logger and split it
    contact_log = contact_logger.FindLog(context)
    combined_contact_data = contact_log.data().transpose() # This will be (N_steps, 12)
    
    left_contact_forces = combined_contact_data[:, 0:3]
    right_contact_forces = combined_contact_data[:, 3:6]
    left_contact_points = combined_contact_data[:, 6:9]
    right_contact_points = combined_contact_data[:, 9:12]


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
        desired_hip_angle,
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
        # Assign the parameters to the instance variables
        
        #hip_kp = 1 for scale = 0.166 (Zippy scale), hip_kp = 8000 for scale 6.67 (Big Foot scale)
        self.hip_kp, self.hip_ki, self.hip_kd = cfg.hip_kp, cfg.hip_ki, cfg.hip_kd
        self.control_period = cfg.control_period

        self.integral_error = 0
        self.time = 0
        # real frequency is calculted by f = (1/2pi) * (g/l)^(1/2) where l is *debatable, likely distance from ground to body COM height
        # based on 1.4Hz, Mugatu's COM should be 12.67 cm below the hip
        # based on the COM height reported in the paper, Mugatu's walking frequency should be 1.93 (assuming l is 6.6cm)
        self.frequency = cfg.frequency
        self.amplitude = cfg.amplitude
        self.wait_time = cfg.wait_time
        self.counter = cfg.counter
    
        """ For now, just a pd control tracking 1 state."""
        LeafSystem.__init__(self)
        self.controller_plant, self.scene_graph, self.controller_diagram, self.instance = setup_walker_controller_plant(
            filename=cfg.urdf_filename)

        # Init context
        self.controller_diagram_context = self.controller_diagram.CreateDefaultContext()
        self.controller_plant_context = self.controller_plant.GetMyContextFromRoot(self.controller_diagram_context)
        
        self.n = self.controller_diagram_context.get_discrete_state_vector().size()
        self.n_pos = self.controller_plant.num_positions()
        self.m = self.controller_plant.num_actuators()

        # Init control signal
        self.control_signal = np.zeros(self.m )

        # Init state measurement
        self.current_state = np.zeros(self.n)
        self.prev_state = self.current_state

        # Init FT measurements
        orientation_cartesian_dim = 6

        #Init target state
        self.target_state = get_home_state(cfg.start_height)
        # print("Init target state:", self.target_state)

        # Specify inputs and outputs
        self.state_input_index = self.DeclareVectorInputPort("state", self.n).get_index()

        self.DeclareVectorOutputPort("control", self.m , self.SetOutput)
        self.DeclareVectorOutputPort("desired_hip_angle", 1, self.SetDesiredAngleOutput)
        # Add periodic update event
        self.DeclarePeriodicDiscreteUpdateEvent(self.control_period, 0, self.Update)
    
    @staticmethod
    def ComputeControl(self, current_state, desired_state):
        pos_error = desired_state[7] - current_state[7]
        vel_error = desired_state[14] - current_state[14]
        # self.derivative_error = (self.current_state[7] - self.prev_state[7]) / self.control_period
        # feedback_input = self.hip_kp*pos_error+ self.hip_kd * self.derivative_error
        feedback_input = self.hip_kp * pos_error + self.hip_kd * vel_error
        return feedback_input

    def Update(self, context, events):
        # get the current state
        self.current_state = self.get_input_port(int(self.state_input_index)).Eval(context)

        # set the state in our internal robot model
        self.controller_plant.SetPositionsAndVelocities(self.controller_plant_context,self.current_state)
        """ 
        We can compute any rigid body dynamics quantities here now with controller plant. 
        List is here https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_multibody_plant.html
        For example: plant.CalcMassMatrix, plant.CalcBiasTerm (coriollis*v)
        Save a class variable for these quantities and then you can just grab it after each timestep.
        """
        elapsed_time = context.get_time()
        act_start_time = 1
        adjusted_time = elapsed_time - act_start_time

        ang_freq = 2 * np.pi * self.frequency
        servo_input = self.amplitude * np.sin(ang_freq * adjusted_time)
        servo_input_vel = self.amplitude * ang_freq * np.cos(ang_freq * adjusted_time)
        # print("servo_input", servo_input)
        if elapsed_time > act_start_time: # give time for sim to sit and settle
            self.target_state[7] = servo_input # comment this out when you're getting sim natural frequency
            self.target_state[14] = servo_input_vel

        self.control_signal[:] = self.ComputeControl(self,
            current_state=self.current_state,
            desired_state=self.target_state,
            )

    def SetOutput(self, context, output):
        output.SetFromVector(self.control_signal)

    # Add this new callback method to the Controller class:
    def SetDesiredAngleOutput(self, context, output):
        """Callback for the desired_hip_angle output port."""
        # self.target_state[7] holds the desired hip angle.
        output.SetFromVector([self.target_state[7]])