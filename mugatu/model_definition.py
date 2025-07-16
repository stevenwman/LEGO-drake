
from utilities import *
import os 
curr_dir = os.path.dirname(os.path.abspath(__file__))


'''---------------------------------------Set up walker model--------------------------------------'''

urdf_file_name = "quick_sim/stick_bot_generated_fixed_inertia.urdf"


def setup_walker_plant(scale, ground_friction, feet_friction, timestep=0.001, filename=None):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=timestep)
    parser = Parser(plant)
    # parser.AddModelsFromString(create_walker_urdf(scale = scale, ground_friction = ground_friction, feet_friction = feet_friction),"urdf")
    parser.AddModels(filename)
    # plant.set_discrete_contact_approximation(DiscreteContactApproximation.kSap)
    plant.Finalize()
    instance = plant.GetModelInstanceByName("walker")
    return plant, scene_graph, builder, instance

def setup_walker_controller_plant(scale, ground_friction, feet_friction, timestep=0.001, filename=None):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=timestep)
    parser = Parser(plant)
    # parser.AddModelsFromString(create_walker_urdf(scale = scale, ground_friction = ground_friction, feet_friction = feet_friction), "urdf")
    parser.AddModels(filename)
    plant.set_discrete_contact_approximation(DiscreteContactApproximation.kLagged)
    plant.Finalize()
    diagram = builder.Build()
    instance = plant.GetModelInstanceByName("walker")
    return plant, scene_graph, diagram, instance


def get_home_state(scale):
    """spider for now."""
    # start_time = context.get_time()
    home_state = np.zeros(15)
    ## state space definition
    # [0] = qw (relative to left leg)
    # [1] = qx (relative to left leg)
    # [2] = qy (relative to left leg)
    # [3] = qz (relative to left leg)
    # [4] = x (relative to left leg)
    # [5] = y (relative to left leg)
    # [6] = z (relative to left leg)
    # [7] = hip joint angle
    # [8] = q _dot (relative to left leg)
    # [9] = q _dot (relative to left leg)
    # [10] = q _dot (relative to left leg)
    # [11] = x_dot (relative to left leg)
    # [12] = y_dot (relative to left leg)
    # [13] = z_dot (relative to left leg)
    # [14] = hip joint angular velocity

    
    # Regular sim home_state values, 
    home_state[0:4] = RollPitchYaw(roll=0,pitch=0,yaw=0).ToQuaternion().wxyz()
    home_state[6] = 0.16 # starting height

    # # use this for checking the natural frequency. This drops the robot in at an angle
    # home_state[0:4] = RollPitchYaw(roll=np.pi/2,pitch=0,yaw=0).ToQuaternion().wxyz()
    # home_state[6] = 0.3
    # home_state[2] = -0.2
    
    return home_state * scale

'''---------------------------------------Set up foot contact--------------------------------------'''
class ContactResultsToArray(LeafSystem):
    def __init__(
            self,
            plant,
            scene_graph,
            collision_pairs = [
                [ScopedName("walker", "left_leg"), ScopedName("walker", "ground")],
                [ScopedName("walker", "right_leg"), ScopedName("walker", "ground")],
            ]
            # collision_pairs: list[list[ScopedName]] # Requires scoped names
            ):

        LeafSystem.__init__(self)
        self.geometryid2name={}
        scene_graph_context = scene_graph.CreateDefaultContext()
        query_object = scene_graph.get_query_output_port().Eval(scene_graph_context)
        inspector = query_object.inspector()
        for geometry_id in inspector.GetAllGeometryIds():
            body = plant.GetBodyFromFrameId(inspector.GetFrameId(geometry_id))
            if hasattr(body,'name'):
                # Scoped name adds the name of the object and the body
                scoped_name = body.scoped_name()
                self.geometryid2name[geometry_id.get_value()]=scoped_name.to_string()
            else:
                self.geometryid2name[geometry_id.get_value()]='NONAME'
        self.collision_pair_map = {}
        start_idx = 0
        for collision_pair in collision_pairs:
            name1 = collision_pair[0].to_string()
            name2 = collision_pair[1].to_string()
            if name1 not in self.collision_pair_map:
                self.collision_pair_map[name1] = {}
            if name2 not in self.collision_pair_map:
                self.collision_pair_map[name2] = {}
            idx_range = [start_idx,start_idx + 3]
            # collect both directions for efficiency later.
            self.collision_pair_map[name1][name2] = idx_range
            self.collision_pair_map[name2][name1] = idx_range
            start_idx += 3
        self.num_forces = start_idx
        self.force_output = np.zeros(self.num_forces)
        self.contact_points = np.zeros(self.num_forces)  # List to store contact points

        self.force_output_dict: dict[str, np.array] = dict()
        self.contact_points_dict: dict[str, np.array] = dict()

        self.DeclareAbstractInputPort(
            "contact_results", AbstractValue.Make(ContactResults())
        )
        self.DeclareVectorOutputPort(
            "contact_results_array", self.num_forces, self.Publish
        )        
        # Add periodic update event
        self.DeclarePeriodicDiscreteUpdateEvent(0.001, 0, self.Publish)

    def GetCollisionPairMap(self):
        return self.collision_pair_map
    
    def Publish(self, context, output):
        formatter = {"float": lambda x: "{:5.2f}".format(x)}
        results = self.get_input_port().Eval(context)

        # Reset forces and contact points for this loop
        self.force_output[:] = 0.0
        self.contact_points[:] = 0.0

        # Arrays for left and right foot forces and contact points
        left_foot_force = np.zeros(3)
        right_foot_force = np.zeros(3)
        left_foot_point = np.zeros(3)
        right_foot_point = np.zeros(3)

        # Loop over all hydroelastic contacts
        for i in range(results.num_hydroelastic_contacts()):
            info = results.hydroelastic_contact_info(i)
            cs = info.contact_surface()
            id1 = cs.id_M().get_value()
            id2 = cs.id_N().get_value()

            name1 = self.geometryid2name[id1]
            name2 = self.geometryid2name[id2]
            spatialforce = info.F_Ac_W()
            fxfyfz = spatialforce.translational()
            contact_point = cs.centroid()

            # Check if the contact is with the left foot
            if (ScopedName("walker", "left_leg").to_string() in name1 or 
                ScopedName("walker", "left_leg").to_string() in name2):
                left_foot_force += fxfyfz
                left_foot_point = contact_point  # Store contact point for the left foot

            # Check if the contact is with the right foot
            elif (ScopedName("walker", "right_leg").to_string() in name1 or 
                ScopedName("walker", "right_leg").to_string() in name2):
                right_foot_force += fxfyfz
                right_foot_point = contact_point  # Store contact point for the right foot

        # Store both the forces and contact points for the current timestep
        self.force_output_dict[str(context.get_time())] = {
            'left_foot_force': left_foot_force.copy(),
            'right_foot_force': right_foot_force.copy()
        }
        self.contact_points_dict[str(context.get_time())] = {
            'left_foot_point': left_foot_point.copy(),
            'right_foot_point': right_foot_point.copy()
        }


    def get_forces_and_points(self):
        return self.force_output_dict, self.contact_points_dict

'''---------------------------------------Set up controller--------------------------------------'''
def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2  
    return np.array([w, x, y, z])

def normalize_quat(q):
    norm = np.linalg.norm(q)
    # return q / norm if norm != 0 else q
    return q / norm 

def quat_inverse(q):
    # Assuming q is a numpy array [w, x, y, z]
    return np.array([q[0], -q[1], -q[2], -q[3]])

class Controller(LeafSystem):
    def __init__(
        self,
        scale,
        ground_friction,
        feet_friction,
        # target_state,
        control_period=0.0005,
        hip_kp=8, 
        hip_ki=0,
        hip_kd=0.1,
        threshold_force = 0.0001, # in N, To set contact mode,
        calib = False
        ):

        self.calib = calib
        # Assign the parameters to the instance variables
        self.hip_kp = hip_kp * 1000 #hip_kp = 1 for scale = 0.166 (Zippy scale), hip_kp = 8000 for scale 6.67 (Big Foot scale)
        self.hip_ki = hip_ki
        self.hip_kd = hip_kd
        self.scale = scale
        self.control_period=control_period
        self.integral_error = 0
        self.time = 0
        self.prev_error = 0
        # real frequency is calculted by f = (1/2pi) * (g/l)^(1/2) where l is *debatable, likely distance from ground to body COM height
        # based on 1.4Hz, Mugatu's COM should be 12.67 cm below the hip
        # based on the COM height reported in the paper, Mugatu's walking frequency should be 1.93 (assuming l is 6.6cm)
        self.frequency = 1.8
        self.frequency = self.frequency/np.sqrt(self.scale)

        # self.frequency = np.sqrt(9.81 / (0.097)) # 0.1267 is the height of Mugatu's COM in meters
        self.wait_time = 1 / (2 * self.frequency)  
        self.counter = 0
        # self.amplitude = 0.5
        self.amplitude = 35 * np.pi / 180
    

        """ For now, just a pd control tracking 1 state."""
        LeafSystem.__init__(self)
        self.controller_plant, self.scene_graph, self.controller_diagram, self.instance = setup_walker_controller_plant(
            scale = scale, 
            ground_friction = ground_friction,
            feet_friction = feet_friction,
            filename = urdf_file_name)

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
        self.threshold_force = threshold_force

        # Init gain matrix
        n_unactuated = self.controller_plant.num_positions() - self.controller_plant.num_actuators()
        """ quaternion difference is size 3 and quaternion is size 4"""
        self.gain_matrix = np.zeros((self.m, self.n - 1)) 
        self.gain_matrix[:,n_unactuated:self.n_pos] = np.eye(self.m)*self.hip_kp #gains for elements corresponding to positions
        self.gain_matrix[:,self.n_pos+n_unactuated:] = np.eye(self.m)*self.hip_kd #gains for elements corresponding to velocities

        #Init target state
        self.target_state = get_home_state(scale)
        # print("Init target state:", self.target_state)

        # Specify inputs and outputs
        self.state_input_index = self.DeclareVectorInputPort(
            "state", self.n
        ).get_index()
        
        self.DeclareVectorOutputPort(
            "control", self.m , self.SetOutput
        )

        # Add periodic update event
        self.DeclarePeriodicDiscreteUpdateEvent(self.control_period, 0, self.Update)
    
    @staticmethod
    def ComputeStateDifference(desired_state, current_state):
        """ Fill out this """
        difference_state = np.zeros(len(current_state)-1)

        # troubleshooting
        current_quat = current_state[0:4]
        desired_quat = desired_state[0:4]
        current_norm = np.linalg.norm(current_quat)
        desired_norm = np.linalg.norm(desired_quat)
        if current_norm == 0:
            print("Current quaternion is a zero vector.")
        if desired_norm == 0:
            print("Desired quaternion is a zero vector.")

        current_quaternion = Quaternion(current_quat/current_norm)
        desired_quaternion = Quaternion(desired_quat/desired_norm)
        difference_quaternion = desired_quaternion.multiply(current_quaternion.inverse())
        difference_angle_axis = AngleAxis(difference_quaternion)
        difference_rotation = difference_angle_axis.axis() * difference_angle_axis.angle()
        difference_state[0:3] = difference_rotation
        difference_state[3:] = desired_state[4:] - current_state[4:]
        return difference_state
    
    @staticmethod
    def ComputeControl(self, current_state, desired_state, gain_matrix, feedforward=None):
        error = desired_state[7] - current_state[7]
        self.integral_error += (error * self.control_period)
        self.derivative_error = (self.current_state[7] - self.prev_state[7]) / self.control_period
        feedback_input = self.hip_kp*error + self.hip_ki * self.integral_error + self.hip_kd * self.derivative_error
        # feedback_input = gain_matrix@(Controller.ComputeStateDifference(desired_state,current_state)) #change to quaternion difference (quaternion is IMU --> input for feedback controller)
        self.prev_error = error
        self.time += self.control_period
        self.prev_state = self.current_state
        # print("Elapsed time:", elapsed_time)
        # print("Control Signal:", feedback_input)
        if feedforward is None:
            return feedback_input
        else:
            return feedback_input + feedforward

    def LegStateMachine(self):
        pass

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
        
        self.mass_matrix = self.controller_plant.CalcMassMatrix(self.controller_plant_context)
        
        # self.mass_matrix = self.controller_plant.CalcMassMatrix(self.controller_plant_context)
        elapsed_time = context.get_time()
        act_start_time = 3
        adjusted_time = elapsed_time - act_start_time

        ang_freq = 2 * np.pi * self.frequency
        servo_input = self.amplitude * np.sin(ang_freq * adjusted_time)
        # print("servo_input", servo_input)
        if elapsed_time > act_start_time and not self.calib: # give time for sim to sit and settle
            self.target_state[7] = servo_input # comment this out when you're getting sim natural frequency
        else:
           pass
    
        # compute control
        # self.control_signal[:] = np.clip(self.ComputeControl(self,
        #     current_state=self.current_state,
        #     desired_state=self.target_state,
        #     gain_matrix=self.gain_matrix
        #     ), -20, 20)
        
        self.control_signal[:] = self.ComputeControl(self,
            current_state=self.current_state,
            desired_state=self.target_state,
            gain_matrix=self.gain_matrix
            )


    def SetOutput(self, context, output):
        output.SetFromVector(self.control_signal)