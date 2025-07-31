from pydrake.all import *   


def start_meshcat():
    meshcat = StartMeshcat()
    print("_"*120)
    print(f" Meshcat is live. Click link: {meshcat.web_url()}")
    print("_"*120)
    return meshcat

def setup_walker_plant(timestep, filename=None):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=timestep)
    parser = Parser(plant)
    # parser.AddModelsFromString(create_walker_urdf(scale = scale, ground_friction = ground_friction, feet_friction = feet_friction),"urdf")
    parser.AddModels(filename)
    # plant.set_discrete_contact_approximation(DiscreteContactApproximation.kSap)
    plant.Finalize()
    instance = plant.GetModelInstanceByName("walker")
    return plant, scene_graph, builder, instance

def setup_walker_controller_plant(timestep=0.001, filename=None):
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

def get_home_state(start_height=0.16, start_rpy=(0, 0, 0)):
    home_state = np.zeros(15)
    ## state space definition
    # [0-3] = qw, qi, qj, qk (relative to left leg)
    # [4-6] = x, y, z (relative to left leg)
    # [7] = hip joint angle
    # [8-10] = wx, wy ,wz (relative to left leg)
    # [11-13] = x_dot, y_dot, z_dot (relative to left leg)
    # [14] = hip joint angular velocity

    # Regular sim home_state values, 
    home_state[0:4] = RollPitchYaw(roll=start_rpy[0],
                                   pitch=start_rpy[1],
                                   yaw=start_rpy[2]).ToQuaternion().wxyz()
    home_state[6] = start_height
    return home_state


class ContactResultsToArray(LeafSystem):
    def __init__(
            self,
            plant,
            scene_graph,
            collision_pairs = [
                [ScopedName("walker", "left_foot"), ScopedName("walker", "ground")],
                [ScopedName("walker", "right_foot"), ScopedName("walker", "ground")],
            ]
            ):

        LeafSystem.__init__(self)
        self.geometryid2name={}
        scene_graph_context = scene_graph.CreateDefaultContext()
        query_object = scene_graph.get_query_output_port().Eval(scene_graph_context)
        inspector = query_object.inspector()
        for geometry_id in inspector.GetAllGeometryIds():
            body = plant.GetBodyFromFrameId(inspector.GetFrameId(geometry_id))
            if hasattr(body,'name'):
                scoped_name = body.scoped_name()
                self.geometryid2name[geometry_id.get_value()]=scoped_name.to_string()
            else:
                self.geometryid2name[geometry_id.get_value()]='NONAME'
        
        # Store collision_pairs as a member variable to be used in CalcCombinedContactData
        self.collision_pairs = collision_pairs 

        # Define the size of the combined output vector:
        # 3 (left_force) + 3 (right_force) + 3 (left_point) + 3 (right_point) = 12
        self.output_vector_size = 12 

        self.DeclareAbstractInputPort(
            "contact_results", AbstractValue.Make(ContactResults())
        )
        # Declare a single output port for combined forces and points.
        # This output port will be calculated by the `CalcCombinedContactData` method.
        self.DeclareVectorOutputPort(
            "combined_contact_data", self.output_vector_size, self.CalcCombinedContactData
        )        
        # REMOVED: DeclarePeriodicDiscreteUpdateEvent. The output port will be evaluated when its data is requested (e.g., by a logger).

    # Renamed from Publish to CalcCombinedContactData, and it now directly sets the output vector
    def CalcCombinedContactData(self, context, output_vector):
        results = self.get_input_port().Eval(context)

        # Initialize a temporary array for this calculation
        combined_output_temp = np.zeros(self.output_vector_size)

        # Arrays for left and right foot forces and contact points
        left_foot_force = np.zeros(3)
        right_foot_force = np.zeros(3)
        left_foot_point = np.zeros(3)
        right_foot_point = np.zeros(3)

        # Loop over all hydroelastic contacts detected by Drake
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

            # Now, iterate through the configured collision_pairs to check for a match
            for pair in self.collision_pairs:
                pair_name1 = pair[0].to_string()
                pair_name2 = pair[1].to_string()

                # Check if the current contact matches one of the specified pairs (order insensitive)
                if (pair_name1 in name1 and pair_name2 in name2) or \
                   (pair_name1 in name2 and pair_name2 in name1):
                    
                    # If it's the left foot's pair
                    if "left_foot" in pair_name1 or "left_foot" in pair_name2:
                        left_foot_force += fxfyfz
                        left_foot_point = contact_point
                        break 
                    
                    # If it's the right foot's pair
                    elif "right_foot" in pair_name1 or "right_foot" in pair_name2:
                        right_foot_force += fxfyfz
                        right_foot_point = contact_point
                        break 
        
        # Populate the combined_output_temp vector
        # Order: left_force_xyz (0:3), right_force_xyz (3:6), left_point_xyz (6:9), right_point_xyz (9:12)
        combined_output_temp[0:3] = left_foot_force
        combined_output_temp[3:6] = right_foot_force
        combined_output_temp[6:9] = left_foot_point
        combined_output_temp[9:12] = right_foot_point

        # Set the output port value directly
        output_vector.SetFromVector(combined_output_temp)


