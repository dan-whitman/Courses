##### PYDRAKE IMPORTS
from pydrake.systems.framework import DiagramBuilder, Diagram, Context
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant, CoulombFriction, BaseBodyJointType, PropellerInfo, Propeller
from pydrake.multibody.parsing import Parser
from pydrake.math import RigidTransform
from pydrake.geometry import SceneGraph, ProximityProperties, AddContactMaterial, HalfSpace
from pydrake.systems.sensors import CameraConfig, ApplyCameraConfig, CameraInfo

##### OTHER IMPORTS
import numpy as np

##### SELF_DEFINED IMPORTS
import inputs.config as config
from utils.visual import VisualPlantSceneFrames
from utils.xacro import XacroToURDF

##### DRONE PLANT FUNCTION (creates hexrotor plant/scene graph objects)
def DronePlantScene(builder: DiagramBuilder, time_step: float) -> tuple[MultibodyPlant, SceneGraph]:
    print(f'\n=====CREATING DRONE PLANT=====')

    # creating the plant and scene graph
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder=builder, time_step=time_step)

    parser = Parser(plant=plant) # initialize parser
    # inspector = scene_graph.model_inspector() # initialize inspector

    # extracting constants
    config.g_mag = np.linalg.norm(plant.gravity_field().gravity_vector())

    # adding floor to world
    X_WG = RigidTransform.Identity() # identity transform

    proximity_properties = ProximityProperties()
    surface_friction = CoulombFriction(static_friction=config.static_friction, dynamic_friction=config.dynamic_friction) # floor friction
    AddContactMaterial(friction=surface_friction, properties=proximity_properties)

    plant.RegisterCollisionGeometry(
        body=plant.world_body(), X_BG=X_WG, shape=HalfSpace(), name='ground_collision', properties=proximity_properties
    )

    # adding models to plant
    Drone = XacroToURDF(str(config.MOD_DIR / 'FPVDrone.urdf.xacro'))
    Target = XacroToURDF(str(config.MOD_DIR / 'Target.urdf.xacro'))

    config.drone_instance = parser.AddModelsFromString(file_contents=Drone, file_type='urdf')[0]
    config.target_instance = parser.AddModelsFromString(file_contents=Target, file_type='urdf')[0]

    # configuring dynamics to rpy representation
    plant.SetBaseBodyJointType(joint_type=BaseBodyJointType.kRpyFloatingJoint)



    # adding visualization frames
    if config.args.frames: # if adding visual frames
        plant, scene_graph = VisualPlantSceneFrames(plant=plant, scene_graph=scene_graph)

    plant.Finalize() # finalize the plant

    return plant, scene_graph # return completed plant and scene graph

##### DRONE PROPELLER FUNCTION (returns propeller info for builder)
def DronePropellers(plant: MultibodyPlant) -> Propeller:
    print(f'\n=====CREATING DRONE PROPELLERS=====')

    # finding prop bodies and frames
    drone_prop_names = ['drone_prop_1', 'drone_prop_2', 'drone_prop_3', 'drone_prop_4'] # link names
    drone_prop_bodies = [plant.GetBodyByName(name=drone_prop_name) for drone_prop_name in drone_prop_names] # model bodies

    # adding the propellers
    drone_prop_info = []
    for i, rotor_prop_body in enumerate(drone_prop_bodies):
        thrust_ratio = config.prop_thrust_ratio # thrust ratio for propeller
        moment_ratio = config.prop_moment_ratio * (-1)**i # moment ratio for propeller (positive and negative to signify direction of spin)

        X_RP = RigidTransform.Identity() # identity transform

        # add to propeller list
        drone_prop_info.append(PropellerInfo(body_index=rotor_prop_body.index(), X_BP=X_RP, thrust_ratio=thrust_ratio, moment_ratio=moment_ratio)) # propeller for split

    # return completed propeller list
    return Propeller(drone_prop_info)

##### DRONE CAMERA FUNCTION (creates camera in plant)
def DroneCamera(builder: DiagramBuilder, plant: MultibodyPlant, scene_graph: SceneGraph) -> None:
    print(f'\n=====CREATING DRONE CAMERA=====')

    # finding camera body
    camera_body = plant.GetBodyByName(name='drone_camera')

    # creating camera configuration
    camera_config = CameraConfig()
    camera_config.name = 'drone_camera'
    camera_config.width = config.camera_width
    camera_config.height = config.camera_height
    camera_config.fps = config.camera_fps
    camera_config.focal = CameraConfig.FocalLength(x=config.camera_fx, y=config.camera_fy)
    camera_config.X_PB.base_frame = 'drone::drone_camera'
    camera_config.rgb = True # capture rgb
    camera_config.depth = True # capture distance (for target state extraction)
    camera_config.label = False

    # save camera information globally
    config.camera_info = CameraInfo(
        wisth=camera_config.width, height=camera_config.height, focal_x=camera_config.focal_x(), focal_y=camera_config.focal_y(), center_x=camera_config.principal_point()[0], center_y=camera_config.principal_point()[1]
    )

    # apply camera to builder
    ApplyCameraConfig(config=camera_config, builder=builder, scene_graph=scene_graph)

##### Drone DIAGRAM FUNCTION (creates open-loop hexrotor diagram)
def DroneDiagram() -> Diagram:
    print(f'\n=====CREATING DRONE DIAGRAM=====')

    # initiating the builder
    open_builder = DiagramBuilder()

    # creating plant and scene graph
    plant, scene_graph = DronePlantScene(builder=open_builder, time_step=0.0)

    # creating the propellers
    propellers = open_builder.AddNamedSystem(name='propellers', system=DronePropellers(plant=plant))

    # creating the camera
    DroneCamera(builder=open_builder, plant=plant, scene_graph=scene_graph)

    # extracting constants
    config.q_num = plant.num_positions(model_instance=config.drone_instance) # number of state coordinates
    config.v_num = plant.num_velocities(model_instance=config.drone_instance) # number of state velocities
    config.u_num = propellers.get_command_input_port().size() # number of control inputs

    # wiring the diagram
    open_builder.Connect(plant.get_body_poses_output_port(), propellers.get_body_poses_input_port())
    open_builder.Connect(propellers.get_spatial_forces_output_port(), plant.get_applied_spatial_force_input_port())

    # exposing ports
    open_builder.ExportInput(input=propellers.get_command_input_port(), name='propeller_thrusts') # propeller inputs
    
    open_builder.ExportOutput(output=plant.get_state_output_port(model_instance=config.drone_instance), name='drone_state') # drone output
    open_builder.ExportOutput(output=plant.get_state_output_port(model_instance=config.target_instance), name='target_state') # target output
    open_builder.ExportOutput(output=scene_graph.get_query_output_port(), name='geometry_query') # geometry output

    # complete diagram
    return open_builder.Build()

##### DRONE INITIAL CONTEXT FUNCTION (creates initial context for diagram)
def DroneInitialContext(open_diagram: Diagram) -> Context:
    print(f'\n=====CREATING DRONE INITIAL CONTEXT=====')

    # creating contexts
    initial_context = open_diagram.CreateDefaultContext() # creating numerical context
    plant = open_diagram.GetSubsystemByName(name='plant') # obtaining plant
    plant_context = open_diagram.GetMutableSubsystemContext(subsystem=plant, context=initial_context) # getting plant context

    # placing drone and finding parameters
    config.drone_mass = plant.CalcTotalMass(context=plant_context, model_instances=[config.drone_instance]) # finding drone mass

    drone_body = plant.GetBodyByName(name='drone_base') # hexrotor base body
    X_WR = RigidTransform(rpy=config.drone_initial_rpy, p=config.drone_initial_position) # target transform
    plant.SetFreeBodyPose(context=plant_context, body=drone_body, X_JpJc=X_WR) # placing hexrotor

    # placing the target
    target_body = plant.GetBodyByName(name='target_base')
    X_WT = RigidTransform(rpy=config.target_initial_rpy, p=config.target_initial_position) # target transform
    plant.SetFreeBodyPose(context=plant_context, body=target_body, X_JpJc=X_WT) # placing target

    # defining initial propeller thrusts
    prop_thrust = 1.00 * config.g_mag * config.drone_mass / config.u_num # splitting thurst over all props
    u_initial = np.full(config.u_num, prop_thrust) # setting control

    open_diagram.GetInputPort(port_name='propeller_thrusts').FixValue(context=initial_context, value=u_initial) # drone thrusts

    # return context
    return initial_context

##### DRONE STATE EVALUATE FUNCTION (evalues hexrotor states and thrusts for given context)
def DroneEval(diagram: Diagram, context: Context) -> tuple[np.ndarray, np.ndarray]:
    print(f'\n=====EVALUATING HEXROTOR STATE=====')

    # evaulating the state
    x = diagram.GetOutputPort(port_name='drone_state').Eval(context)

    # evaulating the thrusts
    if diagram.HasInputPort(port_name='propeller_thrusts'):
        u = diagram.GetInputPort(port_name='propeller_thrusts').Eval(context)

    elif diagram.HasOutputPort(port_name='propeller_thrusts'):
        u = diagram.GetOutputPort(port_name='propeller_thrusts').Eval(context)

    # returning values
    return x, u