##### PYDRAKE IMPORTS
from pydrake.systems.framework import DiagramBuilder, Diagram, Context, LeafSystem, BasicVector
from pydrake.systems.controllers import LinearQuadraticRegulator
from pydrake.systems.primitives import AffineSystem, FirstOrderTaylorApproximation
from pydrake.multibody.plant import ExternallyAppliedSpatialForce
from pydrake.multibody.math import SpatialForce
from pydrake.common.value import Value

##### OTHER IMPORTS
import numpy as np

##### SELF_DEFINED IMPORTS
import inputs.config as config

##### PD SYSTEM DEFINITION (LeafSystem for PD controller)
class ControlPDSystem(LeafSystem):
    # sared variables

    # initialization
    def __init__(self, Kp: float, Kd: float):
        print(f'\n=====CREATING PD SYSTEM=====')
        super().__init__() # initializing the system C++ wrapper

        # assigning system variables
        self.Kp = np.zeros((6, config.qT_num))
        self.Kd = np.zeros((6, config.vT_num))
        np.fill_diagonal(self.Kp, Kp * np.ones(config.qT_num))
        np.fill_diagonal(self.Kd, Kd * np.ones(config.vT_num))

        # declaring internal states

        # declaring ports
        self.target_state_port = self.DeclareVectorInputPort(name='target_state', size=config.qT_num + config.vT_num) # target state input port
        self.target_trajec_port = self.DeclareVectorInputPort(name='target_trajec', size=config.qT_num + config.vT_num) # target trajectory port

        self.target_force_port = self.DeclareAbstractOutputPort(
            name='target_force', alloc=lambda: Value[list[ExternallyAppliedSpatialForce]](),
            calc=self.UpdateForce
        )

    # update force function
    def UpdateForce(self, context: Context, output):
        # read current states from context
        xT = self.target_state_port.Eval(context)
        xT_traj = self.target_trajec_port.Eval(context)

        u = - self.Kp @ (xT[:config.qT_num] - xT_traj[:config.qT_num]) - self.Kd @ (xT[config.qT_num:] - xT_traj[config.qT_num:]) # calculate new forces

        u_out = ExternallyAppliedSpatialForce() # create force
        u_out.body_index = config.target_index # apply to target base index
        u_out.p_BoBq_B = np.zeros(3) # apply force at COM
        u_out.F_Bq_W = SpatialForce(F=u)

        output.set_value([u_out]) # set new forces

##### LQR SYSTEM DEFINITION (LeafSystem for LQR controller)
class ControlLQRSystem(LeafSystem):
    # shared variables

    # initialization
    def __init__(self, affine_system: AffineSystem, Q: np.ndarray, R: np.ndarray, N: np.ndarray, u0: np.ndarray):
        print(f'\n=====CREATING LQR SYSTEM=====')
        super().__init__() # initializing the system C++ wrapper

        # assigning system variables
        self.affine_system = affine_system # assign hexrotor diagram

        self.Q = Q # assign Q matrix
        self.R = R # assign R matrix
        self.N = N # assign N matrix

        self.x0 = None # stored reference state
        self.u0 = u0 # stored reference thrusts

        # determining LQR gain
        self.K, _ = LinearQuadraticRegulator(A=affine_system.A(), B=affine_system.B(), Q=self.Q, R=self.R, N=self.N)

        # declaring internal states

        # declaring ports
        self.drone_state_port = self.DeclareVectorInputPort(name='drone_state', size=config.q_num + config.v_num) # drone state input
        self.reference_state_port = self.DeclareVectorInputPort(name='reference_state', size = config.q_num + config.v_num) # desired drone state reference input

        self.propeller_thrusts_port = self.DeclareVectorOutputPort(name='propeller_thrusts', size=config.u_num, calc=self.UpdateThrusts)

    # update thrusts function
    def UpdateThrusts(self, context: Context, output: BasicVector):
        # read current states from context
        x = self.drone_state_port.Eval(context)
        x0 = self.reference_state_port.Eval(context)

        u = self.u0 - self.K @ (x - x0) # calculate new thrusts
        u = np.clip(u, a_min=config.prop_min_thrust, a_max=config.prop_max_thrust) # saturate thrusts

        output.SetFromVector(value=u) # set new thrusts

##### LQR DIAGRAM FUNCTION (creates LQR diagram block for given system)
def ControlLQRDiagram(open_diagram: Diagram, initial_context: Context) -> Diagram:
    print(f'\n=====CREATING LQR DIAGRAM=====')

    # initialing the builder 
    lqr_builder = DiagramBuilder()

    # creating LQR gain matrices
    Q = np.ones(config.q_num + config.v_num) # Q matrix gain vector
    Q[:3] = config.lqr_rpy_gains # orientation gains
    Q[3:6] = config.lqr_pos_gains # position gains
    Q[config.q_num:config.q_num + 3] = config.lqr_rot_vel_gains # rotation velocity gains
    Q[config.q_num + 3:] = config.lqr_pos_vel_gains # linear velocity gains

    R = np.full(config.u_num, config.lqr_thrust_gains) # R matrix gain vector

    N = np.zeros((config.q_num + config.v_num, config.u_num)) # N matrix gain

    Q = np.diag(Q) # diagonalize Q vector
    R = np.diag(R) # diagonalize R vector

    # linearizing the system
    affine_sytsem = FirstOrderTaylorApproximation(system=open_diagram, context=initial_context)

    # adding in LQR controller
    lqr = lqr_builder.AddNamedSystem(name='lqr', system=ControlLQRSystem(affine_system=affine_sytsem, Q=Q, R=R, N=N, u0=config.u0))

    # exposing ports
    lqr_builder.ExportInput(input=lqr.GetInputPort(port_name='drone_state'), name='drone_state')
    lqr_builder.ExportInput(input=lqr.GetInputPort(port_name='reference_state'), name='reference_state')

    lqr_builder.ExportOutput(output=lqr.GetOutputPort(port_name='propeller_thrusts'), name='propeller_thrusts')

    # complete diagram
    return lqr_builder.Build()

##### CONTROL DIAGRAM FUNCTION (creates closed-loop diagram from open-loop, control, and camera diagram)
def ControlDiagram(open_diagram: Diagram, control_diagram: Diagram, camera_diagram: Diagram) -> Diagram:
    print(f'\n=====CREATING CONTROL DIAGRAM=====')

    # initiating the builder
    closed_builder = DiagramBuilder()

    # adding subsystems
    closed_builder.AddNamedSystem(name='open_diagram', system=open_diagram) # open-loop system
    closed_builder.AddNamedSystem(name='control_diagram', system=control_diagram) # control diagram
    closed_builder.AddNamedSystem(name='camera_diagram', system=camera_diagram) # camera diagram

    # wiring the systems together
    closed_builder.Connect(open_diagram.GetOutputPort(port_name='drone_state'), control_diagram.GetInputPort(port_name='drone_state'))
    closed_builder.Connect(open_diagram.GetOutputPort(port_name='drone_state'), camera_diagram.GetInputPort(port_name='drone_state'))
    closed_builder.Connect(camera_diagram.GetOutputPort(port_name='reference_state'), control_diagram.GetInputPort(port_name='reference_state'))
    closed_builder.Connect(control_diagram.GetOutputPort(port_name='propeller_thrusts'), open_diagram.GetInputPort(port_name='propeller_thrusts'))
    closed_builder.Connect(open_diagram.GetOutputPort(port_name='color_image'), camera_diagram.GetInputPort(port_name='color_image'))
    closed_builder.Connect(open_diagram.GetOutputPort(port_name='depth_image'), camera_diagram.GetInputPort(port_name='depth_image'))
    closed_builder.Connect(open_diagram.GetOutputPort(port_name='target_state'), camera_diagram.GetInputPort(port_name='target_state'))

    # exposing ports
    closed_builder.ExportOutput(output=open_diagram.GetOutputPort(port_name='drone_state'), name='drone_state') # for printing/tracking drone state
    closed_builder.ExportOutput(output=open_diagram.GetOutputPort(port_name='target_state'), name='target_state') # for printing tracking target state
    closed_builder.ExportOutput(output=camera_diagram.GetOutputPort(port_name='reference_state'), name='reference_state') # for printing reference state
    closed_builder.ExportOutput(output=control_diagram.GetOutputPort(port_name='propeller_thrusts'), name='propeller_thrusts') # for printing/tracking propeller thrusts
    closed_builder.ExportOutput(output=camera_diagram.GetOutputPort(port_name='target_estimate_state'), name='target_estimate_state') # drone estimate of target state
    closed_builder.ExportOutput(output=camera_diagram.GetOutputPort(port_name='target_pixels'), name='target_pixels') # true camera pixels for target base
    closed_builder.ExportOutput(output=camera_diagram.GetOutputPort(port_name='target_estimate_pixels'), name='target_estimate_pixels') # estimated camera pixels for target base  
    closed_builder.ExportOutput(output=camera_diagram.GetOutputPort(port_name='frame_image'), name='frame_image') # for creating cv2 video
    closed_builder.ExportOutput(output=open_diagram.GetOutputPort(port_name='geometry_query'), name='geometry_query') # for visualization

    # complete diagram
    return closed_builder.Build() 

##### CONTROLLER CONTEXT FUNCTION (creates closed-loop context from diagram)
def ControlContext(closed_diagram: Diagram, initial_context: Context) -> Context:
    print(f'\n=====CREATING CONTROLLER CONTEXT=====')

    # creating contexts
    closed_context = closed_diagram.CreateDefaultContext() # creating numerical context
    open_diagram = closed_diagram.GetSubsystemByName(name='open_diagram') # obtaining open-loop system
    control_diagram = closed_diagram.GetSubsystemByName(name='control_diagram') # obtaining control diagram
    camera_diagram = closed_diagram.GetSubsystemByName(name='camera_diagram') # obtaining camera diagram

    closed_open_context = open_diagram.GetMyMutableContextFromRoot(root_context=closed_context)
    closed_control_context = control_diagram.GetMyMutableContextFromRoot(root_context=closed_context)
    closed_camera_context = camera_diagram.GetMyMutableContextFromRoot(root_context=closed_context)

    closed_open_context.SetStateAndParametersFrom(source=initial_context)

    # return context
    return closed_context