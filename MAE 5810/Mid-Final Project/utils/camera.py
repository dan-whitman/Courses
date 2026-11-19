##### PYDRAKE IMPORTS
from pydrake.systems.framework import LeafSystem, Context
from pydrake.systems.sensors import CameraConfig, ImageRgb8U, ImageDepth32F
from pydrake.common.value import AbstractValue

##### OTHER IMPORTS
import numpy as np
import cv2

##### SELF-DEFINED IMPORTS
import inputs.config as config

##### CAMERA SYSTEM DEFINITION (LeafSystem for camera)
class CameraSystem(LeafSystem):
    # shared variables

    # initialization
    def __init__(self, camera_width: float, camera_height: float, fps: float):
        print(f'\n=====CREATING RGBD CAMERA SYSTEM=====')
        super().__init__() # initializing the system C++ wrapper

        # assigning system variables
        self.camera_width = camera_width # width of camera (pixels)
        self.camera_height = camera_height # height of camera (pixels)
        self.dt = 1.0 / fps # fps interval for camera updates

        # self.x0 = None # stored drone reference state
        self.xT = np.zeros(config.qT_num + config.vT_num) # stored target state

        # declaring internal states
        
        # declaring internal state updates

        # declaring ports
        self.DeclareVectorInputPort(name='target_state', size=config.qT_num + config.vT_num) # target state input port
        self.DeclareAbstractInputPort(name='color_image', model_value=AbstractValue.Make(ImageRgb8U)) # color image input port
        self.DeclareAbstractInputPort(name='depth_image', model_value=AbstractValue.Make(ImageDepth32F)) # depth image input port

        self.DeclareVectorOutputPort(name='reference_state', size=config.q_num + config.v_num) # calculated drone reference state output
        self.DeclareVectorOutputPort(name='target_estimate_state', size=config.qT_num + config.vT_num) # estimated target state output from camera image
        self.DeclareVectorOutputPort(name='target_pixels', size=2) # true target COM pixels in camera frame
        self.DeclareVectorOutputPort(name='target_estimate_pixels', size=2) # estimated target COM pixels in camera frame
        # self.DeclareAbstractOutputPort(name='frame_image', 
        #                                alloc=lambda: AbstractValue.Make(np.zeros((camera_height, camera_width, 4), dtype=np.uint8)),
        #                                calc=self.ExportFrame
        # )
        # DECLARE STATE OUTPUT PORT INSTEAD?

    # export altered frame function
    def ExportFrame(self, context: Context, output: np.ndarray):
        pass

    # inputs raw color image, raw depth image, and uses openCV to find bounding box for target centroid
        # inputs as AbstractInputPorts
    # then computes estimated target position in world and uses that to determine drone reference (VectorOutputPort)
        # need to define config drone reference offset from target
        # for testing ignore all image stuff and just directly pass in target state (VectorInputPort) and export desired drone state, see if works
    # add output port for estimated target state (target_estimate_state) can be VectorOutputPort
    # add output for frame image (the one with the bounding box and placed target stuff from CameraMeasure)? (AbstractOutputPort)
    # add output ports for both true pixel camera measurements (u,v) and estimated from bounding box (VectorOutputPorts)
    # move camera measure into class function defintion
    # stores last measured target state until new measurment (so drone only follows last known to update reference_state)

##### CAMERA MODEL FUNCTION (inputs camera config, camera transform, and target state and returns measured pixel output)
def CameraMeasure(config: CameraConfig, X_CW: np.array, x_T: np.array) -> tuple:
    # extracting camera config parameters
    fx = config.focal_x()
    fy = config.focal_y()
    u0 = config.principal_point()[0]
    v0 = config.principal_point()[1]

    # creating projection transformation matrix
    K = np.array([[fx, 0, u0, 0],
                  [0, fy, v0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    # transferring to homoegenous coordinates
    if len(x_T) != 4: x_T = np.append(x_T, 1)

    T_CW = X_CW.GetAsMatrix4()

    # performing projection
    p_T = K @ T_CW @ x_T

    # extracting pixels
    w = p_T[2] # finding depth normalization
    u = p_T[0] / w # finding x pixel coordinate
    v = p_T[1] / w # finding y pixel coordinate

    if (u < config.width and u > 0) and (v < config.height and v > 0): # if target is in frame
        return u, v
    else:
        return None, None # return empty set