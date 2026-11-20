##### PYDRAKE IMPORTS
from pydrake.systems.framework import LeafSystem, Context, BasicVector
from pydrake.systems.sensors import CameraConfig, CameraInfo, ImageRgb8U, ImageDepth32F
from pydrake.common.value import AbstractValue
from pydrake.math import RigidTransform, RollPitchYaw

##### OTHER IMPORTS
import numpy as np
import cv2

##### SELF-DEFINED IMPORTS
import inputs.config as config

##### CAMERA SYSTEM DEFINITION (LeafSystem for camera)
class CameraSystem(LeafSystem):
    # shared variables

    # initialization
    def __init__(self, camera_info: CameraInfo, fps: float, X_BC: RigidTransform):
        print(f'\n=====CREATING RGBD CAMERA SYSTEM=====')
        super().__init__() # initializing the system C++ wrapper

        # assigning system variables
        self.camera_info = camera_info # information for the camera
        self.dt = 1.0 / fps # frame rate of the camera
        self.X_BC = X_BC # transform of camera frame relative to its base frame

        self.width = self.camera_info.width # width of camera (pixels)
        self.height = self.camera_info.height # height of camera (pixels)
        self.u0 = self.camera_info.center_x # center x pixel value
        self.v0 = self.camera_info.center_y # center y pixel value
        self.fx = self.camera_info.focal_x # focal length in x direction
        self.fy = self.camera_info.focal_y # focal length in y direction
        self.K = np.array( # intrinsic camera matrix
            [[self.fx, 0, self.u0, 0],
            [0, self.fy, self.v0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]
        )

        self.uT = np.nan # stored x pixel for target
        self.vT = np.nan # stored y pixel for target
        self.wT = np.nan # stored z pixel (depth) for target

        self.xTe = np.zeros(config.qT_num + config.vT_num) # stored target estimate target
        self.uTe = np.nan # stored estimate x pixel for target
        self.vTe = np.nan # stored estimate y pixel for target
        self.wTe = np.nan # stored estimate z pixel (depth) for target

        self.previous_image = None # previous stored camera color image for optical flow

        # declaring internal states
        self.frame_image_index = self.DeclareAbstractState(model_value=AbstractValue.Make(np.zeros((self.height, self.width, 4), dtype=np.uint8))) # modified image for cv2
        
        # declaring internal state updates
        self.DeclarePeriodicUnrestrictedUpdateEvent(period_sec=self.dt, offset_sec=0.0, update=self.UpdateFrame)

        # declaring ports
        self.target_state_port = self.DeclareVectorInputPort(name='target_state', size=config.qT_num + config.vT_num) # target state input port
        self.drone_state_port = self.DeclareVectorInputPort(name='drone_state', size=config.q_num + config.v_num) # drone state input port
        self.color_image_port = self.DeclareAbstractInputPort(name='color_image', model_value=AbstractValue.Make(ImageRgb8U)) # color image input port
        self.depth_image_port = self.DeclareAbstractInputPort(name='depth_image', model_value=AbstractValue.Make(ImageDepth32F)) # depth image input port

        self.reference_state_port = self.DeclareVectorOutputPort(name='reference_state', size=config.q_num + config.v_num, calc=self.UpdateReference) # calculated drone reference state output
        self.target_estimate_state_port = self.DeclareVectorOutputPort(name='target_estimate_state', size=config.qT_num + config.vT_num, calc=) # estimated target state output from camera image
        self.target_pixels_port = self.DeclareVectorOutputPort(name='target_pixels', size=2, calc=self.UpdateTargetPixels) # true target COM pixels in camera frame
        self.target_estimate_pixels_port = self.DeclareVectorOutputPort(name='target_estimate_pixels', size=2, calc=) # estimated target COM pixels in camera frame
        self.frame_image_port = self.DeclareStateOutputPort(name='frame_image', state_index=self.frame_image_index) # output port for modified cv2 image
        # self.frame_image_port = self.DeclareAbstractOutputPort(name='frame_image', 
        #                                alloc=lambda: AbstractValue.Make(np.zeros((camera_height, camera_width, 4), dtype=np.uint8)),
        #                                calc=self.ExportFrame
        # ) # output port for modified cv2 image

    # update altered frame and references function
    def UpdateFrame(self, context: Context, abstract_state):
        # read current states from context
        color_image = self.color_image_port.Eval(context).data # reading current color image data

        if self.previous_image is not None:
            # do optical flow with cv2 (IMPLEMENT LATER)
            pass

        # do cv2 bounding box stuff to figure out what estimate target state is

        # apply true target position to frame

        self.previous_image = np.copy(color_image) # store latest image for optical flow

    # update drone reference output function
    def UpdateReference(self, context: Context, output: BasicVector):
        # read current states from context
        x = self.drone_state_port.Eval(context) # current drone state

        xT = self.target_state_port.Eval(context) # DELETE THIS LATER

    # update target estimate based on bounding box pixels
    def UpdateEstimate(self, context: Context, output: BasicVector):
        # read current states from context
        pass

    # update true target pixels in camera frame
    def UpdateTargetPixels(self, context: Context, output: BasicVector):
        # read current states from context
        x = self.drone_state_port.Eval(context) # current drone state
        xT = self.target_state_port.Eval(context) # current target state

        # calculating transformations
        X_WD = RigidTransform(rpy=RollPitchYaw(x[0], x[1], x[2]), p=x[3:6]) # transform of drone with respect to world
        X_WC = X_WD * self.X_BC # transform of camera with respect to world
        X_CW = X_WC.inverse() # transform of world with respect to camera

        # applying transformations
        pT = self.K @ X_CW.GetAsMatrix4() @ np.append(xT[3:6], 1) # pixels in camera frame

        self.wT = pT[2] # calculate target depth
        self.uT = pT[0] / self.wT # calculate x pixel
        self.vT = pT[1] / self.wT # calculate y pixel

        if self.uT > self.width or self.uT < 0 or self.vT > self.height or self.vT < 0: # if target is out of frame
            self.uT = np.nan # update to nan
            self.vT = np.nan # update to nan
        
        output.SetFromVector(value=np.array([self.uT, self.vT])) # return calculated true pixels







    # state.get_mutable_abstract_state(int(self._traj_index))
    # and .get_value()

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