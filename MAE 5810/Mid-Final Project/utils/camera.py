##### PYDRAKE IMPORTS
from pydrake.systems.framework import LeafSystem, Context, Diagram, DiagramBuilder, State
from pydrake.systems.sensors import CameraInfo, ImageRgba8U, ImageDepth32F
from pydrake.common.value import AbstractValue
from pydrake.math import RigidTransform, RollPitchYaw

##### OTHER IMPORTS
import numpy as np
import cv2

##### SELF-DEFINED IMPORTS
import inputs.config as config
from utils.video import VideoWriteCircle, VideoHSV, VideoWriteRect

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

        self.width = self.camera_info.width() # width of camera (pixels)
        self.height = self.camera_info.height() # height of camera (pixels)
        self.u0 = self.camera_info.center_x() # center x pixel value
        self.v0 = self.camera_info.center_y() # center y pixel value
        self.fx = self.camera_info.focal_x() # focal length in x direction
        self.fy = self.camera_info.focal_y() # focal length in y direction
        self.K = np.array( # intrinsic camera matrix
            [[self.fx, 0, self.u0, 0],
            [0, self.fy, self.v0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]
        )
        self.Kinv = np.linalg.inv(self.K) # intrinsic inverse camera matrix

        self.previous_xTe = None # previous stored target state estimate for velocity approximation
        self.previous_x0 = np.zeros(config.q_num + config.v_num) # previous stored reference state for tracking
        self.previous_image = None # previous stored camera color image for optical flow

        # declaring internal states     
        self.reference_state_index = self.DeclareDiscreteState(num_state_variables=config.q_num + config.v_num) # stored drone reference state
        self.target_estimate_state_index = self.DeclareDiscreteState(num_state_variables=config.qT_num + config.vT_num) # stored target estimate state
        self.target_pixels_index = self.DeclareDiscreteState(num_state_variables=3) # stored true target pixels
        self.target_estimate_pixels_index = self.DeclareDiscreteState(num_state_variables=3) # stored estimated target pixels
        self.frame_image_index = self.DeclareAbstractState(model_value=AbstractValue.Make(np.zeros((self.height, self.width, 4), dtype=np.uint8))) # modified image for cv2
        
        # declaring internal state updates
        self.DeclarePeriodicUnrestrictedUpdateEvent(period_sec=self.dt, offset_sec=0.0, update=self.UpdateFrame)

        # declaring ports
        self.drone_state_port = self.DeclareVectorInputPort(name='drone_state', size=config.q_num + config.v_num) # drone state input port
        self.target_state_port = self.DeclareVectorInputPort(name='target_state', size=config.qT_num + config.vT_num) # target state input port
        self.color_image_port = self.DeclareAbstractInputPort(name='color_image', model_value=AbstractValue.Make(ImageRgba8U(width=self.width, height=self.height))) # color image input port
        self.depth_image_port = self.DeclareAbstractInputPort(name='depth_image', model_value=AbstractValue.Make(ImageDepth32F(width=self.width, height=self.height))) # depth image input port
        
        self.reference_state_port = self.DeclareStateOutputPort(name='reference_state', state_index=self.reference_state_index) # estimated target state output from camera
        self.target_estimate_state_port = self.DeclareStateOutputPort(name='target_estimate_state', state_index=self.target_estimate_state_index) # estimated target state output from camera
        self.target_pixels_port = self.DeclareStateOutputPort(name='target_pixels', state_index=self.target_pixels_index) # true target pixels in camera frame
        self.target_estimate_pixels_port = self.DeclareStateOutputPort(name='target_estimate_pixels', state_index=self.target_estimate_pixels_index) # estimated target pixels in camera frame
        self.frame_image_port = self.DeclareStateOutputPort(name='frame_image', state_index=self.frame_image_index) # output port for modified cv2 image
        # self.frame_image_port = self.DeclareAbstractOutputPort(name='frame_image', 
        #                                alloc=lambda: AbstractValue.Make(np.zeros((camera_height, camera_width, 4), dtype=np.uint8)),
        #                                calc=self.ExportFrame
        # ) # output port for modified cv2 image

    # update altered frame and references function
    def UpdateFrame(self, context: Context, state: State):
        # read current states from context
        x = self.drone_state_port.Eval(context) # read current drone state
        xT = self.target_state_port.Eval(context) # read current target state

        x0 = state.get_mutable_discrete_state(index=self.reference_state_index).get_mutable_value() # get current mutable stored drone reference
        xTe = state.get_mutable_discrete_state(index=self.target_estimate_state_index).get_mutable_value() # get current mutable stored target state estimate
        pT = state.get_mutable_discrete_state(index=self.target_pixels_index).get_mutable_value() # get current mutable stored target pixels
        pTe = state.get_mutable_discrete_state(index=self.target_estimate_pixels_index).get_mutable_value() # get current mutable stored target pixels estimate

        color_image = np.copy(self.color_image_port.Eval(context).data) # reading current color image data
        depth_image = self.depth_image_port.Eval(context) # reading current depth image

        # calculating transformations
        X_WD = RigidTransform(rpy=RollPitchYaw(x[0], x[1], x[2]), p=x[3:6]) # transform of drone with respect to world
        X_WC = X_WD.multiply(other=self.X_BC) # transform for camera with respect to world
        X_CW = X_WC.inverse() # transform of world with respect to camera

        # updating true target pixels
        xT[5] = xT[5] + config.target_height / 2 # set height at center of target body
                
        pT_vec = self.K @ X_CW.GetAsMatrix4() @ np.append(xT[3:6], 1) # pixels in camera frame

        pT[0] = pT_vec[0] / pT_vec[2] # calculate x pixel
        pT[1] = pT_vec[1] / pT_vec[2] # calculate y pixel
        pT[2] = pT_vec[2] # calculate depth

        if pT[0] > self.width or pT[0] < 0 or pT[1] > self.height or pT[1] < 0: # if target is out of frame
            pT[:] = np.full(3, np.nan) # update to nan

        # update estimated pixels
        hsv = VideoHSV(rgba=color_image) # convert to hsv
        mask_lower = cv2.inRange(hsv, np.array([0, 100, 100], dtype=np.uint8), np.array([10, 255, 255], dtype=np.uint8)) # lower mask region
        mask_upper = cv2.inRange(hsv, np.array([170, 100, 100], dtype=np.uint8), np.array([179, 255, 255], dtype=np.uint8)) # upper mask region
        mask = cv2.bitwise_or(mask_lower, mask_upper) # create total mask

        xr = np.nan # no valid coordinates initially
        yr = np.nan # no valid coordinates initially
        wr = np.nan # no valid coordindates initially
        hr = np.nan # no valid coordindates initially

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # find contours in mask
        if contours: # if there are contours
            max_contour = max(contours, key=cv2.contourArea) # find maximum contour
            if cv2.contourArea(max_contour) > 20: # if contour is large enough
                xr, yr, wr, hr = cv2.boundingRect(max_contour) # find bounding box dimensions

        pTe[0] = xr + wr / 2 # find estimated target x COM
        pTe[1] = yr + hr / 2 # find estimated target y COM

        if not (np.isnan(pTe[0:2]).any() or np.isinf(pTe[0:2]).any()): # if there are valid pixel estimates
            pTe[2] = depth_image.at(x=int(pTe[0]), y=int(pTe[1]))[0] + config.target_radius # extract depth and compensate for target size
        else: pTe[2] = np.nan # target not found or valid

        # updating target estimate state
        pTe_vec = np.array([pTe[0] * pTe[2], pTe[1] * pTe[2], pTe[2], 1]) # create pixel vector

        if not (np.isnan(pTe).any() or np.isinf(pTe).any()): # if there are no nan or inf values (target is in camera)
            xTe_vec = X_WC.GetAsMatrix4() @ self.Kinv @ pTe_vec # estimate target position in world frame

            xTe[3:6] = xTe_vec[0:3] # estimated x,y,z coordinate
        else:
            xTe[3:6] = np.nan # target is not valid or found

        # update target velocity estimate (OPTICAL FLOW, IMPLEMENT LATER)
        if self.previous_image is not None:
            # OPTICAL FLOW STUFF HERE (IMPLEMENT LATER)
            pass

        xTe[config.qT_num:] = 0 # initialize to all 0
        
        if not (np.isnan(xTe).any() or np.isinf(xTe).any()): # if there are valid target estimates
            if self.previous_xTe is not None:
                vTe_vec = (xTe[:config.qT_num] - self.previous_xTe[:config.qT_num]) / self.dt # estimate target velocities
                xTe[config.qT_num:] = vTe_vec # update state target estimate

        self.previous_xTe = np.copy(xTe) # update stored target state estimate

        # update drone reference state
        x0[:] = 0 # initialize to all 0
        
        if not (np.isnan(xTe).any() or np.isinf(xTe).any()): # if there are valid target estimates
            x0[3] = xTe[3] + config.drone_target_offset_position[0] # set the reference x coordinate for the drone
            x0[4] = xTe[4] + config.drone_target_offset_position[1] # set the reference y coordinate for the drone
            x0[5] = xTe[5] + config.drone_target_offset_position[2] # set the reference z coordinate for the drone  

            # x0[config.q_num + 3:] = xTe[config.qT_num + 3:] # update desired reference position velocities          

            r = xTe[3:6] - x[3:6] # find displacement vector from x to xT
            yaw = np.atan2(r[1], r[0]) # find desired yaw angle for keeping target in view
            x0[2] = yaw # set the reference yaw angle for the drone

            self.previous_x0 = np.copy(x0) # set old reference
        else:
            x0[:] = self.previous_x0[:] # track last known position (or go to origin)

        # save images and update video frames
        self.previous_image = color_image # store last image
            
        color_image = VideoWriteCircle(rgba=color_image, coords=np.array([pT[0], pT[1]]), color=(0, 0, 0), radius=5, thickness=1) # true target COM position
        color_image = VideoWriteCircle(rgba=color_image, coords=np.array([pTe[0], pTe[1]]), color=(0, 200, 200), radius=8, thickness=1) # estimated target COM position
        color_image = VideoWriteRect(rgba=color_image, coords=np.array([xr, yr]), color=(0, 255, 0), dims=(wr, hr), thickness=2) # target bounding box

        # update stored image state
        state.get_mutable_abstract_state(index=self.frame_image_index).set_value(color_image)

##### CAMERA DIAGRAM FUNCTION (creates camera diagram block for given system)
def CameraDiagram() -> Diagram:
    print(f'\n=====CREATING CAMERA DIAGRAM=====')
    
    # initiating the builder
    camera_builder = DiagramBuilder()

    # adding in the camera
    camera = camera_builder.AddNamedSystem(name='camera', system=CameraSystem(camera_info=config.camera_info, fps=config.camera_fps, X_BC=config.X_DC))

    # exposing ports
    camera_builder.ExportInput(input=camera.GetInputPort(port_name='drone_state'), name='drone_state')
    camera_builder.ExportInput(input=camera.GetInputPort(port_name='target_state'), name='target_state')
    camera_builder.ExportInput(input=camera.GetInputPort(port_name='color_image'), name='color_image')
    camera_builder.ExportInput(input=camera.GetInputPort(port_name='depth_image'), name='depth_image')

    camera_builder.ExportOutput(output=camera.GetOutputPort(port_name='reference_state'), name='reference_state')
    camera_builder.ExportOutput(output=camera.GetOutputPort(port_name='target_estimate_state'), name='target_estimate_state')
    camera_builder.ExportOutput(output=camera.GetOutputPort(port_name='target_pixels'), name='target_pixels')
    camera_builder.ExportOutput(output=camera.GetOutputPort(port_name='target_estimate_pixels'), name='target_estimate_pixels')
    camera_builder.ExportOutput(output=camera.GetOutputPort(port_name='frame_image'), name='frame_image')

    # complete diagram
    return camera_builder.Build()