##### PYDRAKE IMPORTS
from pydrake.systems.framework import LeafSystem, Context, BasicVector, Diagram, DiagramBuilder
from pydrake.systems.sensors import CameraInfo, ImageRgba8U, ImageDepth32F
from pydrake.common.value import AbstractValue
from pydrake.math import RigidTransform, RollPitchYaw

##### OTHER IMPORTS
import numpy as np
import cv2

##### SELF-DEFINED IMPORTS
import inputs.config as config
from utils.video import VideoWritePoint

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
        self.drone_state_port = self.DeclareVectorInputPort(name='drone_state', size=config.q_num + config.v_num) # drone state input port
        self.target_state_port = self.DeclareVectorInputPort(name='target_state', size=config.qT_num + config.vT_num) # target state input port
        self.color_image_port = self.DeclareAbstractInputPort(name='color_image', model_value=AbstractValue.Make(ImageRgba8U(width=self.width, height=self.height))) # color image input port
        self.depth_image_port = self.DeclareAbstractInputPort(name='depth_image', model_value=AbstractValue.Make(ImageDepth32F(width=self.width, height=self.height))) # depth image input port

        self.reference_state_port = self.DeclareVectorOutputPort(name='reference_state', size=config.q_num + config.v_num, calc=self.UpdateReference) # calculated drone reference state output
        self.target_estimate_state_port = self.DeclareVectorOutputPort(name='target_estimate_state', size=config.qT_num + config.vT_num, calc=self.UpdateEstimate) # estimated target state output from camera image
        self.target_pixels_port = self.DeclareVectorOutputPort(name='target_pixels', size=3, calc=self.UpdateTargetPixels) # true target COM pixels in camera frame
        self.target_estimate_pixels_port = self.DeclareVectorOutputPort(name='target_estimate_pixels', size=3, calc=self.UpdateEstimatePixels) # estimated target COM pixels in camera frame
        self.frame_image_port = self.DeclareStateOutputPort(name='frame_image', state_index=self.frame_image_index) # output port for modified cv2 image
        # self.frame_image_port = self.DeclareAbstractOutputPort(name='frame_image', 
        #                                alloc=lambda: AbstractValue.Make(np.zeros((camera_height, camera_width, 4), dtype=np.uint8)),
        #                                calc=self.ExportFrame
        # ) # output port for modified cv2 image

    # update altered frame and references function
    def UpdateFrame(self, context: Context, state):
        # read current states from context
        color_image = np.copy(self.color_image_port.Eval(context).data) # reading current color image data
        depth_image = self.depth_image_port.Eval(context) # reading current depth image

        if self.previous_image is not None:
            # do optical flow with cv2 (IMPLEMENT LATER)
            pass

        self.previous_image = color_image # store last image

        # do cv2 bounding box stuff to figure out what estimate target state is (IMPLEMENT LATER)
        # need to make uTe,vTe,wTe nan if target is out of camera (bounding box is not wide/tall enough or no bounding box found)
        self.uTe = self.uT # DIRECT COPY (CHANGE LATER)
        self.vTe = self.vT # DIRECT COPY (CHANGE LATER)

        if not np.isnan(np.array([self.uTe, self.vTe])).any(): # if there are valid pixel estimates
            self.wTe = depth_image.at(x=int(self.uTe), y=int(self.vTe))[0] + config.target_radius # extract depth and compensate for target size

            self.wTe = self.wT # DIRECT COPY (CHANGE LATER)

        # add overlays to image
        color_image = VideoWritePoint(rgba=color_image, coords=np.array([self.uT, self.vT]), color=(0, 0, 0), radius=6, thickness=2) # true target COM position
        color_image = VideoWritePoint(rgba=color_image, coords=np.array([self.uTe, self.vTe]), color=(0, 200, 200), radius=10, thickness=2) # estimated target COM position

        # update stored image state
        state.get_mutable_abstract_state(int(self.frame_image_index)).set_value(color_image)

    # update drone reference output function
    def UpdateReference(self, context: Context, output: BasicVector):
        # read current states from context

        # updating reference state based on internal target states
        x0 = np.zeros(config.q_num + config.v_num) # initialize to all 0

        x0[3] = self.xTe[3] + config.drone_target_offset_position[0] # set the reference x coordinate for the drone
        x0[4] = self.xTe[4] + config.drone_target_offset_position[1] # set the reference y coordinate for the drone
        x0[5] = self.xTe[5] + config.drone_target_offset_position[2] # set the reference z coordinate for the drone

        # update output
        output.SetFromVector(value=x0) # return new reference state

    # update target estimate based on bounding box pixels and optical flow
    def UpdateEstimate(self, context: Context, output: BasicVector):
        # read current states from context
        x = self.drone_state_port.Eval(context) # current drone state
        xT = self.target_state_port.Eval(context) # current target state (DELETE LATER, FOR TESTING)

        # calculating transformations
        X_WD = RigidTransform(rpy=RollPitchYaw(x[0], x[1], x[2]), p=x[3:6]) # transform of drone with respect to world
        X_WC = X_WD.multiply(other=self.X_BC) # transform for camera with respect to world

        # applying transformations
        pTe = np.array([self.uTe, self.vTe, self.wTe, 1]) # create pixel vector

        if not np.isnan(pTe).any(): # if there are no nan values (target is in camera)
            xTe = X_WC.GetAsMatrix4() @ np.linalg.inv(self.K) @ pTe # estimate target position in world frame

            self.xTe[3] = xTe[0] # estimated x coordinate
            self.xTe[4] = xTe[1] # estimated y coordinate
            self.xTe[5] = xTe[2] # estimated z coordinate

            self.xTe[3] = xT[3] # DIRECT COPY (CHANGE LATER)
            self.xTe[4] = xT[4] # DIRECT COPY (CHANGE LATER)
            self.xTe[5] = xT[5] + config.target_height / 2 # DIRECT COPY (CHANGE LATER)

        # UPDATE VELOCITIES BASED ON OPTICAL FLOW RESULTS AND DEPTH DIFFERENCES HERE

        # update output
        output.SetFromVector(value=self.xTe) # return new estimated state

    # update true target pixels in camera frame
    def UpdateTargetPixels(self, context: Context, output: BasicVector):
        # read current states from context
        x = self.drone_state_port.Eval(context) # current drone state
        xT = self.target_state_port.Eval(context) # current target state

        # calculating transformations
        X_WD = RigidTransform(rpy=RollPitchYaw(x[0], x[1], x[2]), p=x[3:6]) # transform of drone with respect to world
        X_WC = X_WD.multiply(other=self.X_BC) # transform of camera with respect to world
        X_CW = X_WC.inverse() # transform of world with respect to camera

        # applying transformations
        xT[5] = xT[5] + config.target_height / 2 # set height at center of target body
        
        pT = self.K @ X_CW.GetAsMatrix4() @ np.append(xT[3:6], 1) # pixels in camera frame

        self.wT = pT[2] # calculate target depth
        self.uT = pT[0] / self.wT # calculate x pixel
        self.vT = pT[1] / self.wT # calculate y pixel

        if self.uT > self.width or self.uT < 0 or self.vT > self.height or self.vT < 0: # if target is out of frame
            self.uT = np.nan # update to nan
            self.vT = np.nan # update to nan

        # update output
        output.SetFromVector(value=np.array([self.uT, self.vT, self.wT])) # return calculated true pixels

    # update estimated target pixels in camera frame
    def UpdateEstimatePixels(self, context: Context, output: BasicVector):
        # read current states

        # update output
        output.SetFromVector(value=np.array([self.uTe, self.vTe, self.wTe])) # return calculated estimated pixels

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