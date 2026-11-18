##### PYDRAKE IMPORTS
from pydrake.systems.framework import LeafSystem
from pydrake.systems.sensors import CameraConfig

##### OTHER IMPORTS
import numpy as np
import cv2

##### CAMERA SYSTEM DEFINITION (LeafSystem for camera)
class CameraSystem(LeafSystem):
    # shared variables

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

    pass

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