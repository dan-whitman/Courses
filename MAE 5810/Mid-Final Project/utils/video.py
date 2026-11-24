##### PYDRAKE IMPORTS

##### OTHER IMPORTS
import numpy as np
import cv2

##### SELF-DEFINED IMPORTS
import inputs.config as config

##### VIDEO ADD POINT FUNCTION (adds point at coords to video frame)
def VideoWriteCircle(rgba: np.ndarray, coords: np.ndarray, color: tuple, radius: float, thickness: float) -> np.ndarray:
    # print(f'\n=====ADDING CIRCLE TO IMAGE=====')

    # checking if coordinates are valid
    if np.isnan(coords).any(): return rgba
    
    # adding marker to image
    cv2.circle(
        img=rgba, center=(int(coords[0]), int(coords[1])), radius=radius, color=color, thickness=thickness, lineType=cv2.LINE_AA 
    )
    
    return rgba # return new image

##### VIDEO ADD RECTANGLE FUNCTION (adds rectangle at coords to video frame)
def VideoWriteRect(rgba: np.ndarray, coords: np.ndarray, color: tuple, dims: tuple, thickness: float) -> np.ndarray:
    # print(f'\n=====ADDING RECTANGLE TO IMAGE=====')

    # checking if coordinates are valid
    if np.isnan(coords).any(): return rgba

    # converting to arguments
    pt1 = (coords[0], coords[1]) # top-left rectangle corner
    pt2 = (coords[0] + dims[0], coords[1] + dims[1]) # lower-right rectangle corner

    # adding marker to image
    cv2.rectangle(img=rgba, pt1=pt1, pt2=pt2, color=color, thickness=thickness, lineType=cv2.LINE_AA)

    return rgba # return new image


##### RGBA TO BGR IMAGE CONVERSION FUNCTION (inputs rgba frame and returns bgr version)
def VideoBGR(rgba: np.ndarray) -> np.ndarray:
    # print(f'\n=====CONVERTING TO BGR=====')

    # convert to bgr and return
    rgb = rgba[:,:,:3] # remove alpha channel from data
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) # return bgr image

##### RGBA TO HSV IMAGE CONVERSION FUNCTION (inputs rgba frame and returns hsv version)
def VideoHSV(rgba: np.ndarray) -> np.ndarray:
    # print(f'\n=====CONVERTING TO HSV=====')

    # convert to hsv and return
    rgb = rgba[:,:,:3] # remove alpha channel from data
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV) # return hsv image

##### VIDEO FRAME WRITE FUNCTION (inputs rbga frame and adds to video)
def VideoFrame(rgba: np.ndarray) -> None:
    # print(f'\n=====ADDING FRAME TO VIDEO=====')

    bgr = VideoBGR(rgba=rgba) # cinvert to bgr
    config.video.write(bgr) # add frame
