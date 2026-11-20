##### PYDRAKE IMPORTS

##### OTHER IMPORTS
import numpy as np
import cv2

##### SELF-DEFINED IMPORTS

##### VIDEO ADD POINT FUNCTION (adds point at coords to video frame)
def VideoWritePoint(rgba: np.ndarray, coords: np.ndarray, color: tuple, radius: float, thickness: float) -> np.ndarray:
    # print(f'\n=====ADDING POINT TO IMAGE=====')

    # checking if coordinates are valid
    if np.isnan(coords).any(): return rgba
    
    # adding marker to image
    cv2.circle(
        img=rgba, center=(int(coords[0]), int(coords[1])), radius=radius, color=color, thickness=thickness, lineType=cv2.LINE_AA 
    )
    # return new image
    return rgba