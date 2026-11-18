##### PYDRAKE IMPORTS

##### OTHER IMPORTS
import numpy as np

##### SELF_DEFINED IMPORTS
import inputs.config as config

##### CAMERA INFORMATION PRINT FUNCTION (prints information about camera)
def PrintCamera() -> None:
    print(f'\n=====CAMERA INFORMATION=====')

    # print all information
    print('width:', config.camera_info.width)
    print('height:', config.camera_info.height)
    print('fx:', config.camera_info.focal_x)
    print('fy:', config.camera_info.focal_y)
    print('ux:', config.camera_info.center_x)
    print('uy:', config.camera_info.center_y)
    print('Horizontal FOV:', np.degrees(config.camera_info.fov_x()), 'deg')
    print('Vertical FOV:', np.degrees(config.camera_info.fov_y()), 'deg')