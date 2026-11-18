##### CONFIG
# must run the following in terminal to establish pydrake environment variables OR add to ~./bashrc:
'''
export PATH="/opt/drake/bin${PATH:+:${PATH}}"
export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
'''

##### PYDRAKE IMPORTS
# from pydrake.all import 

##### OTHER IMPORTS
import numpy as np

##### SELF-DEFINED IMPORTS
import inputs.config as config
from utils.system import DroneDiagram, DroneInitialContext
from utils.control import ControlLQRDiagram, ControlDiagram, ControlContext
from utils.simulate import SimDiagram, SimContext, SimRun
from utils.visual import VisualHold

from utils.camera import CameraMeasure
from utils.figure import PlotData
from utils.video import WriteVideoText, WriteVideoPoint

##### SCRIPT INITIALIZATION FUNCTION (initializes commands and config args)
def ScriptInit() -> None:
    print(f'\n=====INITIALIZING SCRIPT=====')

    # making directories
    for DIR in config.ALL_DIR: # for each directory
        DIR.mkdir(parents=True, exist_ok=True) # create if doesn't exist

    # parsing arguments
    config.args = config.argparser.parse_args()

##### MODEL SCRIPT (runs the model)
if __name__ == '__main__':
    ScriptInit() # initialize the script with arguments

    # creating open-loop system
    open_diagram = DroneDiagram() # creating open-loop diagram
    initial_context = DroneInitialContext(open_diagram=open_diagram) # define open-loop initial context

    # creating closed-loop control system (CREATE CAMERA DIAGRAM FOR PROCESSING AND CONTROLLER DIAGRAM)
    control_diagram = ControlLQRDiagram(open_diagram=open_diagram, initial_context=initial_context) # create lqr controller
    camera_diagram = # create camera diagram

    closed_diagram = ControlDiagram(open_diagram=open_diagram, control_diagram=control_diagram, camera_diagram=camera_diagram) # create closed-loop diagram
    closed_context = ControlContext(closed_diagram=closed_diagram, initial_context=initial_context) # create closed-loop context

    # creating simulation system

    # printing system information

    # exporting data
    
    # holding visualization open
    if config.argss.visual == 1: VisualHold()

    print(f'=====CLOSING SCRIPT=====')



# data values
times = []
camera_headings = []

drone_positions = []
drone_rpys = []
drone_velocities = []
drone_w_velocities = []

us = []
vs = []

# step through simulation
for t in frame_times:
    # t = frame / camera_fps # simulation time
    simulator.AdvanceTo(t) # advance simulation
    camera_sim_context = camera.GetMyContextFromRoot(simulator.get_context()) # get camera context at t
    plant_sim_context = plant.GetMyContextFromRoot(simulator.get_context()) # get plant context at t

    # finding data of interest
    X_WC = camera.body_pose_in_world_output_port().Eval(camera_sim_context) # finding camera pose
    X_CW = X_WC.inverse() # finding world in camera
    R_WC = X_WC.rotation().matrix() # camera rotation (in world frame)
    heading_WC = R_WC @ np.array([0.0, 0.0, 1.0]) # extracting heading

    X_WD = plant.EvalBodyPoseInWorld(plant_sim_context, drone_body) # finding drone pose
    V_WD = plant.EvalBodySpatialVelocityInWorld(plant_sim_context, drone_body) # finding drone velocoties
    p_WD = X_WD.translation() # drone position (in world frame)
    R_WD = X_WD.rotation().matrix() # drone rotation (in world frame)
    rpy = RollPitchYaw(R_WD) # finding rpy
    rpy = [rpy.roll_angle(), rpy.pitch_angle(), rpy.yaw_angle()] # finding rpy
    v_WD = V_WD.translational() # drone linear velocity (in world frame)
    w_WD = V_WD.rotational() # drone rotational velocoty (in world frame)

    X_WT = plant.EvalBodyPoseInWorld(plant_sim_context, target_body) # finding target pose
    x_T = X_WT.translation() # target position (in world frame)

    u, v = CameraMeasure(config, X_CW, x_T) # finding target measurement in camera

    # appending to data variables
    times.append(t)
    camera_headings.append(heading_WC.copy())

    drone_positions.append(p_WD.copy())
    drone_rpys.append(rpy.copy())
    drone_velocities.append(v_WD.copy())
    drone_w_velocities.append((R_WD.T @ np.array(w_WD)).copy())

    us.append(u)
    vs.append(v)

    if args.video == 1: # if outputting video
        image = camera.color_image_output_port().Eval(camera_sim_context) # getting color image 
        rgba = image.data # extracting data
        rgba = rgba.copy() # creating copy for manipulation

        rgba = WriteVideoText(rgba=rgba, textflag=u == None, t=t) # add text to image
        rgba = WriteVideoPoint(rgba=rgba, coords=(u, v)) # add point to image

        rgb = rgba[:,:,:3] # removing apha channel
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) # converting to bgr for opencv
        video.write(bgr) # add frame to video

# convert data and parse
times = np.array(times)
camera_headings = np.array(camera_headings)

drone_positions = np.array(drone_positions)
drone_rpys = np.array(drone_rpys)
drone_velocities = np.array(drone_velocities)
drone_w_velocities = np.array(drone_w_velocities)

us = np.array(us)
vs = np.array(vs)


##### OUTPUTTING DATA
print(f'\n=====SAVING RESULTS=====')

# exporting video
if args.video == 1: # if outputting video
    video.release() # save video
    print('Video saved!')

# exporting data
if args.data == 1: # if outputting data
    np.savez(str(DAT_DIR / data_file_name), 
        times=times, drone_positions=drone_positions, drone_rpys=drone_rpys, drone_velocities=drone_velocities, drone_w_velocities=drone_w_velocities
    )
    print('Data saved!')

# plotting results
if args.figures == 1: # if outputting figures
    plotdata = [times, camera_headings, us, vs, camera_width, camera_height] # combining data to pass to function (bad way to do this, rethink later)
    figheadings, figtargetpixels = PlotData(plotdata=plotdata) # plotting figures

    figheadings.savefig(str(FIG_DIR / heading_file_name), dpi=300, bbox_inches='tight')
    figtargetpixels.savefig(str(FIG_DIR / pixel_file_name), dpi=300, bbox_inches='tight')

    print('Figures saved!')

##### PRINTING INFO
print(f'\n=====CAMERA INFO=====')
print('fx:', config.focal_x())
print('fy:', config.focal_y())
print('ux:', config.principal_point()[0])
print('uy:', config.principal_point()[1])
print('Horizontal FOV:', np.degrees(camera_info.fov_x()), 'deg')
print('Vertical FOV:', np.degrees(camera_info.fov_y()), 'deg')

input(f'\nKeeping Meshcat alive!')