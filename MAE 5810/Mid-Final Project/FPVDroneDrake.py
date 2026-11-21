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
from pathlib import Path

##### SELF-DEFINED IMPORTS
import inputs.config as config
from utils.system import DroneDiagram, DroneInitialContext
from utils.control import ControlLQRDiagram, ControlDiagram, ControlContext
from utils.camera import CameraDiagram
from utils.simulate import SimDiagram, SimContext, SimRun
from utils.visual import VisualHold
from utils.print import PrintCamera

# from utils.figure import PlotData
# from utils.video import WriteVideoText, WriteVideoPoint

##### SCRIPT INITIALIZATION FUNCTION (initializes commands and config args)
def ScriptInit() -> None:
    print(f'\n=====INITIALIZING SCRIPT=====')

    # making directories
    config.LOC_DIR = Path(__file__).resolve().parent # resolve current directory

    # updating config paths (NEED TO UPDATE EVERY TIME MAKE NEW DIRECTORY)
    config.INP_DIR = config.LOC_DIR / config.INP_DIR # inputs directory
    config.MOD_DIR = config.LOC_DIR / config.MOD_DIR # model directory
    config.UTI_DIR = config.LOC_DIR / config.UTI_DIR # utils directory
    config.VID_DIR = config.LOC_DIR / config.VID_DIR # video directory
    config.FIG_DIR = config.LOC_DIR / config.FIG_DIR # figure directory
    config.DAT_DIR = config.LOC_DIR / config.DAT_DIR # data directory

    config.ALL_DIR = ( # combined directory list (NEED TO UPDATE EVERY TIME MAKE NEW DIRECTORY)
        config.INP_DIR,
        config.MOD_DIR,
        config.UTI_DIR,
        config.VID_DIR,
        config.FIG_DIR,
        config.DAT_DIR,
    )

    # making directories
    for DIR in config.ALL_DIR: # for each directory
        DIR.mkdir(parents=True, exist_ok=True) # create if doesn't exist

    # parsing arguments
    config.args = config.argparser.parse_args()

##### MODEL SCRIPT (runs the model)
if __name__ == '__main__':
    ScriptInit() # initialize the script with arguments

    # creating open-loop system
    open_diagram = DroneDiagram(with_camera=True) # creating open-loop diagram
    initial_context = DroneInitialContext(open_diagram=open_diagram, with_camera=True) # define open-loop initial context

    # creating linearizable open-loop system for controller
    open_linear_diagram = DroneDiagram(with_camera=False) # exclude camera for linearizability
    initial_linear_context = DroneInitialContext(open_diagram=open_linear_diagram, with_camera=False) # excluded camera context

    # creating closed-loop control system (CREATE CAMERA DIAGRAM FOR PROCESSING AND CONTROLLER DIAGRAM)
    control_diagram = ControlLQRDiagram(open_diagram=open_linear_diagram, initial_context=initial_linear_context) # create lqr controller
    camera_diagram = CameraDiagram() # create camera diagram

    closed_diagram = ControlDiagram(open_diagram=open_diagram, control_diagram=control_diagram, camera_diagram=camera_diagram) # create closed-loop diagram
    closed_context = ControlContext(closed_diagram=closed_diagram, initial_context=initial_context) # create closed-loop context

    # creating simulation system
    sim_diagram = SimDiagram(closed_diagram=closed_diagram) # create final simulaton diagram
    sim_context = SimContext(sim_diagram=sim_diagram, closed_context=closed_context) # define final simulation context
    SimRun(sim_diagram=sim_diagram, sim_context=sim_context) # running the simulation

    # printing system information
    PrintCamera()

    # exporting data
    if config.args.video == 1: config.video.release() # save video
    
    # holding visualization open
    VisualHold()

    print(f'=====CLOSING SCRIPT=====')


'''
##### OUTPUTTING DATA
print(f'\n=====SAVING RESULTS=====')

# exporting data
if args.data == 1: # if outputting data
    np.savez(str(DAT_DIR / data_file_name), 
        times=times, drone_positions=drone_positions, drone_rpys=drone_rpys, drone_velocities=drone_velocities, drone_w_velocities=drone_w_velocities
    )

# plotting results
if args.figures == 1: # if outputting figures
    plotdata = [times, camera_headings, us, vs, camera_width, camera_height] # combining data to pass to function (bad way to do this, rethink later)
    figheadings, figtargetpixels = PlotData(plotdata=plotdata) # plotting figures

    figheadings.savefig(str(FIG_DIR / heading_file_name), dpi=300, bbox_inches='tight')
    figtargetpixels.savefig(str(FIG_DIR / pixel_file_name), dpi=300, bbox_inches='tight')
'''