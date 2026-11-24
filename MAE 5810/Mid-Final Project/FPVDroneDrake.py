##### CONFIG
# must run the following in terminal to establish pydrake environment variables OR add to ~./bashrc:
'''
export PATH="/opt/drake/bin${PATH:+:${PATH}}"
export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
'''

##### PYDRAKE IMPORTS
# from pydrake.all import 

##### OTHER IMPORTS
from pathlib import Path

##### SELF-DEFINED IMPORTS
import inputs.config as config
from utils.system import DroneDiagram, DroneInitialContext
from utils.control import ControlLQRDiagram, ControlDiagram, ControlContext
from utils.camera import CameraDiagram
from utils.simulate import SimDiagram, SimContext, SimRun
from utils.visual import VisualHold
from utils.print import PrintCamera
from utils.logging import LogSave

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
    open_diagram = DroneDiagram(full_plant=True) # creating open-loop diagram
    initial_context = DroneInitialContext(open_diagram=open_diagram, full_plant=True) # define open-loop initial context

    # creating linearizable open-loop system for controller
    open_drone_diagram = DroneDiagram(full_plant=False) # exclude camera for linearizability

    # creating closed-loop control system (CREATE CAMERA DIAGRAM FOR PROCESSING AND CONTROLLER DIAGRAM)
    control_diagram = ControlLQRDiagram(open_diagram=open_drone_diagram) # create lqr controller
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

    if config.args.data == 1: # if saving data
        LogSave(data=config.t_log, cols=len(config.t_log), filename='t_log.npy')
        LogSave(data=config.x_log, cols=len(config.t_log), filename='x_log.npy')
        LogSave(data=config.xT_log, cols=len(config.t_log), filename='xT_log.npy')
        LogSave(data=config.x0_log, cols=len(config.t_log), filename='x0_log.npy')
        LogSave(data=config.u_log, cols=len(config.t_log), filename='u_log.npy')
        LogSave(data=config.xTe_log, cols=len(config.t_log), filename='xTe_log.npy')
        LogSave(data=config.pT_log, cols=len(config.t_log), filename='pT_log.npy')
        LogSave(data=config.pTe_log, cols=len(config.t_log), filename='pTe_log.npy')
    
    # holding visualization open
    VisualHold()

    print(f'=====CLOSING SCRIPT=====')