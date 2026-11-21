##### PYDRAKE IMPORTS
from pydrake.systems.framework import DiagramBuilder, Diagram, Context
from pydrake.systems.analysis import Simulator

##### OTHER IMPORTS
import cv2

##### SELF_DEFINED IMPORTS
import inputs.config as config
from utils.visual import VisualInit, VisualStart, VisualStop

##### SIMULATION DIAGRAM FUNCTION (inputs closed diagram and adds visualizer for simulation)
def SimDiagram(closed_diagram: Diagram) -> Diagram:
    print(f'\n=====CREATING SIMULATION DIAGRAM=====')

    # initiating the builder
    sim_builder = DiagramBuilder()

    # adding the trajectory diagram
    sim_builder.AddNamedSystem(name='closed_diagram', system=closed_diagram)

    # adding visualizaer
    sim_builder, _ = VisualInit(builder=sim_builder, query_object_port=closed_diagram.GetOutputPort(port_name='geometry_query'))

    # exposing ports
    sim_builder.ExportOutput(output=closed_diagram.GetOutputPort(port_name='drone_state'), name='drone_state') # for printing/tracking hexrotor state
    sim_builder.ExportOutput(output=closed_diagram.GetOutputPort(port_name='target_state'), name='target_state') # for printing tracking target state
    sim_builder.ExportOutput(output=closed_diagram.GetOutputPort(port_name='reference_state'), name='reference_state') # for printing reference state
    sim_builder.ExportOutput(output=closed_diagram.GetOutputPort(port_name='propeller_thrusts'), name='propeller_thrusts') # for printing/tracking propeller thrusts
    sim_builder.ExportOutput(output=closed_diagram.GetOutputPort(port_name='target_estimate_state'), name='target_estimate_state') # drone estimate of target state
    sim_builder.ExportOutput(output=closed_diagram.GetOutputPort(port_name='target_pixels'), name='target_pixels') # true camera pixels for target base
    sim_builder.ExportOutput(output=closed_diagram.GetOutputPort(port_name='target_estimate_pixels'), name='target_estimate_pixels') # estimated camera pixels for target base  
    sim_builder.ExportOutput(output=closed_diagram.GetOutputPort(port_name='frame_image'), name='frame_image') # for creating cv2 video

    # complete diagram
    return sim_builder.Build()

##### SIMULATOR CONTEXT FUNCTION (creates simulator context from diagram)
def SimContext(sim_diagram: Diagram, closed_context: Context) -> Context:
    print(f'\n=====CREATING SIMULATION CONTEXT=====')

    # creating contexts
    sim_context = sim_diagram.CreateDefaultContext() # creating numerical context
    closed_diagram = sim_diagram.GetSubsystemByName(name='closed_diagram') # obtaining closed-loop system

    sim_closed_context = closed_diagram.GetMyMutableContextFromRoot(root_context=sim_context)
    sim_closed_context.SetStateAndParametersFrom(source=closed_context)

    # return context
    return sim_context

##### SIMULATOR RUN FUNCTION (runs the simulator)
def SimRun(sim_diagram: Diagram, sim_context: Context) -> None:
    print(f'\n=====STARTING SIMULATION=====')

    # creating simulator    
    simulator = Simulator(system=sim_diagram, context=sim_context)
    simulator.Initialize()
    simulator.set_target_realtime_rate(realtime_rate=config.simulation_rate)

    # starting visualization
    VisualStart()

    # creating video output
    if config.args.video == 1: # if outputting video
        config.video = cv2.VideoWriter(
            filename=str(config.VID_DIR / config.vid_file_name), apiPreference=0, fourcc=cv2.VideoWriter_fourcc(*'MJPG'), fps=config.camera_fps, frameSize=(config.camera_width, config.camera_height)
        )

    # running simulation
    for frame_time in config.frame_times: # for each time step
        simulator.AdvanceTo(boundary_time=frame_time) # advance simulation

        # evaluate current data and log
        x = sim_diagram.GetOutputPort(port_name='drone_state').Eval(sim_context) # evaluate drone state
        xT = sim_diagram.GetOutputPort(port_name='target_state').Eval(sim_context) # evaluate target state
        x0 = sim_diagram.GetOutputPort(port_name='reference_state').Eval(sim_context) # evaluate controller reference state
        u = sim_diagram.GetOutputPort(port_name='propeller_thrusts').Eval(sim_context) # evaluate propeller thrusts
        xTe = sim_diagram.GetOutputPort(port_name='target_estimate_state').Eval(sim_context) # evaluate estimated target state
        pT = sim_diagram.GetOutputPort(port_name='target_pixels').Eval(sim_context) # evaluate target pixels in camera
        pTe = sim_diagram.GetOutputPort(port_name='target_estimate_pixels').Eval(sim_context) # evaluate estimated target pixels in camera
        rgba = sim_diagram.GetOutputPort(port_name='frame_image').Eval(sim_context) # get camera frame image

        # DO STUFF HERE (LOG DATA)

        xT[5] = xT[5] + config.target_height / 2
        # print(f'{frame_time:.2f}', xT[3:6], xTe[3:6])
        print(f'{frame_time:.2f}', pT, pTe)
        # print(f'{frame_time:.2f}', x[3:6], x0[3:6])

        # adding frame to video
        if config.args.video == 1:
            rgb = rgba[:,:,:3] # remove alpha channel from data
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) # convert to bgr color for opencv
            config.video.write(bgr) # add frame to video

    # end visualization
    VisualStop()