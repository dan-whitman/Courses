##### PYDRAKE IMPORTS
from pydrake.systems.framework import DiagramBuilder, OutputPort
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.visualization import AddFrameTriadIllustration
from pydrake.geometry import SceneGraph, StartMeshcat, MeshcatVisualizer, MeshcatVisualizerParams

##### OTHER IMPORTS

##### SELF_DEFINED IMPORTS
import inputs.config as config

##### PLANT SCENE FRAME FUNCTION (adds visual frames to plant/scene graph)
def VisualPlantSceneFrames(plant: MultibodyPlant, scene_graph: SceneGraph) -> tuple[MultibodyPlant, SceneGraph]:
    print(f'\n=====ADDING VISUAL FRAMES=====')

    for i in range(plant.num_model_instances()): # for each model
        model_instance = ModelInstanceIndex(i) # get the model
        body_indices = plant.GetBodyIndices(model_instance=model_instance) # get each body

        for body_index in body_indices: # for each body
            body = plant.get_body(body_index=body_index) # get the body

            # add frame
            AddFrameTriadIllustration(
                scene_graph=scene_graph, plant=plant, body=body, length=config.triad_length, radius=config.triad_radius
            )

    return plant, scene_graph # return updated plant and scene graph

##### VISUALIZER INITIALIZATION FUNCTION (initializes meshcat and updates builder)
def VisualInit(builder: DiagramBuilder, query_object_port: OutputPort) -> tuple[DiagramBuilder, MeshcatVisualizer]:
    print(f'\n=====INITIALIZING VISUALIZER=====')

    config.meshcat = StartMeshcat() # initialize meshcat

    # adding visualizer to builder
    visualizer = MeshcatVisualizer.AddToBuilder(
        builder=builder, query_object_port=query_object_port, meshcat=config.meshcat, params=MeshcatVisualizerParams()
    )

    # returning updated builder and visualizer
    return builder, visualizer

##### VISUALIZER START FUNCTION (begins meshcat recording)
def VisualStart() -> None:
    print(f'\n=====STARTING VISUALIZATION=====')

    config.meshcat.StartRecording(set_visualizations_while_recording=True) # begin recording

##### VISUALIZER STOP FUNCTION (stops and published meshcat recording)
def VisualStop() -> None:
    print(f'\n=====STOPPING VISUALIZATION=====')

    config.meshcat.StopRecording() # stop simulation
    config.meshcat.PublishRecording() # publish simulation

##### VISUALIZER HOLD FUNCTION (keeps visualizer open after simulation)
def VisualHold() -> None:
    input(f'\n=====KEEPING MESHCAT ALIVE=====\n')