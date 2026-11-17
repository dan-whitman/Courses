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