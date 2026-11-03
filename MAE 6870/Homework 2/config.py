##### IMPORTS
from pathlib import Path
import numpy as np

##### SCRIPT DIRECTORIES
DAT_DIR = Path('data') # data directory
FIG_DIR = Path('figs') # figure directory
UTI_DIR = Path('utils') # utils directory

##### GLOBAL VARIABLES
grid_distance = 1 # (m)
temp_heated = 20 # (C)
temp_unheated = 20 # (C)

radius_sensing = 3.5 # (m)
radius_comm = 4.5 # (m)

coords_robot1 = (4, 4) # (coords)
coords_robot2 = (1, 1) # (coords)
coords_robot3 = (1, 7) # (coords)
coords_robot4 = (4, 8) # (coords)
coords_robot5 = (8, 7) # (coords)
coords_robot6 = (8, 3) # (coords)
coords_robot7 = (5, 1) # (coords)

coords_robots = (coords_robot1, 
                 coords_robot2, 
                 coords_robot3, 
                 coords_robot4, 
                 coords_robot5, 
                 coords_robot6,
                 coords_robot7)
num_robots = len(coords_robots)

seed = 100 # for reproducability/testing
rng = np.random.default_rng(seed=seed)

grid_temp_colors = ('#4292c6', '#f01e2c')
grid_temp_vmin = 10 # minimum colorbar temperature for plotting (^oC)
grid_temp_vmax = 50 # maximum colorbar temperature for plotting (^oC)

grid_error_colors = ("#ff0000", "#00ff00")
grid_error_vmin = -15 # minimum colorbar error for plotting (^oC)
grid_error_vmax = 15 # maximum colorbar error for plotting (^oC)