##### PYDRAKE IMPORTS
from pydrake.math import RollPitchYaw

##### OTHER IMPORTS
import numpy as np
import argparse
from pathlib import Path

##### SELF-DEFINED IMPORTS

##### USER INPUTS
user_name = 'DAN'
camera_width = 640 # (px)
camera_height = 480 # (px)
camera_fps = 15 # (fps)
camera_fx = 500 # (px)
camera_fy = 500 # (px)

simulation_duration = 5.0 # (s)
drone_initial_position = [0.0, 0.0, 0.5] # (m)
drone_initial_rpy = RollPitchYaw(0.0, 0.0, 0.0) # (rad)
target_initial_position = [1.0, 3.0, 0.0] # (m)

##### SCRIPT ARGUMENTS
argparser = argparse.ArgumentParser(description='HexRotor Simulation Model') # creating parser

argparser.add_argument('-f', '--frames', type=int, required=False, default=0, help='(1) for model frame visibility, (!=1) for no frames')
argparser.add_argument('-v', '--video', type=int, required=False, default=1, help='(1) for simulation video output, (!=1) for no video')
argparser.add_argument('-g', '--figures', type=int, required=False, default=1, help='(1) for figure outputs, (!=1) for no figures')
argparser.add_argument('-d', '--data', type=int, required=False, default=1, help='(1) for data output, (!=1) for no data')

##### SCRIPT DIRECTORIES
INP_DIR = Path('inputs') # inputs directory
MOD_DIR = Path('models') # model directory
UTI_DIR = Path('utils') # utils directory

ALL_DIR = (INP_DIR, MOD_DIR, UTI_DIR, MIS_DIR) # directory tuple