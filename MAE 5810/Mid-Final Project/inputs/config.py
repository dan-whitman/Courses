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
camera_fx = 450 # (px)
camera_fy = 400 # (px)
camera_zmax = 20.0 # (m)
camera_zmin = 0.1 # (m)

simulation_duration = 10.0 # (s)
simulation_rate = 4.0 # ()

frame_times = np.arange(0.0, simulation_duration, 1.0 / camera_fps)

drone_initial_position = np.array([-2.0, -4.0, 1.0]) # (m)
drone_initial_rpy = RollPitchYaw(0.0, 0.0, np.pi/2) # (rad)

# drone_initial_position = np.array([-2.0, 4.0, 1.0]) # (m)
# drone_initial_rpy = RollPitchYaw(0.0, 0.0, 0.0) # (rad)

target_initial_position = np.array([0.5, 5.5, 0.0]) # (m)
target_initial_rpy = RollPitchYaw(0.0, 0.0, 0.0) # (rad)
target_final_position = np.array([15.0, -2.0, 0.0]) # (m)
target_final_rpy = RollPitchYaw(0.0, 0.0, 0.0) # (rad)
target_motion_duration = simulation_duration # (s)

obstacle_initial_position = np.array([2.0, -4.0, 0.0])
obstacle_initial_rpy = RollPitchYaw(0.0, 0.0, 0.0)

drone_target_offset_position = np.array([-1.5, 1.5, 0.1]) # (m) desired drone offset from target

##### SAVE FILE NAMES
vid_file_name = 'fpv_camera_' + user_name + '.avi'
heading_file_name = 'camera_heading_' + user_name
pixel_file_name = 'pixel_measurement_' + user_name
data_file_name = 'model_data_' + user_name

##### DRONE INFORMATION
prop_thrust_ratio = 1.0 # ()
prop_moment_ratio = 0.1 # ()
prop_min_thrust = 0.0 # (N)
prop_max_thrust = 3.0 # (N)

##### TARGET INFORMATION
target_height = 1.0 # (m)
target_radius = 0.25 # (m)

##### CONTROL GAINS
lqr_pos_gains = 50.0 # position control gains for lqr
lqr_rpy_gains = 100.0 # rpy control gains for lqr

lqr_pos_vel_gains = 200.0 # linear velocity gains for lqr
lqr_rot_vel_gains = 100.0 # rotational velocity gains for lqr

lqr_thrust_gains = 1.0 # propeller thrust gains for lqr

Kp_gain = 80 # proportional gain for PD
Kd_gain = 30 # derivative gain for PD

##### DRAKE PARAMETERS
static_friction = 0.01 # () world ground surface friction
dynamic_friction = 0.01 # () world ground dynamic friction

triad_length = 0.15 # (m) for visualization
triad_radius = 0.005 # (m) for visualization

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
VID_DIR = Path('vids') # video directory
FIG_DIR = Path('figs') # figure directory
DAT_DIR = Path('data') # data directory

LOC_DIR = None # local script execution directory
ALL_DIR = None # all directories

##### STORED GLOBAL PARAMETERS
args = None # empty parser arguments

g_mag = None # gravity

q_num = None # number of drone state coordinates
v_num = None # number of drone state velocities
u_num = None # number of control inputs

qT_num = None # number of target state coordinates
vT_num = None # number of target state velocities

drone_instance = None # drone plant index for reference
drone_mass = None # (kg) drone mass
target_instance = None # target plant index for reference
target_index = None # target plant base index for control
obstacle_instance = None # obstacle plant index for reference

u0 = None # (N) reference equilibrium propeller thrusts

camera_info = None # camera infomration
X_DC = None # camera transform to drone body

meshcat = None # meshcat instance
video = None # video instance