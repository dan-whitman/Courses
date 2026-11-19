##### IMPORTS
import numpy as np
from pathlib import Path

##### GLOBAL PARAMETERS
simulate = True # set if simulating (NEEDS TO BE FALSE BEFORE SUBMITTING JOB)

linear_velocity_limit = 0.7 # velocity limit for thresholding
linear_velocity_gain = 0.6 # velocity gain for controller
angular_velocity_limit = np.pi/2 # angular velocoty limit for controller

##### SCRIPT DIRECTORIES
DAT_DIR = Path('data') # data directory
FIG_DIR = Path('figs') # figure directory

LOC_DIR = None # local execution directory

##### PROBLEM PARAMETERS (A. static formation, C. proximity-based communication)
a_distance = False # (true for distance, false for displacement)
b_leader = True # (true for leader, false for global)
c_proximity = False # (true for proxmimity, false for all)

ac_N = 8 # number of robots
ac_iterations = 2000 # simulation iterations, each = 0.033 seconds
ac_formation_gain = 12.0 # formation gain to drive towards formation quicker

b_N = 4 # number of robots
b_iterations = 2000 # simulation iterations, each = 0.33 seconds
b_formattion_gain = 12.0 # formation gain to drive towards formation quicker
b_leader_velocity_limit = 0.1 # velocity limit for leader robot
b_waypoints = np.array([ # corner waypoints (forms a square)
    [-1.0, 0.5],
    [-1.0, -0.5],
    [1.0, -0.5],
    [1.0, 0.5]
])
b_waypoint_tol = 0.05 # (m) tolerance to say if in waypoint

c_radius = 1.0 # (m) proximity communication radius

a_dist_sim_name = 'a_dist_sim.npy' # simulated distance file name
a_disp_sim_name = 'a_disp_sim.npy' # simulated displacement file name
a_dist_real_name = 'a_dist_real.npy' # real distance file name
a_disp_real_name = 'a_disp_real.npy' # real displacement file name

b_global_sim_name = 'b_global_sim.npy' # simulated global motion file name
b_global_sim_ref_name = 'b_global_sim_ref.npy' # simulated global reference pose
b_global_sim_way_name = 'b_global_sim_way.npy' # simulated global waypoint info
b_global_sim_avg_name = 'b_global_sim_avg.npy' # simulated global average pose
b_leader_sim_name = 'b_leader_sim.npy' # simulated leader motion file name
b_global_real_name = 'b_global_real.npy' # real global motion file name
b_global_real_ref_name = 'b_global_real_ref.npy' # real global reference pose
b_global_real_way_name = 'b_global_real_way.npy' # real global waypoint info
b_global_real_avg_name = 'b_global_real_avg.npy' # real global average pose
b_leader_real_name = 'b_leader_real.npy' # real leader motion file name

c_dist_sim_name = 'c_dist_sim.npy' # simulated distance file name (proximity)
c_dist_real_name = 'c_dist_real.npy' # real distance file name (proximity)

ac_d = 0.4 # (m)
ac_dmatrix = np.array([ # shape matrix (forms a triangular formation for distance-based)
    [0.0, ac_d, 0.0, np.sqrt(2.0)*ac_d, 2*ac_d, np.sqrt(2.0)*ac_d, 0.0, ac_d],
    [ac_d, 0.0, ac_d, ac_d, 0.0, 0.0, 0.0, 0.0],
    [0.0, ac_d, 0.0, np.sqrt(2.0)*ac_d, 0.0, 0.0, 4*ac_d, 0.0],
    [np.sqrt(2.0)*ac_d, ac_d, np.sqrt(2.0)*ac_d, 0.0, np.sqrt(2.0)*ac_d, 0.0, 0.0, 0.0],
    [2*ac_d, 0.0, 0.0, np.sqrt(2.0)*ac_d, 0.0, np.sqrt(2.0)*ac_d, 0.0, 0.0],
    [np.sqrt(2.0)*ac_d, 0.0, 0.0, 0.0, np.sqrt(2.0)*ac_d, 0.0, np.sqrt(2.0)*ac_d, ac_d],
    [0.0, 0.0, 4*ac_d, 0.0, 0.0, np.sqrt(2.0)*ac_d, 0.0, ac_d],
    [ac_d, 0.0, 0.0, 0.0, 0.0, ac_d, ac_d, 0.0]
])

b_d = 0.6 # (m)
b_dmatrix = np.array([ # shape matrix (forms a square with leader at center)
    [0.0, np.sqrt(2.0)*b_d/2, np.sqrt(2.0)*b_d/2, np.sqrt(2.0)*b_d/2, np.sqrt(2.0)*b_d/2],
    [np.sqrt(2.0)*b_d/2, 0.0, b_d, np.sqrt(2.0)*b_d, b_d],
    [np.sqrt(2.0)*b_d/2, b_d, 0.0, b_d, np.sqrt(2.0)*b_d],
    [np.sqrt(2.0)*b_d/2, np.sqrt(2.0)*b_d, b_d, 0.0, b_d],
    [np.sqrt(2.0)*b_d/2, b_d, np.sqrt(2.0)*b_d, b_d, 0.0]
])

ac_disp = np.array([ # final robot position tuple (forms a triangular formation for displacement-based)
    [0.0, 0.0],
    [ac_d, 0.0],
    [2*ac_d, 0.0],
    [ac_d, ac_d],
    [0.0, 2*ac_d],
    [-ac_d, ac_d],
    [-2*ac_d, 0.0],
    [-ac_d, 0.0]
])

b_disp = np.array([ # final robot positions relative to waypoint coordinates
    [-b_d/2, b_d/2],
    [b_d/2, b_d/2],
    [b_d/2, -b_d/2],
    [-b_d/2, -b_d/2]
])

ac_initial = np.array([ # initial robot conditions for reproducability
    [-1.2, 0.3, np.pi],
    [-0.3, 0.3, np.pi/2],
    [0.3, 0.3, np.pi/2],
    [1.2, 0.3, 0.0],
    [-1.2, -0.3, np.pi],
    [-0.3, -0.3, -np.pi/2],
    [0.3, -0.3, -np.pi/2],
    [1.2, -0.3, 0.0]
])

b_initial = np.array([
    [-0.3, 0.0, 0.0],
    [0.0, 0.6, np.pi/2],
    [0.0, 0.3, np.pi/4],
    [0.0, -0.3, -np.pi/4],
    [0.0, -0.6, -np.pi/2]
])
