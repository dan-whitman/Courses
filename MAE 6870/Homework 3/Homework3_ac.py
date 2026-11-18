##### RPS IMPORTS
import rps.robotarium as robotarium
from rps.utilities.graph import topological_neighbors
from rps.utilities.transformations import create_si_to_uni_dynamics
from rps.utilities.barrier_certificates import create_si_barrier_certificate_with_boundary

##### OTHER IMPORTS
import numpy as np
from pathlib import Path

##### SELF-DEFINED IMPORTS
import config
from utils import GraphAdjacency, GraphLaplacian, GraphRigidity, GraphProximity

##### HOMEWORK SCRIPT (static formation control / proximity-based communication)
# resolving directories
config.LOC_DIR = Path(__file__).resolve().parent # find local directory
config.DAT_DIR = config.LOC_DIR / config.DAT_DIR # update data directory

# setting configuration
if config.c_proximity: config.a_distance = True # force to use distance if using proximity

# create initial conditions for robots
initial_conditions = config.ac_initial.T

# generating robotarium
if config.simulate: # if simulating
    r = robotarium.Robotarium(
        number_of_robots=config.ac_N, show_figure=True, initial_conditions=initial_conditions, sim_in_real_time=False, skip_initialization=True
    )
else: # if real experiment
    r = robotarium.Robotarium(
        number_of_robots=config.ac_N, show_figure=True, initial_conditions=initial_conditions, sim_in_real_time=True, skip_initialization=False
    )

# generating matrices for problem
A = GraphAdjacency(W=config.ac_dmatrix) # valid for static-formation
L = GraphLaplacian(W=config.ac_dmatrix) # valid for static formation
R, rank = GraphRigidity(W=config.ac_dmatrix, positions=config.ac_disp) # valid for static formation

# adding safety and control functions
si_barrier_cert = create_si_barrier_certificate_with_boundary()
si_to_uni_dyn = create_si_to_uni_dynamics(linear_velocity_gain=config.linear_velocity_gain, angular_velocity_limit=config.angular_velocity_limit)

# performing simulation
data_poses = [initial_conditions] # initial poses list

for t in range(config.ac_iterations): # for each time step
    poses = r.get_poses() # get all poses of robots
    velocities = np.zeros((2, config.ac_N)) # create empty velocity array

    if config.c_proximity: Ap = GraphProximity(poses=poses, radius=config.c_radius) # if proximity-based, find new adjacency

    # updating robot commands
    for i in range(config.ac_N): # for each robot
        if config.c_proximity: neighbors = topological_neighbors(L=Ap, agent=i) # find all neighbors for proximity Laplacian
        else: neighbors = topological_neighbors(L=L, agent=i) # find all neighbors for static Laplacian

        for j in neighbors: # for each neighbor
            if config.a_distance == True: # distance formation control
                a_distance = np.linalg.norm(poses[:2,i] - poses[:2,j]) # find current distance between i and j
                d_distance = config.ac_dmatrix[i,j] # find desired distance between i and j

                velocities[:,i] += config.ac_formation_gain * (a_distance**2 - d_distance**2) * (poses[:2,j] - poses[:2,i]) # add to velocity command

            else: # displacement formation control
                velocities[:,i] += config.ac_formation_gain * ((poses[:2,j] - config.ac_disp[j,:]) - (poses[:2,i] - config.ac_disp[i,:])) # add to velocity command

    # velocity saturation
    speeds = np.linalg.norm(velocities, axis=0) # find current speed of each robot
    speed_threshold = config.linear_velocity_limit * r.MAX_LINEAR_VELOCITY # determine threshold value
    threshold_idx = speeds > speed_threshold # find speeds that surpass threshold
        
    if np.any(threshold_idx): # if there are any to saturate
        velocities[:,threshold_idx] *= speed_threshold / speeds[threshold_idx] # apply speed reduction

    # safety and dynamics application
    velocities = si_barrier_cert(dxi=velocities, x=poses[:2,:]) # ensure there are no collisions
    inputs = si_to_uni_dyn(dxi=velocities, states=poses) # convert to proper control inputs

    # step forward simulation
    r.set_velocities(ids=np.arange(config.ac_N), velocities=inputs) # set control velocities
    r.step() # step forward time

    # collect data
    data_poses.append(poses) # add poses to array

# saving data
data_poses = np.stack(data_poses, axis=2) # convert to np array
if config.simulate: # if running simulation
    config.DAT_DIR.mkdir(parents=True, exist_ok=True) # create directory if doesn't exist

    if config.c_proximity: np.save(str(config.DAT_DIR / config.c_dist_sim_name), data_poses) # save file
    elif config.a_distance: np.save(str(config.DAT_DIR / config.a_dist_sim_name), data_poses) # save file
    else: np.save(str(config.DAT_DIR / config.a_disp_sim_name), data_poses) # save file
else: # if real
    if config.c_proximity: np.save(config.c_dist_real_name, data_poses) # save file
    elif config.a_distance: np.save(config.a_dist_real_name, data_poses) # save file
    else: np.save(config.a_disp_real_name, data_poses) # save file

# printing results
r.debug() # debug report

if config.simulate: # if running simulation
    print(f'\nDesired coordinates =')
    print(config.ac_disp)

    print(f'\nDesired distances = ')
    print(config.ac_dmatrix)

    print(f'\nAdjacency =')
    print(A)

    print(f'\nLaplacian =')
    print(L)

    print(f'\nRigidity shape = {np.shape(R)}, nodes = {config.ac_N}, rank = {rank}')

    print(f'\nProximity radius =', config.c_radius)