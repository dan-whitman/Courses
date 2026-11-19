##### RPS IMPORTS
import rps.robotarium as robotarium
from rps.utilities.graph import topological_neighbors, completeGL
from rps.utilities.transformations import create_si_to_uni_dynamics
from rps.utilities.barrier_certificates import create_si_barrier_certificate_with_boundary
from rps.utilities.controllers import create_si_position_controller

##### OTHER IMPORTS
import numpy as np
from pathlib import Path

##### SELF-DEFINED IMPORTS
import config
from utils import PosesAverage

##### HOMEWORK SCRIPT (moving-reference formation control)
# resolving directories
config.LOC_DIR = Path(__file__).resolve().parent # find local directory
config.DAT_DIR = config.LOC_DIR / config.DAT_DIR # update data directory

# setting configuration
if config.b_leader: config.b_N += 1 # add leader robot

# create initial conditions for robots
if config.b_leader: initial_conditions = config.b_initial.T # if using a leader
else: initial_conditions = config.b_initial[1:,:].T # if all know plan

# generating robotarium
if config.simulate: # if simulating
    r = robotarium.Robotarium(
        number_of_robots=config.b_N, show_figure=True, initial_conditions=initial_conditions, sim_in_real_time=False, skip_initialization=True
    )
else: # if real experiment
    r = robotarium.Robotarium(
        number_of_robots=config.b_N, show_figure=True, initial_conditions=initial_conditions, sim_in_real_time=True, skip_initialization=False
    )

# creating graph matrices
if config.b_leader: # if using a leader
    L = np.zeros((config.b_N, config.b_N))
    Lf = -completeGL(n=config.b_N-1) # followers are fully connected
    L[1:config.b_N,1:config.b_N] = Lf # add to Laplacian
    L[1,1] += 1 # robot 0 receives commands from leader
    L[3,3] += 1 # robot 2 receives commands from leader
    L[1,0] = -1
    L[3,0] = -1

# adding safety and control functions
si_barrier_cert = create_si_barrier_certificate_with_boundary()
si_to_uni_dyn = create_si_to_uni_dynamics(linear_velocity_gain=config.linear_velocity_gain, angular_velocity_limit=config.angular_velocity_limit)

# adding motion controllers
leader_controller = create_si_position_controller(
    x_velocity_gain=config.linear_velocity_gain, y_velocity_gain=config.linear_velocity_gain, velocity_magnitude_limit=config.b_leader_velocity_limit
)

# performing simulation
data_poses = [initial_conditions] # initial poses list
data_average_pose = [PosesAverage(poses=initial_conditions)] # initial average poses list
waypoint_idx = 0 # waypoint index
data_waypoint_pose = [np.append(waypoint_idx, config.b_waypoints[waypoint_idx,:])] # initial waypoint list
if not config.b_leader: # if not using leader
    reference_pose = np.array([0.0, 0.0]) # initial reference pose (for global)
    data_reference_pose = [reference_pose] # initial reference pose data (for global)

for t in range(config.b_iterations): # for each time step
    poses = r.get_poses() # get all poses of robots
    velocities = np.zeros((2, config.b_N)) # create empty velocity array

    # updating robot commands
    if config.b_leader: # if using a leader
        for i in range(config.b_N): # for each robot
            neighbors = topological_neighbors(L=L, agent=i) # find all neighbors
            for j in neighbors: # for each neighbor
                a_distance = np.linalg.norm(poses[:2,i] - poses[:2,j]) # find current distance between i and j
                d_distance = config.b_dmatrix[i,j] # find desired distance between i and j

                velocities[:,i] += config.b_formattion_gain * (a_distance**2 - d_distance**2) * (poses[:2,j] - poses[:2,i]) # add to velocity command
    
        # update leader pose
        velocities[:,0] = leader_controller(states=poses[:2,0].reshape(2,1), poses=np.reshape(config.b_waypoints[waypoint_idx,:],(2,1))).flatten()

        # update waypoint state
        if np.linalg.norm(poses[:2,0] - config.b_waypoints[waypoint_idx,:]) < config.b_waypoint_tol:
            waypoint_idx = (waypoint_idx + 1) % config.b_waypoints.shape[0] # wrap waypoint index
    
    else: # if all know plan
        # find average formation position
        average = PosesAverage(poses=poses)

        for i in range(config.b_N): # for each robot
            velocities[:,i] = -config.b_formattion_gain * (poses[:2,i] - (config.b_disp[i,:] + reference_pose)) # add to velocity command

        # update reference pose
        reference_pose += 0.033 * leader_controller(states=reference_pose.reshape((2,1)), poses=np.reshape(config.b_waypoints[waypoint_idx,:],(2,1))).flatten()

        # update waypoint state
        if np.linalg.norm(reference_pose - config.b_waypoints[waypoint_idx,:]) < config.b_waypoint_tol:
            waypoint_idx = (waypoint_idx + 1) % config.b_waypoints.shape[0] # wrap waypoint index

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
    r.set_velocities(ids=np.arange(config.b_N), velocities=inputs) # set control velocities
    r.step() # step forward time

    # collect data
    data_poses.append(poses) # add poses to array
    data_waypoint_pose.append(np.append(waypoint_idx, config.b_waypoints[waypoint_idx,:])) # add waypoint idx to array
    if not config.b_leader: # if not using leader
        data_average_pose.append(average) # add average to array
        data_reference_pose.append(np.copy(reference_pose)) # add reference pose to array

# saving data
data_poses = np.stack(data_poses, axis=2) # convert to np array
data_waypoint_pose = np.array(data_waypoint_pose) # convert to np array
if not config.b_leader: # if not using leader
    data_average_pose = np.array(data_average_pose) # convert to np array
    data_reference_pose = np.array(data_reference_pose) # convert to np array
if config.simulate: # if running simulation
    config.DAT_DIR.mkdir(parents=True, exist_ok=True) # create directory if doesn't exist

    if config.b_leader: # if using leader
        np.save(str(config.DAT_DIR / config.b_leader_sim_name), data_poses) # save file
    else: # if all know plan
        np.save(str(config.DAT_DIR / config.b_global_sim_name), data_poses) # save file
        np.save(str(config.DAT_DIR / config.b_global_sim_avg_name), data_average_pose) # save file
        np.save(str(config.DAT_DIR / config.b_global_sim_way_name), data_waypoint_pose) # save file
        np.save(str(config.DAT_DIR / config.b_global_sim_ref_name), data_reference_pose) # save file
else: # if real
    if config.b_leader: # if using leader
        np.save(config.b_leader_real_name, data_poses) # save file
    else: # if all know plan
        np.save(config.b_global_real_name, data_poses) # save file
        np.save(config.b_global_real_avg_name, data_average_pose) # save file
        np.save(config.b_global_real_way_name, data_waypoint_pose) # save file
        np.save(config.b_global_real_ref_name, data_reference_pose) # save file

# printing results
r.debug() # debug report
