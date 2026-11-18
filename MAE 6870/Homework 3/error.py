##### RPS IMPORTS

##### OTHER IMPORTS
import numpy as np

##### SELF-DEFINED IMPORTS
from utils import GraphAdjacency

##### ERROR DISTANCE CONSTRAINT FUNCTION (inputs distance matrix and agent positions and returns error)
def ErrorDistanceConstraint(W: np.ndarray, poses: np.ndarray) -> float:
    # finding number of graph edges and nodes
    # finding number graph nodes
    N, _ = np.shape(W)
    A = GraphAdjacency(W=W) # gives 1 where there is an edge between nodes
    E = int(np.sum(A) / 2) # sum all 1s and divide by 2 for double counting

    # calculating the error
    error = 0 # initially 0 error
    for i in range(N): # for each robot 
        for j in range(i + 1, N): # for each other robot
            if A[i,j] != 0: # if there is an edge
                a_distance = np.linalg.norm(poses[:2,i] - poses[:2,j]) # find current distance between i and j
                d_distance = W[i,j] # find desired distance between i and j

                error += (a_distance - d_distance) ** 2 # add to error sum

    return np.sqrt(error / E) # return final error

##### SQUARE ERROR FUNCTION (returns square error between sim and real data)
def ErrorSquare(sim_poses: np.ndarray, real_poses: np.ndarray) -> float:
    sim_poses = sim_poses.T # transpose data for easier parsing
    real_poses = real_poses.T # transpose data for easier parsing

    # calculating the error
    error = 0 # initially 0 error
    for sim_pose, real_pose in zip(sim_poses, real_poses): # for each agent's position
        error = error + np.linalg.norm(sim_pose[:2] - real_pose[:2]) ** 2

    return error # return final square error

##### REFERENCE ERROR FUNCTION (returns square error between reference pose and waypoint pose)
def ErrorReference(average_pose: np.ndarray, waypoint_pose: np.ndarray) -> float:
    # return error
    return ErrorSquare(sim_poses=np.array([average_pose]), real_poses=np.array([waypoint_pose[1:]]))