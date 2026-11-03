##### IMPORTS
import numpy as np
import scipy.linalg as la

##### SELF-DEFINED IMPORTS
import config
from utils.measure import MeasureL2Dist

####R COMMUNICATION NEIGHBORHOOD FUNCTION (inputs robot coordinates and returns all robot neighborhoods and degree)
def GraphCommNeighborhood(coords_robots: tuple) -> tuple[list, list]:
    # create empty neighborhood list
    neighborhoods_comm = []
    comm_degrees = []

    # loop through every robot and find neighborhood
    for index_base, coords_base in enumerate(coords_robots): # for each robot
        robot_neighbors = [] # begin with no neighbors
        for index_out, coords_out in enumerate(coords_robots): # for each other robot
            if index_base != index_out: # if not assessing self
                if MeasureL2Dist(coords1=coords_base, coords2=coords_out) <= config.radius_comm: # if within range
                    robot_neighbors.append(index_out+1) # add to neighbors
        neighborhoods_comm.append(robot_neighbors) # add to final neigbor list
        comm_degrees.append(len(robot_neighbors)) # add to final degree list

    # return final list of neighborhoods and degrees
    return neighborhoods_comm, comm_degrees

##### GRAPH ADJACENCY FUNCTION (inputs robot coordinates and returns graph adjacency matrix)
def GraphAdjacency(coords_robots: tuple) -> np.array:
    # finding robot neighborhoods
    comm_neighbors, _ = GraphCommNeighborhood(coords_robots=coords_robots)

    # constructing adjacency matrix from neighborhoods
    A = np.zeros((len(coords_robots), len(coords_robots))) # empty matrix

    for row_idx, neighbors in enumerate(comm_neighbors): # for each robot
        neighbors_idx = np.array(neighbors) - 1 # find neighbor indices
        A[row_idx,neighbors_idx] = 1 # add to adjacency

    # return final adjacency matrix
    return A

##### GRAPH IN-DEGREE FUNCTION (inputs adjacency matrix A and returns in-degree matrix and vector)
def GraphInDegree(A: np.array) -> tuple[np.array, np.array]:
    # finding in-degree
    d = np.sum(A, axis=1) # sum of each row
    return np.diag(d), d # diagonal matrix and d

##### GRAPH METROPOLIS FUNCTION (inputs robot coordinates and returns graph metropolis weight matrix and eigenvalues)
def GraphMetropolis(coords_robots: tuple) -> tuple[np.array, np.array]:
    # finding robot neighborhoods and degree
    neighborhoods_comm, comm_degrees = GraphCommNeighborhood(coords_robots=coords_robots)

    # constructing metropolis weight matrix
    W = np.zeros((len(coords_robots), len(coords_robots))) # empty matrix

    for row_idx, neighbors in enumerate(neighborhoods_comm): # for each robot
        neighbors_idx = np.array(neighbors) - 1 # find neighbor indices
        degrees = [comm_degrees[idx] for idx in neighbors_idx] # find neighbor degrees
        max_degrees = [max(comm_degrees[row_idx], degree) for degree in degrees] # finding maximum degree for weights

        W[row_idx,neighbors_idx] = 1 / (1 + np.array(max_degrees)) # add off-diagonal metropolis weights
        W[row_idx,row_idx] = 1 - sum(W[row_idx,:]) # add on-diagonal metropolis weights

    # find eigenvalues of Laplacian
    eigvals, _, _ = la.eig(W, left=True, right=True)

    # return final metropolis weight matrix
    return W, eigvals