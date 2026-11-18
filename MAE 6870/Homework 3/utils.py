##### RPS IMPORTS

##### OTHER IMPORTS
import numpy as np

##### SELF-DEFINED IMPORTS

##### GRAPH ADJACENCY FUNCTION (generates graph adjacency matrix for given weight matrix)
def GraphAdjacency(W: np.ndarray) -> np.ndarray:
    # creating empty array
    A = np.zeros(np.shape(W))

    # construc adjacency
    for row_idx, row in enumerate(W): # for every row
        for col_idx, ele in enumerate(row): # for every column
            if ele != 0: A[row_idx,col_idx] = 1 # set adjacency weight

    # return A
    return A

##### GRAPH IN-DEGREE FUNCTION (inputs graph adjacency and returns in degree matrix)
def GraphInDegree(A: np.ndarray) -> np.ndarray:
    # sum over all rows and return diagonal matrix
    return np.diag(np.sum(A, axis=1))

##### GRAPH LAPLACIAN FUNCTION (inputs distance weight matrix and generates proper graph Laplacian)
def GraphLaplacian(W: np.ndarray) -> np.ndarray:
    # creating adjacency matrix
    A = GraphAdjacency(W=W)

    # finding indegree matrix
    D = GraphInDegree(A=A)

    # return L
    return D - A

##### GRAPH-PROXIMITY FUNCTION (inputs robot current positions and communication radius and generates proximity-based Laplacian matrix)
def GraphProximity(poses: np.ndarray, radius: float) -> np.ndarray:
    # create empty adjacency matrix
    _, N = np.shape(poses) # find number of robots
    Ap = np.zeros((N, N)) # empty matrix

    # constructing proximity adjacency
    for i in range(N): # for each robot
        for j in range(i+1, N): # for each other robot
            a_distance = np.linalg.norm(poses[:2,i] - poses[:2,j]) # find current distance between i and j
            if a_distance < radius: # if robots are within proximity of each other
                Ap[i,j] = 1 # update element
                Ap[j,i] = 1 # update symmetric element

    # return adjacency
    return Ap

##### GRAPH RIGIDITY FUNCTION (inputs weight matrix and desired positions and returns rigidity matrix and rank)
def GraphRigidity(W: np.ndarray, positions: np.ndarray) -> tuple[np.ndarray, float]:
    # finding number graph nodes
    N, _ = np.shape(W)

    # finding number of graph edges
    A = GraphAdjacency(W=W) # gives 1 where there is an edge between nodes
    E = int(np.sum(A) / 2) # sum all 1s and divide by 2 for double counting

    # create rigidity matrix
    R = [] # empty array
    for i in range(N): # for each robot
        for j in range(i + 1, N): # for each other robot beyond i
            if A[i,j] != 0: # if there is an edge
                R_row = np.zeros(2 * N) # create row of all 0s

                diff = positions[i,:] - positions[j,:] # find distance between positions

                # update row elements
                R_row[2*i] = diff[0]
                R_row[2*i + 1] = diff[1]
                R_row[2*j] = -diff[0]
                R_row[2*j + 1] = -diff[1]

                R.append(R_row) # append to final rigidity matrix

    R = np.array(R) # transform into an array
    rank = np.linalg.matrix_rank(R) # find rank

    # return results
    return R, rank

##### AVERAGE POSITION FUNCTION
def PosesAverage(poses: np.ndarray) -> np.ndarray:
    # find number of robots
    _, N = np.shape(poses)

    # find average x and y positions
    x = np.sum(poses[0,:]) / N
    y = np.sum(poses[1,:]) / N

    # return value
    return np.array([x, y])