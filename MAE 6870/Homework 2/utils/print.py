##### IMPORTS
import numpy as np

##### SELF-DEFINED IMPORTS
import config
from utils.graph import GraphCommNeighborhood, GraphMetropolis, GraphInDegree

##### PRINT INIT FUNCTION (prepares terminal for printing)
def PrintInit() -> None:
    print(f'\n===== PRINTING INFORMATION =====\n')

##### GRAPH PRINT FUNCTION (prints A, W, L, and eigenvalues)
def PrintGraph() -> None:
    # finding robot degrees
    _, robot_degrees = GraphCommNeighborhood(coords_robots=config.coords_robots) # robot comm neighborhoods

    # finding weight matrix
    W, eigvals = GraphMetropolis(coords_robots=config.coords_robots) # metropolis matrix and eigenvalues

    _, dW = GraphInDegree(A=W) # showing double-stochastic
    _, dWT = GraphInDegree(A=W.T) # showing double-stochastic

    # printing everything
    print('robot degrees:', robot_degrees)
    print(f'\nW =', W) # weight matrix
    print('W eigenvalues:', np.real(eigvals))
    print(f'W row sums: {dW}, W col sums: {dWT}') # double-stochastic properties

##### PROBLEM 3 PRINT FUNCTION
def PrintErrors(max_max_error: float, max_error_idx_idx: tuple) -> None:
    # printing everything
    print(f'\nMaximum cell error: {max_max_error} at ({max_error_idx_idx[0]}, {max_error_idx_idx[1]})')

##### PROBLEM 4/5 PRINT FUNCTION
def PrintMSE(MSE_all: tuple, MSE_shared: tuple, RMSE_shared: tuple, problem: int) -> None:
    # printing everything
    print(f'\nProblem {problem}:')
    print(f'MSE all weighted = {MSE_all[0]}, MSE all unweighted = {MSE_all[1]}')
    print(f'MSE shared weighted = {MSE_shared[0]}, MSE shared unweighted = {MSE_shared[1]}')
    print(f'RMSE shared weighted = {RMSE_shared[0]}, MSE all unweighted = {RMSE_shared[1]}')
