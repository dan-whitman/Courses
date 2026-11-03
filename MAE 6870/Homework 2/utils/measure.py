##### IMPORTS
import numpy as np

##### SELF-DEFINED IMPORTS
import config

##### REAL TEMPERATURE FUNCTION (inputs grid and returns real temperature at each cell)
def MeasureTempReal(grid: np.array) -> np.array:
    # find grid size and flatten
    rows, cols = np.shape(grid)
    grid_flat = np.reshape(grid, -1)

    # creating grid of temperature values and reshaping to return
    grid_temp_real = np.array([config.temp_unheated + config.temp_unheated * cell for cell in grid_flat])
    return np.reshape(grid_temp_real, (rows,cols))

##### L2 NORM DISTANCE FUNCTION (inputs two coords on grid and returns their l2 distance)
def MeasureL2Dist(coords1: tuple, coords2: tuple) -> float:
    # scaling coords by grid size
    coords1 = config.grid_distance * np.array(coords1)
    coords2 = config.grid_distance * np.array(coords2)

    # finding l2 distance
    return np.linalg.norm(coords1 - coords2)

##### SENSING NEIGHBORHOOD FUNCTION (inputs robot coords and returns its neighborhood on grid)
def MeasureSenseNeighborhood(coords_robot: tuple, grid: np.array) -> np.array:
    # find grid size
    rows, cols = np.shape(grid)

    # find distance to every cell in grid
    grid_dist = [MeasureL2Dist(coords1=coords_robot, coords2=(row_idx,col_idx)) for row_idx, row in enumerate(grid) for col_idx, _ in enumerate(row)]

    # create neighborhood grid
    grid_neighborhood = np.array([1.0 if dist <= config.radius_sensing else 0.0 for dist in grid_dist])

    # return and reshape result
    return np.reshape(grid_neighborhood, (rows,cols))

##### CELL MEASURE SET FUNCTION (inputs graph sense neighborhoods and returns all/number of robots that can measure each cell)
def MeasureCellSet(neighborhoods_sense: list) -> tuple[list, np.array]:
    # find grid size and flatten all grids
    rows, cols = np.shape(neighborhoods_sense[0])
    neighborhoods_flat = [np.reshape(neighborhood_sense, -1) for neighborhood_sense in neighborhoods_sense]

    # creating all cell measurement sets
    cell_sets = [[] for _ in range(rows * cols)] # empty cell set
    for robot_idx, neighborhood_flat in enumerate(neighborhoods_flat): # for each robot's sensing neighborhood
        for cell_idx, cell in enumerate(neighborhood_flat): # for each cell in robot's neighborhood
            if cell == 1: cell_sets[cell_idx].append(robot_idx + 1) # add robot to sensing

    # finding number of robots for each cell
    cell_nums = np.array([len(cell_set) for cell_set in cell_sets])

    # reshape result and return
    cell_sets = [cell_sets[rows*i:cols+rows*i] for i in range(rows)]
    cell_nums = np.reshape(cell_nums, (rows,cols))
    return cell_sets, cell_nums

##### CELL MEASUREMENT FUNCTION (inputs true cell temperature theta and robot uncertainty sigma and returns a meaasurement)
def MeasureCell(theta: float, sigma: float) -> float:
    # define error for measurement
    epsilon = config.rng.normal(loc=0, scale=sigma) # pull from normal N(0,sigma)

    # return measurement
    return theta + epsilon

##### MEASUREMENT ERROR FUNCTION (inputs temperature array for a robot and returns the maximum error value and index)
def MeasureError(robot_temperature: np.array, grid_centralized: np.array) -> tuple[float, tuple]:
    # find difference across all cells
    error = np.abs(robot_temperature - grid_centralized) # error matrix

    # find maximum error and index
    max_error = np.max(error) # error value
    max_error_idx = np.unravel_index(np.argmax(error), np.shape(error)) # error index

    # returning value
    return max_error, max_error_idx

##### CENTRALIZED FORMULA MEASUREMENT FUNCTION (inputs robot measurements and variances and outputs expected result for each cell)
def MeasureCentralized(raw_measures: list, vars_robot: tuple, cell_sets: list) -> np.array:
    ## find size of grid and initialize
    rows, cols = np.shape(raw_measures[0])
    grid_centralized = np.zeros((rows, cols)) # centralized grid
    omega = np.zeros((rows, cols)) # omega grid

    # finding centralized value for each cell
    for robot_idx, raw_measure in enumerate(raw_measures): # for each robot
        for row_idx, row in enumerate(raw_measure): # for each row
            for col_idx, measure in enumerate(row): # for each measurement
                if robot_idx+1 in cell_sets[row_idx][col_idx]: # robot is in cell's measure set
                    grid_centralized[row_idx, col_idx] = grid_centralized[row_idx, col_idx] + measure / vars_robot[robot_idx] # add to expected value of cell
                    omega[row_idx, col_idx] = omega[row_idx, col_idx] + 1 / vars_robot[robot_idx] # add to cell's omega
    grid_centralized = grid_centralized / omega # normalize all cells

    # return expected grid
    return grid_centralized

##### SHARED SENSING MEASUREMENT FUNCTION (inputs sensing neighborhoods/cell sets and returns all cells shared with desired robot)
def MeasureSharedCells(cell_sets: list, neighborhood_sense: np.array) -> np.array:
    # constructing initial shared region
    grid_shared = np.zeros(np.shape(neighborhood_sense))

    # going through each cell and seeing if it is shared
    for row_idx, row in enumerate(neighborhood_sense): # for each row
        for col_idx, cell in enumerate(row): # for each cell
            if cell == 1: # if cell is being measured by robot we care about 
                if len(cell_sets[row_idx][col_idx]) > 1: # if cell is shared
                    grid_shared[row_idx,col_idx] = 1 # add to shared grid

    # return final grid
    return grid_shared

##### MSE MEASUREMENT FUNCTION (inputs estimated temperatures and true temperatures and returns MSE for each cell and for desired region)
def MeasureMSE(robot_temperature: np.array, grid_temp_real: np.array, grid_shared: np.array) -> tuple[float, float, float]:
    # computing MSE of grid
    grid_MSE = (robot_temperature - grid_temp_real) ** 2 # total region
    grid_MSE_shared = grid_MSE * grid_shared # shared region

    # computing average MSE
    MSE_all = np.mean(grid_MSE) # total grid MSE
    MSE_shared = np.sum(grid_MSE_shared) / np.count_nonzero(grid_MSE_shared) # shared grid MSE
    RMSE_shared = MSE_shared ** 0.5 # shared RMSE

    # returning results
    return MSE_all, MSE_shared, RMSE_shared