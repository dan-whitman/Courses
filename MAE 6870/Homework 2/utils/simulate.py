##### IMPORTS
import numpy as np
import copy

##### SELF-DEFINED IMPORTS
from utils.measure import MeasureCell, MeasureMSE

##### CONSENSUS SIMULATION FUNCTION (inputs all robot measurements and performes K steps of consensus)
def SimulateConsensus(robot_measures_w: list, robot_measures_u: list, W: np.array, K: float) -> tuple[list, list]:
    # constructing initial sets
    robot_consensuses_w = [] # empty consensus step set
    robot_consensuses_u = [] # empty consensus step set
    robot_consensuses_w.append(copy.deepcopy(robot_measures_w)) # initial state
    robot_consensuses_u.append(copy.deepcopy(robot_measures_u)) # initial state

    # performing consensus
    for _ in range(K): # for each consensus step
        robot_measures_tmp_w = copy.deepcopy(robot_measures_w) # create copy to alter
        robot_measures_tmp_u = copy.deepcopy(robot_measures_u) # create copy to alter
        for robot_idx, (robot_measure_w, robot_measure_u) in enumerate(zip(robot_measures_w, robot_measures_u)): # for each robot
            for row_idx, (row_w, row_u) in enumerate(zip(robot_measure_w, robot_measure_u)): # for each row
                for col_idx, (_, _) in enumerate(zip(row_w, row_u)): # for each cell
                    airc_w = 0.0 # initial cell consensus numerator
                    airc_u = 0.0 # initial cell consensus numerator
                    birc_w = 0.0 # initial cell consensus denominator
                    birc_u = 0.0 # initial cell consensus denominator
                    for other_robot_idx, (other_robot_measure_w, other_robot_measure_u) in enumerate(zip(robot_measures_w, robot_measures_u)): # for each other robot
                        airc_w = airc_w + W[robot_idx, other_robot_idx] * other_robot_measure_w[row_idx][col_idx][0] # consensus on airc
                        airc_u = airc_u + W[robot_idx, other_robot_idx] * other_robot_measure_u[row_idx][col_idx][0] # consensus on airc
                        birc_w = birc_w + W[robot_idx, other_robot_idx] * other_robot_measure_w[row_idx][col_idx][1] # consensus on birc    
                        birc_u = birc_u + W[robot_idx, other_robot_idx] * other_robot_measure_u[row_idx][col_idx][1] # consensus on birc                                  
                    robot_measures_tmp_w[robot_idx][row_idx][col_idx] = (airc_w, birc_w) # new measurement
                    robot_measures_tmp_u[robot_idx][row_idx][col_idx] = (airc_u, birc_u) # new measurement
        robot_consensuses_w.append(robot_measures_tmp_w) # update state
        robot_consensuses_u.append(robot_measures_tmp_u) # update state
        robot_measures_w = robot_measures_tmp_w # reassign measures
        robot_measures_u = robot_measures_tmp_u # reassign measures

    # return consensus data
    return robot_consensuses_w, robot_consensuses_u

##### TEMPERATURE ESTIMATE FUNCTION (inputs consensus estimates and returns temperature estimates for each robot)
def SimulateTemperatues(robot_consensuses_w: list, robot_consensuses_u: list) -> tuple[list, list]:
    # constructing initial sets
    robot_temperatures_w = []
    robot_temperatures_u = []

    # finding robot temperature estimates
    for robot_measures_w, robot_measures_u in zip(robot_consensuses_w, robot_consensuses_u): # for each step of consensus
        robot_temperatures_step_w = [] # initially empty
        robot_temperatures_step_u = [] # initially empty
        for robot_measure_w, robot_measure_u in zip(robot_measures_w, robot_measures_u): # for each robot
            robot_temperature_w = [] # initially empty
            robot_temperature_u = [] # initially empty
            for row_w, row_u in zip(robot_measure_w, robot_measure_u): # for each row
                row_temperature_w = [] # initially empty
                row_temperature_u = [] # initially empty
                for measure_w, measure_u in zip(row_w, row_u): # for each cell
                    if measure_w[1] > 0: # if birc > 0
                        row_temperature_w.append(measure_w[0] / measure_w[1]) # estimate temperature
                        row_temperature_u.append(measure_u[0] / measure_u[1]) # estimate temperature
                    else:
                        row_temperature_w.append(np.nan) # unknown estimate
                        row_temperature_u.append(np.nan) # unknown estimate
                robot_temperature_w.append(row_temperature_w) # add row temperature to estimate
                robot_temperature_u.append(row_temperature_u) # add row temperature to estimate
            robot_temperatures_step_w.append(np.array(robot_temperature_w)) # add robot to step
            robot_temperatures_step_u.append(np.array(robot_temperature_u)) # add robot to step
        robot_temperatures_w.append(robot_temperatures_step_w) # add step to list
        robot_temperatures_u.append(robot_temperatures_step_u) # add step to list

    # return final estimates
    return robot_temperatures_w, robot_temperatures_u

##### SIMULATE INITIAL MEASURMENT FUNCTION (constructs initial zero filled lists for robot prior measurements)
def SimulateInitMeasure(grid_temp: np.array, vars_robot: tuple) -> list:
    # initial empty set
    robot_measures_prev = [] # empty measurements set

    # filling zero for all cells for all robots
    for _ in enumerate(vars_robot): # for each robot
        robot_measure_prev = [] # initially empty
        for row in grid_temp: # for each row
            row_measure_prev = [] # initially empty
            for _ in row: # for each cell
                row_measure_prev.append((0.0, 0.0)) # append (0.0, 0.0)
            robot_measure_prev.append(row_measure_prev) # add row to array
        robot_measures_prev.append(robot_measure_prev) # add robot to array

    # return final empty measurement array
    return robot_measures_prev

##### SINGLE MEASUREMENT SIMULATION FUNCTION (inputs temperature grid, cell sets, robot variances, and consensus steps and returns estimated grids for each robot)
def SimulateSingle(grid_temp_real: np.array, W: np.array, cell_sets: list, vars_robot: tuple, K: float, robot_measures_prev_w: list, robot_measures_prev_u: list) -> tuple[list, list, list]:
    # constructing initial sets
    robot_measures_w = [] # empty measurements set
    robot_measures_u = [] # empty measurements set
    raw_measures = [] # empty raw measurements set

    # taking measurement of all cells for all robots
    for robot_idx, var_robot in enumerate(vars_robot): # for each robot
        robot_measure_w = [] # initally empty
        robot_measure_u = [] # initally empty
        raw_robot_measure = [] # initially empty
        for row_idx, row in enumerate(grid_temp_real): # for each row
            row_measure_w = [] # initially empty
            row_measure_u = [] # initially empty
            raw_row_measure = [] # initially empty
            for col_idx, temp in enumerate(row): # for each cell
                airc_w = robot_measures_prev_w[robot_idx][row_idx][col_idx][0] # merge with previous measurement
                airc_u = robot_measures_prev_u[robot_idx][row_idx][col_idx][0] # merge with previous measurement
                birc_w = robot_measures_prev_w[robot_idx][row_idx][col_idx][1] # merge with previous measurement
                birc_u = robot_measures_prev_u[robot_idx][row_idx][col_idx][1] # merge with previous measurement
                if robot_idx+1 in cell_sets[row_idx][col_idx]: # robot is in cell's measure set
                    measure = MeasureCell(theta=temp, sigma=var_robot**0.5) # get measurement
                    airc_w = airc_w + measure / var_robot # set numerator
                    airc_u = airc_u + measure # set numerator
                    birc_w = birc_w + 1.0 / var_robot # set denominator
                    birc_u = birc_u + 1.0 # set denominator
                else:
                    measure = 0.0
                    airc_w = airc_w + 0.0 # set numerator
                    airc_u = airc_u + 0.0 # set numerator
                    birc_w = birc_w + 0.0 # set denominator
                    birc_u = birc_u + 0.0 # set denominator
                row_measure_w.append((airc_w, birc_w)) # add measurements to array
                row_measure_u.append((airc_u, birc_u)) # add measurements to array
                raw_row_measure.append(measure) # add raw measurement to array
            robot_measure_w.append(row_measure_w) # add row to array
            robot_measure_u.append(row_measure_u) # add row to array
            raw_robot_measure.append(raw_row_measure) # add row to array
        robot_measures_w.append(robot_measure_w) # add robot to array
        robot_measures_u.append(robot_measure_u) # add robot to array
        raw_measures.append(np.array(raw_robot_measure)) # add robot to array

    # perform consensus
    robot_consensuses_w, robot_consensuses_u = SimulateConsensus(robot_measures_w=robot_measures_w, robot_measures_u=robot_measures_u, W=W, K=K)

    # return final estimated measurements
    return robot_consensuses_w, robot_consensuses_u, raw_measures

##### MSE TRIAL FUNCTION (performs trial to compute MSE over trials, returns averaged MSE metrics and final temperature grid for plotting)
def SimulateMSETrials(grid_temp_real: np.array, W: np.array, cell_sets: list, vars_robot: tuple, K: float, M: float, grid_shared: np.array, robot_measures_prev_w: list, robot_measures_prev_u: list) -> tuple[tuple, tuple, tuple, np.array, np.array]:
    # initialize values
    MSE_all_avg_w = 0 # total grid MSE
    MSE_all_avg_u = 0 # total grid MSE
    MSE_shared_avg_w = 0 # shared grid MSE
    MSE_shared_avg_u = 0 # shared grid MSE
    RMSE_shared_avg_w = 0 # shared grid RMSE
    RMSE_shared_avg_u = 0 # shared grid RMSE

    # performing simulation trials
    for _ in range(M): # for each trial
        robot_consensuses_w, robot_consensuses_u, _ = SimulateSingle(grid_temp_real=grid_temp_real, W=W, cell_sets=cell_sets, vars_robot=vars_robot, K=K, robot_measures_prev_w=robot_measures_prev_w, robot_measures_prev_u=robot_measures_prev_u) # perform simulation
        robot_temperatures_w, robot_temperatures_u = SimulateTemperatues(robot_consensuses_w=robot_consensuses_w, robot_consensuses_u=robot_consensuses_u) # convert to temperatures
        robot_consensus_final_w = robot_consensuses_w[-1] # take final consensus estimate
        robot_consensus_final_u = robot_consensuses_u[-1] # take final consensus estimate        
        robot_temperature_final_w = robot_temperatures_w[-1] # take final temperature estimate
        robot_temperature_final_u = robot_temperatures_u[-1] # take final temperature estimate
        MSE_all_w, MSE_shared_w, RMSE_shared_w = MeasureMSE(robot_temperature=robot_temperature_final_w[0], grid_temp_real=grid_temp_real, grid_shared=grid_shared) # find trial errors
        MSE_all_u, MSE_shared_u, RMSE_shared_u = MeasureMSE(robot_temperature=robot_temperature_final_u[0], grid_temp_real=grid_temp_real, grid_shared=grid_shared) # find trial errors
        MSE_all_avg_w = MSE_all_avg_w + MSE_all_w # add to total
        MSE_all_avg_u = MSE_all_avg_u + MSE_all_u # add to total
        MSE_shared_avg_w = MSE_shared_avg_w + MSE_shared_w # add to total
        MSE_shared_avg_u = MSE_shared_avg_u + MSE_shared_u # add to total
        RMSE_shared_avg_w = RMSE_shared_avg_w + RMSE_shared_w # add to total
        RMSE_shared_avg_u = RMSE_shared_avg_u + RMSE_shared_u # add to total

    # averaging results
    MSE_all_avg_w = MSE_all_avg_w / M 
    MSE_all_avg_u = MSE_all_avg_u / M 
    MSE_shared_avg_w = MSE_shared_avg_w / M
    MSE_shared_avg_u = MSE_shared_avg_u / M
    RMSE_shared_avg_w = RMSE_shared_avg_w / M
    RMSE_shared_avg_u = RMSE_shared_avg_u / M

    # returning results
    return (MSE_all_avg_w, MSE_all_avg_u), (MSE_shared_avg_w, MSE_shared_avg_u), (RMSE_shared_avg_w, RMSE_shared_avg_u), robot_consensus_final_w, robot_consensus_final_u, robot_temperature_final_w, robot_temperature_final_u

##### SEQUENTIAL SIMULATION FUNCTION
def SimulateSequential(grid_temp_real: np.array, W: np.array, cell_sets: tuple, vars_robot: tuple, K: float, M: float, grid_shared: np.array, t: float) -> tuple[list, list, list, list]:
    # create initial arrays
    MSE_all_seq = []
    MSE_shared_seq = []
    robot_temperatures_w_seq = []
    robot_temperatures_u_seq = []

    # create initial zero guesses
    robot_measures_prev_w = SimulateInitMeasure(grid_temp=grid_temp_real, vars_robot=vars_robot) # weighted initial
    robot_measures_prev_u = SimulateInitMeasure(grid_temp=grid_temp_real, vars_robot=vars_robot) # unweighted initial

    # perform simulation trials
    for ele in range(t): # for each time step
        print(f't = {ele}')

        # calculate the MSE for that time step and final consensus state for each robot
        MSE_all_t, MSE_shared_t, _, robot_measures_prev_w_t, robot_measures_prev_u_t, robot_temperatures_w_t, robot_temperatures_u_t = SimulateMSETrials(grid_temp_real=grid_temp_real, W=W, cell_sets=cell_sets, vars_robot=vars_robot, K=K, M=M, grid_shared=grid_shared, robot_measures_prev_w=robot_measures_prev_w, robot_measures_prev_u=robot_measures_prev_u)

        # append to data and copy new previous measurement states
        MSE_all_seq.append(MSE_all_t)
        MSE_shared_seq.append(MSE_shared_t)

        robot_measures_prev_w = robot_measures_prev_w_t
        robot_measures_prev_u = robot_measures_prev_u_t

        robot_temperatures_w_seq.append(robot_temperatures_w_t)
        robot_temperatures_u_seq.append(robot_temperatures_u_t)

    # returning results
    return MSE_all_seq, MSE_shared_seq, robot_temperatures_w_seq, robot_temperatures_u_seq