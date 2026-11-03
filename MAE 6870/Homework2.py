##### IMPORTS
import numpy as np

##### SELF-DEFINED IMPORTS
import config
from utils.imports import ImportCSV
from utils.measure import MeasureTempReal, MeasureSenseNeighborhood, MeasureCellSet, MeasureError, MeasureCentralized, MeasureSharedCells
from utils.graph import GraphAdjacency, GraphMetropolis
from utils.visual import VisualGraphNetwork, VisualShow, VisualTempGrid, VisualXij, VisualErrorGrid, VisualMSE
from utils.simulate import SimulateSingle, SimulateMSETrials, SimulateInitMeasure, SimulateSequential, SimulateTemperatues
from utils.print import PrintInit, PrintGraph, PrintErrors, PrintMSE
from utils.save import SaveFigures

##### HOMEWORK SCRIPT
if __name__ == '__main__':
    ##### IMPORTING GRID
    grid_raw = ImportCSV(filepath=str(config.DAT_DIR / 'room_2x2.csv')) # raw grid
    grid_temp_real = MeasureTempReal(grid=grid_raw) # true temperature grid

    ##### CONSTRUCTING NETWORK
    W, _ = GraphMetropolis(coords_robots=config.coords_robots) # consensus weights

    ##### DEFINING NEIGHBORHOODS
    neighborhoods_sense = [MeasureSenseNeighborhood(coords_robot=coords_robot, grid=grid_raw) for coords_robot in config.coords_robots] # robot sensing neighborhoods

    ##### DEFINING MEASUREMENT SETS
    cell_sets, cell_nums = MeasureCellSet(neighborhoods_sense=neighborhoods_sense)

    ##### SIMULATING SINGLE MEASUREMENT (PROBLEM 3)
    print(f'\n=====PROBLEM 3=====')
    vars_robot = (25.0,) * config.num_robots # robot variances
    K = 100 # consensus steps

    # initial zero previous measurements
    robot_measures_prev_w = SimulateInitMeasure(grid_temp=grid_temp_real, vars_robot=vars_robot)
    robot_measures_prev_u = SimulateInitMeasure(grid_temp=grid_temp_real, vars_robot=vars_robot)

    # simulating a measurement
    robot_consensuses_3, _, raw_measures_3 = SimulateSingle(grid_temp_real=grid_temp_real, W=W, cell_sets=cell_sets, vars_robot=vars_robot, K=K, robot_measures_prev_w=robot_measures_prev_w, robot_measures_prev_u=robot_measures_prev_u) # single measurement simulation        
    robot_temperatures_3, _ = SimulateTemperatues(robot_consensuses_w=robot_consensuses_3, robot_consensuses_u=robot_consensuses_3) # convert to temperatures
    grid_centralized = MeasureCentralized(raw_measures=raw_measures_3, vars_robot=vars_robot, cell_sets=cell_sets) # find centralized prediction

    # finding errors
    max_errors = [MeasureError(robot_temperature=robot_temperature, grid_centralized=grid_centralized)[0] for robot_temperature in robot_temperatures_3[-1]] # find errors across all robots
    max_errors_idx = [MeasureError(robot_temperature=robot_temperature, grid_centralized=grid_centralized)[1] for robot_temperature in robot_temperatures_3[-1]] # find error indices

    max_max_error = np.max(max_errors) # find the maximum error
    max_max_error_idx = np.argmax(max_errors) # find its index
    max_error_idx_idx = max_errors_idx[max_max_error_idx] # find cell with maximum error

    # finish problem 3

    ##### SIMULATING ONE VERY ACCURATE SENSOR (PROBLEM 4)
    print(f'\n=====PROBLEM 4=====')
    vars_robot = [255.0,] * config.num_robots # robot variances
    vars_robot[0] = 1.0 # robot 1 accurate
    vars_robot = tuple(vars_robot)

    # construct shared region for robot 1
    grid_shared_1 = MeasureSharedCells(cell_sets=cell_sets, neighborhood_sense=neighborhoods_sense[0])

    # simulate trials for MSE and plot
    M = 100 # independent trials
    MSE_all_4, MSE_shared_4, RMSE_shared_4, _, _, grid_temp_w_4, grid_temp_u_4 = SimulateMSETrials(grid_temp_real=grid_temp_real, W=W, cell_sets=cell_sets, vars_robot=vars_robot, K=K, M=M, grid_shared=grid_shared_1, robot_measures_prev_w=robot_measures_prev_w, robot_measures_prev_u=robot_measures_prev_u) # get MSEs

    ##### SIMULATING ONE VERY INACCURATE SENSOR (PROBLEM 5)
    print(f'\n=====PROBLEM 5=====')
    vars_robot = [1.0,] * config.num_robots # robot variances
    vars_robot[0] = 255.0 # robot 1 accurate
    vars_robot = tuple(vars_robot)

    # simulate trials for MSE and plot
    MSE_all_5, MSE_shared_5, RMSE_shared_5, _, _, grid_temp_w_5, grid_temp_u_5 = SimulateMSETrials(grid_temp_real=grid_temp_real, W=W, cell_sets=cell_sets, vars_robot=vars_robot, K=K, M=M, grid_shared=grid_shared_1, robot_measures_prev_w=robot_measures_prev_w, robot_measures_prev_u=robot_measures_prev_u) # get MSEs

    ##### REPEATED MEASUREMENTS WITH ONE VERY ACCURATE SENSOR (PROBLEM 6)
    print(f'\n=====PROBLEM 6=====')
    vars_robot = [255.0,] * config.num_robots # robot variances
    vars_robot[0] = 1.0 # robot 1 accurate
    vars_robot = tuple(vars_robot)

    # simulate sequential
    K = 100 # consensus steps
    M = 100 # independent trials
    t = 20 # time steps
    MSE_all_seq, MSE_shared_seq, robot_temperatures_w_seq_6, robot_temperatures_u_seq_6 = SimulateSequential(grid_temp_real=grid_temp_real, W=W, cell_sets=cell_sets, vars_robot=vars_robot, K=K, M=M, grid_shared=grid_shared_1, t=t)

    # getting grid temps for plotting
    grid_temp_w_0 = robot_temperatures_w_seq_6[0][0]
    grid_temp_w_f = robot_temperatures_w_seq_6[-1][0]
    grid_temp_u_0 = robot_temperatures_u_seq_6[0][0]
    grid_temp_u_f = robot_temperatures_u_seq_6[-1][0]

    ##### VISUALIZING RESULTS
    print(f'\n=====PLOTTING=====')
    fig_graph, fig_graph_name = VisualGraphNetwork(A=GraphAdjacency(coords_robots=config.coords_robots), fig_name='graph_network') # graph network
    fig_true_temp, fig_true_temp_name = VisualTempGrid(grid_temp=grid_temp_real, coords_robots=config.coords_robots, grid_shared=np.array([]), plt_name='True Temperature', fig_title='True Temperature Field with Robots', fig_name='true_temperature') # true temperature grid
    
    fig_x55_3, fig_x55_3_name = VisualXij(robot_temperatures=robot_temperatures_3, grid_centralized=grid_centralized, coord_cell=(5,5), fig_name='x55_3_plot') # consensus plot

    fig_temp_w_4, fig_temp_w_4_name = VisualTempGrid(grid_temp=grid_temp_w_4[0], coords_robots=(), grid_shared=grid_shared_1, plt_name='Problem 4 Temp W', fig_title=f'Estimated Temperature Field\nOne Very Accurate Sensor (Weighted)', fig_name='temperature_w_4') # weighted temperature for problem 4
    fig_temp_u_4, fig_temp_u_4_name = VisualTempGrid(grid_temp=grid_temp_u_4[0], coords_robots=(), grid_shared=grid_shared_1, plt_name='Problem 4 Temp U', fig_title=f'Estimated Temperature Field\nOne Very Accurate Sensor (Unweighted)', fig_name='temperature_u_4') # unweighted temperature for problem 4
    fig_error_w_4, fig_error_w_4_name = VisualErrorGrid(grid_temp=grid_temp_w_4[0], grid_temp_real=grid_temp_real, grid_shared=grid_shared_1, plt_name='Problem 4 Error W', fig_title=f'Measured Error Field\nOne Very Accurate Sensor (Weighted)', fig_name='error_w_4') # weighted errors for problem 4
    fig_error_u_4, fig_error_u_4_name = VisualErrorGrid(grid_temp=grid_temp_u_4[0], grid_temp_real=grid_temp_real, grid_shared=grid_shared_1, plt_name='Problem 4 Error U', fig_title=f'Measured Error Field\nOne Very Accurate Sensor (Unweighted)', fig_name='error_u_4') # unweighted errors for problem 4

    fig_temp_w_5, fig_temp_w_5_name = VisualTempGrid(grid_temp=grid_temp_w_5[0], coords_robots=(), grid_shared=grid_shared_1, plt_name='Problem 5 Temp W', fig_title=f'Estimated Temperature Field\nOne Very Inaccurate Sensor (Weighted)', fig_name='temperature_w_5') # weighted temperature for problem 5
    fig_temp_u_5, fig_temp_u_5_name = VisualTempGrid(grid_temp=grid_temp_u_5[0], coords_robots=(), grid_shared=grid_shared_1, plt_name='Problem 5 Temp U', fig_title=f'Estimated Temperature Field\nOne Very Inaccurate Sensor (Unweighted)', fig_name='temperature_u_5') # unweighted temperature for problem 5
    fig_error_w_5, fig_error_w_5_name = VisualErrorGrid(grid_temp=grid_temp_w_5[0], grid_temp_real=grid_temp_real, grid_shared=grid_shared_1, plt_name='Problem 5 Error W', fig_title=f'Measured Error Field\nOne Very Inaccurate Sensor (Weighted)', fig_name='error_w_5') # weighted errors for problem 5
    fig_error_u_5, fig_error_u_5_name = VisualErrorGrid(grid_temp=grid_temp_u_5[0], grid_temp_real=grid_temp_real, grid_shared=grid_shared_1, plt_name='Problem 5 Error U', fig_title=f'Measured Error Field\nOne Very Inaccurate Sensor (Unweighted)', fig_name='error_u_5') # unweighted errors for problem 5

    fig_mse, fig_mse_name = VisualMSE(MSE_all_seq=MSE_all_seq, MSE_shared_seq=MSE_shared_seq, fig_name='mse_plot')
    fig_temp_w_t0, fig_temp_w_t0_name = VisualTempGrid(grid_temp=grid_temp_w_0, coords_robots=(), grid_shared=grid_shared_1, plt_name='Problem 6 Temp W 0', fig_title=f'Estimated Temperature Field\nFirst Sequential Measurement (Weighted)', fig_name='temperature_w_0_6') # weighted temperature for problem 6 t0
    fig_temp_w_tf, fig_temp_w_tf_name = VisualTempGrid(grid_temp=grid_temp_w_f, coords_robots=(), grid_shared=grid_shared_1, plt_name='Problem 6 Temp W f', fig_title=f'Estimated Temperature Field\nLast Sequential Measurement (Weighted)', fig_name='temperature_w_f_6') # weighted temperature for problem 6 t0
    fig_temp_u_t0, fig_temp_u_t0_name = VisualTempGrid(grid_temp=grid_temp_u_0, coords_robots=(), grid_shared=grid_shared_1, plt_name='Problem 6 Temp U 0', fig_title=f'Estimated Temperature Field\nFirst Sequential Measurement (Unweighted)', fig_name='temperature_u_0_6') # unweighted temperature for problem 6 t0
    fig_temp_u_tf, fig_temp_u_tf_name = VisualTempGrid(grid_temp=grid_temp_u_f, coords_robots=(), grid_shared=grid_shared_1, plt_name='Problem 6 Temp U f', fig_title=f'Estimated Temperature Field\nLast Sequential Measurement (Unweighted)', fig_name='temperature_u_f_6') # unweighted temperature for problem 6 t0

    figs = [fig_graph, fig_true_temp, fig_x55_3, fig_temp_w_4, fig_temp_u_4, fig_error_w_4, fig_error_u_4, fig_temp_w_5, fig_temp_u_5, fig_error_w_5, fig_error_u_5, fig_mse, fig_temp_w_t0, fig_temp_u_t0, fig_temp_w_tf, fig_temp_u_tf] # figures
    fig_names = [fig_graph_name, fig_true_temp_name, fig_x55_3_name, fig_temp_w_4_name, fig_temp_u_4_name, fig_error_w_4_name, fig_error_u_4_name, fig_temp_w_5_name, fig_temp_u_5_name, fig_error_w_5_name, fig_error_u_5_name, fig_mse_name, fig_temp_w_t0_name, fig_temp_u_t0_name, fig_temp_w_tf_name, fig_temp_u_tf_name] # figure names
    # figs = [fig_temp_w_t0, fig_temp_u_t0, fig_temp_w_tf, fig_temp_u_tf] # figures
    # fig_names = [fig_temp_w_t0_name, fig_temp_u_t0_name, fig_temp_w_tf_name, fig_temp_u_tf_name] # figure names


    ##### SAVING AND DISPLAYING RESULTS
    PrintInit() # initial printing preparation
    PrintGraph() # printing graph information
    PrintErrors(max_max_error=max_max_error, max_error_idx_idx=max_error_idx_idx) # problem 3 errors
    PrintMSE(MSE_all=MSE_all_4, MSE_shared=MSE_shared_4, RMSE_shared=RMSE_shared_4, problem=4) # problem 4 MSEs
    PrintMSE(MSE_all=MSE_all_5, MSE_shared=MSE_shared_5, RMSE_shared=RMSE_shared_5, problem=5) # problem 5 MSEs

    SaveFigures(figs=figs, fig_names=fig_names) # saving results
    VisualShow() # showing plots

    ##### DEBUGGING AND OTHER PRINTING
    print()
    # print(cell_nums)