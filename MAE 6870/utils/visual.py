##### IMPORTS
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import matplotlib.patches as patches
import networkx as nx
import numpy as np

##### SELF-DEFINED IMPORTS
import config

##### GRAPH VISUALIZE FUNCTION (inputs adjacency matrix and visualizes the network graph)
def VisualGraphNetwork(A: np.array, fig_name=str) -> tuple[plt.Figure, str]:
    # creating figure
    fig_graph = plt.figure('Graph Network', figsize=(7,5))
    fig_graph.suptitle('Graph Network Structure')

    # construct graph from adjacency
    G = nx.from_numpy_array(A)

    mapping = {node: node + 1 for node in G.nodes()} # remapping node labels
    G_shifted = nx.relabel_nodes(G, mapping)

    # visualizing graph
    nx.draw_networkx(
        G=G_shifted, with_labels=True, node_color='skyblue', node_size=800, font_weight='bold',
    )

    # finalizing plot and returning
    return fig_graph, fig_name

##### TEMPERATURE GRID VISUALIZATION FUNCTION (inputs temperature grid and robot coordinates and creates plot)
def VisualTempGrid(grid_temp: np.array, coords_robots: tuple, grid_shared: np.array, plt_name: str, fig_title: str, fig_name: str) -> tuple[plt.Figure, str]:
    # creating figure
    fig_temp = plt.figure(plt_name, figsize=(7,5))
    fig_temp.suptitle(fig_title)
    ax = fig_temp.add_subplot(1, 1, 1)

    # find grid size
    rows, cols = np.shape(grid_temp)
    X, Y = np.meshgrid(np.arange(cols+1), np.arange(rows+1))

    # defining colormap
    cmap = mcolors.LinearSegmentedColormap.from_list(name='cmap', colors=config.grid_temp_colors, N=256)

    # plotting temperatures
    mesh = ax.pcolormesh(X, Y, grid_temp, cmap=cmap, vmin=config.grid_temp_vmin, vmax=config.grid_temp_vmax, edgecolors='#ffffff', linewidth=1)
    ax.set_xticks(np.arange(cols)+0.5)
    ax.set_xticklabels(np.arange(cols))
    ax.set_yticks(np.arange(rows)+0.5)
    ax.set_yticklabels(np.arange(rows))
    ax.set_xlabel(r'Column $c$ ($m$)')
    ax.set_ylabel(r'Row $r$ ($m$)')
    ax.set_aspect('equal')

    # plotting colorbar
    cbar = fig_temp.colorbar(mesh, ax=ax)
    cbar.set_label(r'Temperature ($^oC$)', rotation=90, labelpad=15)

    # creating robot tokens and plotting on grid (if included)
    robot_tokens = [(coord[0], coord[1], idx+1, '#000000') for idx, coord in enumerate(coords_robots)]

    for y, x, label, color in robot_tokens: # adding sensing circles
        sense = plt.Circle((x+0.5, y+0.5), radius=config.radius_sensing, fill=False, edgecolor=color, linewidth=2)
        ax.add_patch(sense)
    for y, x, label, color in robot_tokens: # adding robot markers
        robot = plt.Circle((x+0.5, y+0.5), radius=0.3, facecolor='#ffffff', edgecolor=color, linewidth=2)
        ax.add_patch(robot)
        ax.text(x+0.5, y+0.5, label, color=color, weight='bold', fontsize=11, ha='center', va='center')

    # outlining desired shared cells (if included)
    for row_idx, row in enumerate(grid_shared): # for each row
        for col_idx, cell in enumerate(row): # for each cell
            if cell == 1: # if cell is shared
                outline = patches.Rectangle(
                    ((col_idx+0.08)*config.grid_distance, (row_idx+0.08)*config.grid_distance), width=0.84*config.grid_distance, height=0.84*config.grid_distance, linewidth=2.0, edgecolor='#000000', facecolor='none'
                )
                ax.add_patch(outline)

    # finalizing plot and returning
    fig_temp.tight_layout()
    return fig_temp, fig_name

##### ERROR GRID VISUALIZATION FUNCTION (inputs grid and creates plots)
def VisualErrorGrid(grid_temp: np.array, grid_temp_real: np.array, grid_shared: np.array, plt_name: str, fig_title: str, fig_name: str) -> tuple[plt.Figure, str]:
    # creating figure
    fig_error = plt.figure(plt_name, figsize=(7,5))
    fig_error.suptitle(fig_title)
    ax = fig_error.add_subplot(1, 1, 1)

    # find grid size
    rows, cols = np.shape(grid_temp)
    X, Y = np.meshgrid(np.arange(cols+1), np.arange(rows+1))

    # defining colormap
    cmap = mcolors.LinearSegmentedColormap.from_list(name='cmap', colors=config.grid_error_colors, N=256)

    # finding error grid
    grid_error = grid_temp - grid_temp_real

    # plotting temperatures
    mesh = ax.pcolormesh(X, Y, grid_error, cmap=cmap, vmin=config.grid_error_vmin, vmax=config.grid_error_vmax, edgecolors='#ffffff', linewidth=1)
    ax.set_xticks(np.arange(cols)+0.5)
    ax.set_xticklabels(np.arange(cols))
    ax.set_yticks(np.arange(rows)+0.5)
    ax.set_yticklabels(np.arange(rows))
    ax.set_xlabel(r'Column $c$ ($m$)')
    ax.set_ylabel(r'Row $r$ ($m$)')
    ax.set_aspect('equal')

    # plotting colorbar
    cbar = fig_error.colorbar(mesh, ax=ax)
    cbar.set_label(r'Error ($^oC$)', rotation=90, labelpad=15)

    # graying out uneeded cells
    for row_idx, row in enumerate(grid_shared): # for each row
        for col_idx, cell in enumerate(row): # for each cell
            if cell == 0: # if cell not shared
                outline = patches.Rectangle(
                    ((col_idx+0.05)*config.grid_distance, (row_idx+0.05)*config.grid_distance), width=0.9*config.grid_distance, height=0.9*config.grid_distance, linewidth=1.0, edgecolor="#121212", facecolor='#121212'
                )
                ax.add_patch(outline)

    # finalizing plot and returning
    fig_error.tight_layout()
    return fig_error, fig_name


##### VISUALIZE CELL Xij FUNCTION (inputs robot temperatures over time and desired cell coordinates ij for plotting)
def VisualXij(robot_temperatures: list, grid_centralized: np.array, coord_cell: tuple, fig_name: str) -> tuple[plt.Figure, str]:
    # creating figure
    fig_xij = plt.figure(f'Cell X{coord_cell[0]}{coord_cell[1]}', figsize=(7,5))
    fig_xij.suptitle(fr'Consensus Convergence for $x_{{{coord_cell[0]}{coord_cell[1]}}}^{{(i)}}$')
    ax = fig_xij.add_subplot(1, 1, 1)

    # extracting xij for all robots for all k
    xij = [[robot_temperatures_step[robot_idx][coord_cell[0],coord_cell[1]] for robot_idx, _ in enumerate(robot_temperatures_step)] for robot_temperatures_step in robot_temperatures]
    xij = np.array(xij) # make np array for index slicing
    _, cols = np.shape(xij) # get shape
    K = np.arange(len(robot_temperatures)) # consensus steps

    # plotting timeseries solution for xij
    for col_idx in range(cols): ax.plot(K, xij[:,col_idx], label=fr'Robot {col_idx+1}')
    ax.axhline(y=grid_centralized[coord_cell[0],coord_cell[1]], color='gray', linestyle='--', label=fr'Centralized Estimate $\hat\theta_{{rc}}^w$')
    ax.legend(loc='best')
    ax.set_xlabel(r'Consensuse step ($k$)')
    ax.set_ylabel(r'Robot estimates ($^oC$)')

    # finalizing plot and returning
    return fig_xij, fig_name

##### VISUAL MSE VERSUS TIME FUNCTION (plots total and shared MSE versus time)
def VisualMSE(MSE_all_seq: list, MSE_shared_seq: list, fig_name: str) -> tuple[plt.Figure, str]:
    # creating figure
    fig_mse = plt.figure('MSE plot', figsize=(7,5))
    fig_mse.suptitle(fr'Total Grid and Shared $O_1$ MSE versus Sequential Step $t$')
    ax = fig_mse.add_subplot(1, 1, 1)

    # finding number of steps and convert to np array
    t = np.arange(len(MSE_all_seq))
    MSE_all_seq = np.array(MSE_all_seq)
    MSE_shared_seq = np.array(MSE_shared_seq)

    # plotting MSEs
    ax.plot(t, MSE_all_seq[:,0], label=fr'Total Grid MSE (weighted)')
    ax.plot(t, MSE_all_seq[:,1], label=fr'Total Grid MSE (unweighted)')
    ax.plot(t, MSE_shared_seq[:,0], label=fr'Shared $O_1$ MSE (weighted)')
    ax.plot(t, MSE_shared_seq[:,1], label=fr'Shared $O_1$ MSE (unweighted)')
    ax.legend(loc='best')
    ax.set_xlabel(r'Sequential step ($t$)')
    ax.set_ylabel(r'Average MSE $(^oC)^2$')
    ax.set_xticks(t)
    ax.set_xticklabels(t+1)

    # finalizing plot and returning
    return fig_mse, fig_name

##### VISUAL SHOW FUNCTION (displays all plots)
def VisualShow() -> None:
    # showing all plots
    plt.show()