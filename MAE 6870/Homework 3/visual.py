##### RPS IMPORTS

##### OTHER IMPORTS
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from pathlib import Path

##### SELF-DEFINED IMPORTS
import config
from error import ErrorDistanceConstraint, ErrorSquare, ErrorReference

##### FORMATION VISUALIZER FUNCTION (inputs desired robot positions and plots/saves formation)
def VisualFormation(positions: np.ndarray) -> plt.Figure:
    # defining constants
    workspace_width = 3.2 # (m)
    workspace_height = 2.0 # (m)

    # creating figure
    fig_form = plt.figure('Formation plot', figsize=(7,5))
    fig_form.suptitle(f'Desired agent formation\nPart A. Static-formation control')
    ax = fig_form.add_subplot(1, 1, 1)

    # adding the workspace to the plot
    workspace = patches.Rectangle((-workspace_width/2, -workspace_height/2), width=workspace_width, height=workspace_height, linewidth=2.0, edgecolor='#000000', facecolor='#999999')
    ax.add_patch(workspace)

    # adding the robots to the plot
    for robot_idx, [x, y] in enumerate(positions): # for each robot
        robot = plt.Circle((x, y), radius=0.06, facecolor='#ffffff', edgecolor='#0000ff', linewidth=1) # add position marker
        ax.add_patch(robot)
        ax.text(x, y-0.01, robot_idx, color='#000000', weight='bold', fontsize=10, ha='center', va='center') # add index

    ax.set_xlim((-workspace_width/2-0.1, workspace_width/2+0.1))
    ax.set_ylim((-workspace_height/2-0.1, workspace_height/2+0.1))
    ax.set_xlabel(r'$x$ $(m)$')
    ax.set_ylabel(r'$y$ $(m)$')
    ax.set_aspect('equal')

    # return plot
    return fig_form

##### SQUARE ERROR PLOT FUNCTION (inputs error and returns error plot)
def VisualSquareError(errors: list, legend: list, idx: int, tag: str) -> plt.Figure:
    # defining constants
    t_step = 0.033 # (s)

    # extracting errors
    if len(errors) > 1: # if plotting multiple errors
        dist_error = errors[0]
        disp_error = errors[1]
    else: error = errors[0]

    # creating figure
    fig_error = plt.figure(f'Square Error Plot {idx}', figsize=(7,5))
    fig_error.suptitle(f'Square Error versus time\n' + tag)
    ax = fig_error.add_subplot(1, 1, 1)

    # plotting data
    t = t_step * np.arange(0, len(errors[0])) # time data
    if len(errors) > 1:
        ax.plot(t, dist_error, linestyle='-', label=legend[0])
        ax.plot(t, disp_error, linestyle='--', label=legend[1])
        ax.legend(loc='best')
    else: ax.plot(t, error, linestyle='-')
    ax.set_xlabel(r'Experiment time $t$ $(s)$')
    ax.set_ylabel(r'Square Error $(m^2)$')

    # return figure
    return fig_error

##### CONSTRAINT ERROR PLOT FUNCTION (inputs error and returns plot)
def VisualConstraintError(errors: list, idx: int, tag: str) -> plt.Figure:
    # defining constants
    t_step = 0.033 # (s)

    # extracting errors
    sim_error = errors[0]
    real_error = errors[1]

    # creating figure
    fig_error = plt.figure(f'Constraint Error Plot {idx}', figsize=(7,5))
    fig_error.suptitle(f'Constraint Error versus Desired\n' + tag)
    ax = fig_error.add_subplot(1, 1, 1)

    # plotting data
    t = t_step * np.arange(0, len(errors[0])) # time data
    ax.plot(t, sim_error, linestyle='-', label=fr'Simulated')
    ax.plot(t, real_error, linestyle='--', label=fr'Real')
    ax.legend(loc='best')
    ax.set_xlabel(r'Experiment time $t$ $(s)$')
    ax.set_ylabel(r'Square Error $(m)$')

    # return figure
    return fig_error

##### TRAJECTORY PLOT FUNCTION (inputs pose data and plots trajectories)
def VisualTrajectory(poses: list, waypoints: np.ndarray, idx: int, tag: str) -> plt.Figure:
    # defining constants
    workspace_width = 3.2 # (m)
    workspace_height = 2.0 # (m)

    _, N, _ = np.shape(poses[0]) # find number of robots

    # extracting poses
    sim_poses = poses[0]
    real_poses = poses[1]
    if len(poses) > 2: # if adding reference and average
        sim_avg_poses = poses[2]
        real_avg_poses = poses[3]
        sim_ref_poses = poses[4]
        real_ref_poses = poses[5]

    # creating figure
    fig_traj = plt.figure(f'Trajectory Plot {idx}', figsize=(7,5))
    fig_traj.suptitle(f'Robot Trajectories\n' + tag)
    ax = fig_traj.add_subplot(1, 1, 1)

    # adding the workspace to the plot
    workspace = patches.Rectangle((-workspace_width/2, -workspace_height/2), width=workspace_width, height=workspace_height, linewidth=2.0, edgecolor='#000000', facecolor='#999999')
    ax.add_patch(workspace)

    # plotting waypoints
    if waypoints is not None:
        for waypoint_idx, [x, y] in enumerate(waypoints):
            waypoint = plt.Circle((x, y), radius=0.06, facecolor='#ffffff', edgecolor='#0000ff', linewidth=1) # add position marker
            ax.add_patch(waypoint)
            ax.text(x, y-0.01, waypoint_idx, color='#000000', weight='bold', fontsize=10, ha='center', va='center') # add index

    # plotting trajectories
    for i in range(N):
        if i == 0: ax.plot(sim_poses[0,i,:], sim_poses[1,i,:], linestyle='-', label=fr'Simulated')
        else: ax.plot(sim_poses[0,i,:], sim_poses[1,i,:], linestyle='-')
    ax.set_prop_cycle(None)
    for i in range(N):
        if i == 0: ax.plot(real_poses[0,i,:], real_poses[1,i,:], linestyle='--', label=fr'Real')
        else: ax.plot(real_poses[0,i,:], real_poses[1,i,:], linestyle='--')
    if len(poses) > 2: # if plotting reference and average
        ax.plot(sim_avg_poses[:,0], sim_avg_poses[:,1], linestyle='-', color='#000000', label=fr'Simulated $\bar{{p}}$')
        ax.plot(real_avg_poses[:,0], real_avg_poses[:,1], linestyle='--', color='#000000', label=fr'Real $\bar{{p}}$')
        ax.plot(sim_ref_poses[:,0], sim_ref_poses[:,1], linestyle='-', color='#00ff00', label=fr'Reference path')
        # ax.plot(real_ref_poses[:,0], real_ref_poses[:,1], linestyle='--', color='#00ff00', label=fr'Reference')
    ax.legend(loc='best')
    ax.set_xlim((-workspace_width/2-0.1, workspace_width/2+0.1))
    ax.set_ylim((-workspace_height/2-0.1, workspace_height/2+0.1))
    ax.set_xlabel(r'$x$ $(m)$')
    ax.set_ylabel(r'$y$ $(m)$')
    ax.set_aspect('equal')

    # return figure
    return fig_traj

##### MAIN SCRIPT (FOR PLOTTING)
if __name__ == '__main__':
    # resolving directories
    config.LOC_DIR = Path(__file__).resolve().parent # find local directory
    config.DAT_DIR = config.LOC_DIR / config.DAT_DIR # update data directory
    config.FIG_DIR = config.LOC_DIR / config.FIG_DIR # update figure directory

    # creating formation plot
    fig_form = VisualFormation(positions=config.ac_disp)

    # calculating errors (A. static formation)
    a_dist_sim_poses = np.load(str(config.DAT_DIR / config.a_dist_sim_name)) # simulated distance algorithm data
    a_dist_real_poses = np.load(str(config.DAT_DIR / config.a_dist_real_name)) # real distance algorithm data
    a_disp_sim_poses = np.load(str(config.DAT_DIR / config.a_disp_sim_name)) # simulated displacement algorithm data
    a_disp_real_poses = np.load(str(config.DAT_DIR / config.a_disp_real_name)) # real displacement algorithm data

    a_dist_square_error = np.array([]) # empty error array
    a_dist_sim_const_error = np.array([]) # empty error array
    a_dist_real_const_error = np.array([]) # empty error array
    a_disp_sim_const_error = np.array([]) # empty error array
    a_disp_real_const_error = np.array([]) # empty error array
    a_disp_square_error = np.array([]) # empty error array

    _, N, t_steps = np.shape(a_dist_sim_poses) # finding number of robots and number of time step data points
    for t in range(t_steps): # for each time step
        a_dist_square_error = np.append(a_dist_square_error, ErrorSquare(sim_poses=a_dist_sim_poses[:,:,t], real_poses=a_dist_real_poses[:,:,t])) # find square error
        a_disp_square_error = np.append(a_disp_square_error, ErrorSquare(sim_poses=a_disp_sim_poses[:,:,t], real_poses=a_disp_real_poses[:,:,t])) # find square error

        a_dist_sim_const_error = np.append(a_dist_sim_const_error, ErrorDistanceConstraint(W=config.ac_dmatrix, poses=a_dist_sim_poses[:,:,t])) # find distance error
        a_dist_real_const_error = np.append(a_dist_real_const_error, ErrorDistanceConstraint(W=config.ac_dmatrix, poses=a_dist_real_poses[:,:,t])) # find distance error
        a_disp_sim_const_error = np.append(a_disp_sim_const_error, ErrorDistanceConstraint(W=config.ac_dmatrix, poses=a_disp_sim_poses[:,:,t])) # find distance error
        a_disp_real_const_error = np.append(a_disp_real_const_error, ErrorDistanceConstraint(W=config.ac_dmatrix, poses=a_disp_real_poses[:,:,t])) # find distance error

    # calculating errors (B. moving reference)
    b_global_sim_poses = np.load(str(config.DAT_DIR / config.b_global_sim_name)) # simulated global data
    b_global_real_poses = np.load(str(config.DAT_DIR / config.b_global_real_name)) # real global data
    b_global_sim_avg = np.load(str(config.DAT_DIR / config.b_global_sim_avg_name)) # simulated global average data
    b_global_real_avg = np.load(str(config.DAT_DIR / config.b_global_real_avg_name)) # real global average data
    b_global_sim_ref = np.load(str(config.DAT_DIR / config.b_global_sim_ref_name)) # simulated global reference data
    b_global_real_ref = np.load(str(config.DAT_DIR / config.b_global_real_ref_name)) # real global reference data
    b_global_sim_way = np.load(str(config.DAT_DIR / config.b_global_sim_way_name)) # simulated global waypoint data
    b_global_real_way = np.load(str(config.DAT_DIR / config.b_global_real_way_name)) # real global waypoint data
    b_leader_sim_poses = np.load(str(config.DAT_DIR / config.b_leader_sim_name)) # simulated leader reference data
    b_leader_real_poses = np.load(str(config.DAT_DIR / config.b_leader_real_name)) # real leader reference data

    b_global_square_error = np.array([]) # empty error array
    b_leader_square_error = np.array([]) # empty error array
    b_global_sim_way_error = np.array([]) # empty error array
    b_global_real_way_error = np.array([]) # empty error array
    b_global_sim_const_error = np.array([]) # empty error array
    b_global_real_const_error = np.array([]) # empty error array
    b_leader_sim_const_error = np.array([]) # empty error array
    b_leader_real_const_error = np.array([]) # empty error array
    b_leader_sim_fol_const_error = np.array([]) # empty error array
    b_leader_real_fol_const_error = np.array([]) # empty error array

    _, N, t_steps = np.shape(b_global_sim_poses) # finding number of robots and number of time step data points
    for t in range(t_steps): # for each time step
        b_global_square_error = np.append(b_global_square_error, ErrorSquare(sim_poses=b_global_sim_poses[:,:,t], real_poses=b_global_real_poses[:,:,t])) # find square error
        b_leader_square_error = np.append(b_leader_square_error, ErrorSquare(sim_poses=b_leader_sim_poses[:,:,t], real_poses=b_leader_real_poses[:,:,t])) # find square error

        b_global_sim_const_error = np.append(b_global_sim_const_error, ErrorDistanceConstraint(W=config.b_dmatrix[1:,1:], poses=b_global_sim_poses[:,:,t]))
        b_global_real_const_error = np.append(b_global_real_const_error, ErrorDistanceConstraint(W=config.b_dmatrix[1:,1:], poses=b_global_real_poses[:,:,t]))
        b_leader_sim_const_error = np.append(b_leader_sim_const_error, ErrorDistanceConstraint(W=config.b_dmatrix, poses=b_leader_sim_poses[:,:,t]))
        b_leader_real_const_error = np.append(b_leader_real_const_error, ErrorDistanceConstraint(W=config.b_dmatrix, poses=b_leader_real_poses[:,:,t]))
        b_leader_sim_fol_const_error = np.append(b_leader_sim_fol_const_error, ErrorDistanceConstraint(W=config.b_dmatrix[1:,1:], poses=b_leader_sim_poses[:,1:,t]))
        b_leader_real_fol_const_error = np.append(b_leader_real_fol_const_error, ErrorDistanceConstraint(W=config.b_dmatrix[1:,1:], poses=b_leader_real_poses[:,1:,t]))
        
        b_global_sim_way_error = np.append(b_global_sim_way_error, ErrorReference(average_pose=b_global_sim_avg[t,:], waypoint_pose=b_global_sim_way[t,:])) # find reference error
        b_global_real_way_error = np.append(b_global_real_way_error, ErrorReference(average_pose=b_global_real_avg[t,:], waypoint_pose=b_global_real_way[t,:])) # find reference error

    # calculating errors (C. proximity)
    c_dist_sim_poses = np.load(str(config.DAT_DIR / config.c_dist_sim_name)) # simulated distance algorithm data
    c_dist_real_poses = np.load(str(config.DAT_DIR / config.c_dist_real_name)) # simulated distance algorithm data

    c_dist_square_error = np.array([]) # empty error array
    c_dist_sim_const_error = np.array([]) # empty error array
    c_dist_real_const_error = np.array([]) # empty error array

    _, N, t_steps = np.shape(c_dist_sim_poses) # finding number of robots and number of time step data points
    for t in range(t_steps):
        c_dist_square_error = np.append(c_dist_square_error, ErrorSquare(sim_poses=c_dist_sim_poses[:,:,t], real_poses=c_dist_real_poses[:,:,t])) # find square error

        c_dist_sim_const_error = np.append(c_dist_sim_const_error, ErrorDistanceConstraint(W=config.ac_dmatrix, poses=c_dist_sim_poses[:,:,t])) # find distance error
        c_dist_real_const_error = np.append(c_dist_real_const_error, ErrorDistanceConstraint(W=config.ac_dmatrix, poses=c_dist_real_poses[:,:,t])) # find distance error

    # creating error plots (A. static formation)
    a_fig_square_error = VisualSquareError(errors=[a_dist_square_error, a_disp_square_error], legend=[fr'Distance-based', fr'Displacement-based'], idx=0, tag=f'Part A. Static-formation control')
    a_fig_dist_const_error = VisualConstraintError(errors=[a_dist_sim_const_error, a_dist_real_const_error], idx=0, tag=f'Part A. Static-formation control (distance-based)')
    a_fig_disp_const_error = VisualConstraintError(errors=[a_disp_sim_const_error, a_disp_real_const_error], idx=1, tag=f'Part A. Static-formation control (displacement-based)')

    # creating error plots (B. moving reference)
    b_fig_square_error = VisualSquareError(errors=[b_global_square_error, b_leader_square_error], legend=[fr'Global knowledge', fr'Leader-follower'], idx=1, tag=f'Part B. Track a moving reference (global knowledge v. leader-follower)')
    b_fig_reference_error = VisualSquareError(errors=[b_global_sim_way_error, b_global_real_way_error], legend=[fr'Simulated', fr'Real'], idx=2, tag=f'Part B. Track a moving reference (reference v. waypoint)')
    b_fig_global_const_error = VisualConstraintError(errors=[b_global_sim_const_error, b_global_real_const_error], idx=2, tag=f'Part B. Track a moving reference (global knowledge)')
    b_fig_leader_const_error = VisualConstraintError(errors=[b_leader_sim_const_error, b_leader_real_const_error], idx=3, tag=f'Part B. Track a moving reference (leader-follower w/ leader)')
    b_fig_leader_fol_const_error = VisualConstraintError(errors=[b_leader_sim_fol_const_error, b_leader_real_fol_const_error], idx=4, tag=f'Part B. Track a moving reference (leader-follower w/o leader)')

    # creating error plots (C. proximity)
    c_fig_square_error = VisualSquareError(errors=[c_dist_square_error], legend=None, idx=3, tag=f'Part C. Proximity-based communication')
    c_fig_const_error = VisualConstraintError(errors=[c_dist_sim_const_error, c_dist_real_const_error], idx=5, tag=f'Part C. Proximity-based communication')

    # creating trajectory plots (A. static formation)
    a_fig_dist_traj = VisualTrajectory(poses=[a_dist_sim_poses, a_dist_real_poses], waypoints=None, idx=0, tag=f'Part A. Static-formation control (distance-based)')
    a_fig_disp_traj = VisualTrajectory(poses=[a_disp_sim_poses, a_disp_real_poses], waypoints=None, idx=1, tag=f'Part A. Static-formation control (displacement-based)')

    # creating trajectory plots (B. moving reference)
    b_fig_global_traj = VisualTrajectory(poses=[b_global_sim_poses, b_global_real_poses, b_global_sim_avg, b_global_real_avg, b_global_sim_ref, b_global_real_ref], waypoints=config.b_waypoints, idx=2, tag=f'Part B. Tracking a moving reference (global knowledge)')
    b_fig_leader_traj = VisualTrajectory(poses=[b_leader_sim_poses, b_leader_real_poses], waypoints=config.b_waypoints, idx=3, tag=f'Part B. Tracking a moving reference (leader-follower)')

    # creating trajectory plots (C. proximity)
    c_fig_dist_traj = VisualTrajectory(poses=[c_dist_sim_poses, c_dist_real_poses], waypoints=None, idx=4, tag=f'Part C. Proximity-based communication')

    # saving figures
    config.FIG_DIR.mkdir(parents=True, exist_ok=True) # create directory if doesn't exist

    fig_form.savefig(str(config.FIG_DIR / 'formation'), dpi=300, bbox_inches='tight')

    a_fig_square_error.savefig(str(config.FIG_DIR / 'a_square_error'), dpi=300, bbox_inches='tight')
    a_fig_dist_const_error.savefig(str(config.FIG_DIR / 'a_dist_const_error'), dpi=300, bbox_inches='tight')
    a_fig_disp_const_error.savefig(str(config.FIG_DIR / 'a_disp_const_error'), dpi=300, bbox_inches='tight')
    a_fig_dist_traj.savefig(str(config.FIG_DIR / 'a_traj_dist'), dpi=300, bbox_inches='tight')
    a_fig_disp_traj.savefig(str(config.FIG_DIR / 'a_traj_disp'), dpi=300, bbox_inches='tight')    

    b_fig_square_error.savefig(str(config.FIG_DIR / 'b_square_error'), dpi=300, bbox_inches='tight')
    b_fig_reference_error.savefig(str(config.FIG_DIR / 'b_reference_error'), dpi=300, bbox_inches='tight')
    b_fig_global_const_error.savefig(str(config.FIG_DIR / 'b_global_const_error'), dpi=300, bbox_inches='tight')
    b_fig_leader_const_error.savefig(str(config.FIG_DIR / 'b_leader_const_error'), dpi=300, bbox_inches='tight')
    b_fig_leader_fol_const_error.savefig(str(config.FIG_DIR / 'b_leader_fol_const_error'), dpi=300, bbox_inches='tight')
    b_fig_global_traj.savefig(str(config.FIG_DIR / 'b_traj_global'), dpi=300, bbox_inches='tight')
    b_fig_leader_traj.savefig(str(config.FIG_DIR / 'b_traj_leader'), dpi=300, bbox_inches='tight')

    c_fig_square_error.savefig(str(config.FIG_DIR / 'c_square_error'), dpi=300, bbox_inches='tight')
    c_fig_const_error.savefig(str(config.FIG_DIR / 'c_const_error'), dpi=300, bbox_inches='tight')
    c_fig_dist_traj.savefig(str(config.FIG_DIR / 'c_traj_dist'), dpi=300, bbox_inches='tight')    

    # showing results
    plt.show()