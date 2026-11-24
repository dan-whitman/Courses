##### PYDRAKE IMPORTS

##### OTHER IMPORTS
import numpy as np
import matplotlib.pyplot as plt

##### SELF-DEFINED IMPORTS
import inputs.config as config
from FPVDroneDrake import ScriptInit

##### VBLOCK INDEX RETURN FUNCTION (input idx range, outputs idx bins)
def PlotVBinIdx(idx: list) -> list:
    # print(f'\n=====CREATING PIXEL BINS=====')

    idxblocks = [] # initial empty list

    if idx: # if the list is not empty
        i = 0

        while i < len(idx): # while inside the list
            idxblocks.append(idx[i]) # add element i to bin start
            if i != len(idx) - 1: # if not at last element
                for j in range(1, len(idx) - i): # loop over remaining elements
                    if idx[i+j] != idx[i] + j: # if not consecutive
                        idxblocks.append(idx[i+j-1]) # add element i+j-1 to bin end
                        i = i + j # update i
                        break
                    if i + j == len(idx) - 1: # if at last element
                        idxblocks.append(idx[i+j]) # add last element to bin
                        i = len(idx) # update i to break while
                        break
            else: break # if at last element

    if len(idxblocks) % 2 == 1: idxblocks.append(-1) # if odd, append -1
    idxblocks = [idxblocks[i:i+2] for i in range(0,len(idxblocks),2)] # group bins

    return idxblocks # return final bins

##### PLOTTING TARGET PIXELS FUNCTION (plots true and estimated target pixels with dead zones)
def PlotTargetPixels(t: np.ndarray, data: list) -> plt.Figure:
    print(f'\n=====PLOTTING TARGET PIXELS=====')

    # extracting data
    pT = data[0] # true pixel data
    pTe = data[1] # estimated pixel data

    pT_nanidx = [idx for idx, p in enumerate(pT[0,:]) if np.isnan(p)] # all values where true is nan
    pTe_nanidx = [idx for idx, p in enumerate(pTe[0,:]) if np.isnan(p)] # all values where estimate is nan

    pT_nanbins = PlotVBinIdx(idx=pT_nanidx) # true nan bins
    pTe_nanbins = PlotVBinIdx(idx=pTe_nanidx) # estimated nan bins

    # creating figure
    fig = plt.figure(f'Target Pixels Plot', figsize=(7, 5))
    fig.suptitle(fr'Target Pixels ($p_T$ and $p_T^e$) versus Time')

    # plotting data
    ax = fig.add_subplot(1, 1, 1)

    ax.plot(t, pT[0,:], linestyle='-', color='#ff0000', label=fr'True $u_T$')
    ax.plot(t, pT[1,:], linestyle='--', color='#ff0000', label=fr'True $v_T$')
    ax.plot(t, pTe[0,:], linestyle='-', color='#0000ff', label=fr'Estimated $u_T^e$')
    ax.plot(t, pTe[1,:], linestyle='--', color='#0000ff', label=fr'Estimated $v_T^e$')

    ax.hlines(config.camera_width, xmin=t[0], xmax=t[-1], color='#000000', linestyles='-')
    ax.hlines(config.camera_height, xmin=t[0], xmax=t[-1], color='#000000', linestyles='--')
    ax.axhline(0, color='#000000')

    if pT_nanbins: # if true nan bins
        for bin in pT_nanbins: # add bins to plot
            ax.axvspan(t[bin[0]], t[bin[1]], color='#ff0000', alpha=0.4)
    if pTe_nanbins: # if estimated nan bins
            for bin in pTe_nanbins: # add bins to plot
                ax.axvspan(t[bin[0]], t[bin[1]], color='#0000ff', alpha=0.4)
        
    ax.set_xlabel(r'Time ($s$)')
    ax.set_ylabel(r'Camera pixel measurements ($px$)')
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.grid()

    # return figure
    return fig

##### PLOTTING TARGET POSITIONS FUNCTIONS (plots true and estimated positions with dead zones)
def PlotTargetPositions(t: np.ndarray, data: list) -> plt.Figure:
    print(f'\n=====PLOTTING TARGET POSITIONS=====')

    # extracting data
    xT = data[0] # true target state data
    xTe = data[1] # estimated target state data

    xTe_nanidx = [idx for idx, x in enumerate(xTe[3,:]) if np.isnan(x)] # all values where true is nan

    xTe_nanbins = PlotVBinIdx(idx=xTe_nanidx) # estimated nan bins

    # creating figure
    fig = plt.figure(f'Target Positions Plot', figsize=(7, 5))
    fig.suptitle(fr'Target Positions ($x_T$ and $x_T^e$) versus Time')

    # plotting data
    ax = fig.add_subplot(3, 1, 1)

    ax.plot(t, xT[3,:], linestyle='-', color='#ff0000', label=fr'True $x$')
    ax.plot(t, xTe[3,:], linestyle='-', color='#0000ff', label=fr'Estimated $x^e$')

    if xTe_nanbins: # if estimated nan bins
        for bin in xTe_nanbins: # add bins to plot
            ax.axvspan(t[bin[0]], t[bin[1]], color='#0000ff', alpha=0.4)

    ax.set_xlabel(r'Time ($s$)')
    ax.set_ylabel(r'Global $x$-coordinate ($m$)')
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.grid()

    ax = fig.add_subplot(3, 1, 2)
    
    ax.plot(t, xT[4,:], linestyle='-', color='#ff0000', label=fr'True $y$')
    ax.plot(t, xTe[4,:], linestyle='-', color='#0000ff', label=fr'Estimated $y^e$')

    if xTe_nanbins: # if estimated nan bins
        for bin in xTe_nanbins: # add bins to plot
            ax.axvspan(t[bin[0]], t[bin[1]], color='#0000ff', alpha=0.4)

    ax.set_xlabel(r'Time ($s$)')
    ax.set_ylabel(r'Global $y$-coordinate ($m$)')
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.grid()

    ax = fig.add_subplot(3, 1, 3)
    
    ax.plot(t, xT[5,:], linestyle='-', color='#ff0000', label=fr'True $z$')
    ax.plot(t, xTe[5,:], linestyle='-', color='#0000ff', label=fr'Estimated $z^e$')

    if xTe_nanbins: # if estimated nan bins
        for bin in xTe_nanbins: # add bins to plot
            ax.axvspan(t[bin[0]], t[bin[1]], color='#0000ff', alpha=0.4)

    ax.set_xlabel(r'Time ($s$)')
    ax.set_ylabel(r'Global $z$-coordinate ($m$)')
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.grid()

    # return figure
    return fig

##### PLOTTING TARGET VELOCITIES FUNCTION (plots true and estimated target velocities with dead zones)
def PlotTargetVelocities(t: np.ndarray, data: list) -> plt.Figure:
    print(f'\n=====PLOTTING TARGET VELOCITIES=====')

    # extracting data
    xT = data[0] # true target state data
    xTe = data[1] # estimated target state data

    xTe_nanidx = [idx for idx, x in enumerate(xTe[3,:]) if np.isnan(x)] # all values where true is nan

    xTe_nanbins = PlotVBinIdx(idx=xTe_nanidx) # estimated nan bins

    # creating figure
    fig = plt.figure(f'Target Velocities Plot', figsize=(7, 5))
    fig.suptitle(fr'Target Velocities ($\dot x_T$ and $\dot x_T^e$) versus Time')

    # plotting data
    ax = fig.add_subplot(3, 1, 1)

    ax.plot(t, xT[9,:], linestyle='-', color='#ff0000', label=fr'True $\dot x$')
    ax.plot(t, xTe[9,:], linestyle='-', color='#0000ff', label=fr'Estimated $\dot x^e$')

    if xTe_nanbins: # if estimated nan bins
        for bin in xTe_nanbins: # add bins to plot
            ax.axvspan(t[bin[0]], t[bin[1]], color='#0000ff', alpha=0.4)

    ax.set_xlabel(r'Time ($s$)')
    ax.set_ylabel(r'Global $x$-velocity ($m/s$)')
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.grid()

    ax = fig.add_subplot(3, 1, 2)
    
    ax.plot(t, xT[10,:], linestyle='-', color='#ff0000', label=fr'True $\dot y$')
    ax.plot(t, xTe[10,:], linestyle='-', color='#0000ff', label=fr'Estimated $\dot y^e$')

    if xTe_nanbins: # if estimated nan bins
        for bin in xTe_nanbins: # add bins to plot
            ax.axvspan(t[bin[0]], t[bin[1]], color='#0000ff', alpha=0.4)

    ax.set_xlabel(r'Time ($s$)')
    ax.set_ylabel(r'Global $y$-velocity ($m/s$)')
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.grid()

    ax = fig.add_subplot(3, 1, 3)
    
    ax.plot(t, xT[11,:], linestyle='-', color='#ff0000', label=fr'True $\dot z$')
    ax.plot(t, xTe[11,:], linestyle='-', color='#0000ff', label=fr'Estimated $\dot z^e$')

    if xTe_nanbins: # if estimated nan bins
        for bin in xTe_nanbins: # add bins to plot
            ax.axvspan(t[bin[0]], t[bin[1]], color='#0000ff', alpha=0.4)

    ax.set_xlabel(r'Time ($s$)')
    ax.set_ylabel(r'Global $z$-velocity ($m/s$)')
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.grid()

    # return figure
    return fig

##### PLOTTING DRONE POSITION AND REFERENCE FUNCTION (plots drone position and yaw with dead zones)
def PlotDroneReference(t: np.ndarray, data: list) -> plt.Figure:
    print(f'\n=====PLOTTING DRONE REFERENCE=====')

    # extracting data
    x = data[0]
    x0 = data[1]
    xTe = data[2]

    xTe_nanidx = [idx for idx, x in enumerate(xTe[3,:]) if np.isnan(x)] # all values where estimate is nan

    xTe_nanbins = PlotVBinIdx(idx=xTe_nanidx) # estimated nan bins

    # creating figure
    fig = plt.figure(f'Drone Reference Plot', figsize=(7, 5))
    fig.suptitle(fr'Drone State, Reference, and Estimated Target State ($x$, $x_0$, and $x_T^e$) versus Time')

    # plotting data
    ax = fig.add_subplot(2, 2, 1)

    ax.plot(t, x[3,:], linestyle='-', color='#ff0000', label=fr'Drone $x$')
    ax.plot(t, x0[3,:], linestyle='-', color='#00aa00', label=fr'Reference $x$')
    ax.plot(t, xTe[3,:], linestyle='-', color='#0000ff', label=fr'Estimated Target $x^e$')

    if xTe_nanbins: # if estimated nan bins
        for bin in xTe_nanbins: # add bins to plot
            ax.axvspan(t[bin[0]], t[bin[1]], color='#0000ff', alpha=0.4)

    ax.set_xlabel(r'Time ($s$)')
    ax.set_ylabel(r'Global $x$-coordinate ($m$)')
    # ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.legend(loc='best')
    ax.grid()

    ax = fig.add_subplot(2, 2, 2)
    
    ax.plot(t, x[4,:], linestyle='-', color='#ff0000', label=fr'Drone $y$')
    ax.plot(t, x0[4,:], linestyle='-', color='#00aa00', label=fr'Reference $y$')
    ax.plot(t, xTe[4,:], linestyle='-', color='#0000ff', label=fr'Estimated Target $y^e$')

    if xTe_nanbins: # if estimated nan bins
        for bin in xTe_nanbins: # add bins to plot
            ax.axvspan(t[bin[0]], t[bin[1]], color='#0000ff', alpha=0.4)

    ax.set_xlabel(r'Time ($s$)')
    ax.set_ylabel(r'Global $y$-coordinate ($m$)')
    # ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.legend(loc='best')
    ax.grid()

    ax = fig.add_subplot(2, 2, 3)
    
    ax.plot(t, x[5,:], linestyle='-', color='#ff0000', label=fr'Drone $z$')
    ax.plot(t, x0[5,:], linestyle='-', color='#00aa00', label=fr'Reference $z$')
    ax.plot(t, xTe[5,:], linestyle='-', color='#0000ff', label=fr'Estimated Target $z^e$')

    if xTe_nanbins: # if estimated nan bins
        for bin in xTe_nanbins: # add bins to plot
            ax.axvspan(t[bin[0]], t[bin[1]], color='#0000ff', alpha=0.4)

    ax.set_xlabel(r'Time ($s$)')
    ax.set_ylabel(r'Global $z$-coordinate ($m$)')
    # ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.legend(loc='best')
    ax.grid()

    ax = fig.add_subplot(2, 2, 4)
    
    ax.plot(t, x[2,:], linestyle='-', color='#ff0000', label=fr'Drone $\psi$')
    ax.plot(t, x0[2,:], linestyle='-', color='#00aa00', label=fr'Reference $\psi$')

    if xTe_nanbins: # if estimated nan bins
        for bin in xTe_nanbins: # add bins to plot
            ax.axvspan(t[bin[0]], t[bin[1]], color='#0000ff', alpha=0.4)

    ax.set_xlabel(r'Time ($s$)')
    ax.set_ylabel(r'Global $\psi$-coordinate (yaw) ($rad$)')
    # ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.legend(loc='best')
    ax.grid()

    # return figure
    return fig

##### PLOTTING DATA MAIN SCRIPT (plots all data and saves)
if __name__ == '__main__':
    ScriptInit() # initialize the script with arguments

    # loading all data
    t_data = np.load(str(config.DAT_DIR / 't_log.npy'))
    x_data = np.load(str(config.DAT_DIR / 'x_log.npy'))
    xT_data = np.load(str(config.DAT_DIR / 'xT_log.npy'))
    x0_data = np.load(str(config.DAT_DIR / 'x0_log.npy'))
    u_data = np.load(str(config.DAT_DIR / 'u_log.npy'))
    xTe_data = np.load(str(config.DAT_DIR / 'xTe_log.npy'))
    pT_data = np.load(str(config.DAT_DIR / 'pT_log.npy'))
    pTe_data = np.load(str(config.DAT_DIR / 'pTe_log.npy'))

    # creating plots
    fig_drone_reference = PlotDroneReference(t=t_data[0,:], data=[x_data, x0_data, xTe_data]) # drone reference plots
    fig_target_positions = PlotTargetPositions(t=t_data[0,:], data=[xT_data, xTe_data]) # target state plots
    fig_target_velocities = PlotTargetVelocities(t=t_data[0,:], data=[xT_data, xTe_data]) # target state plots
    fig_pixels = PlotTargetPixels(t=t_data[0,:], data=[pT_data, pTe_data]) # target pixel plots

    # saving plots
    fig_drone_reference.savefig(str(config.FIG_DIR / 'drone_reference'), dpi=300, bbox_inches='tight')
    fig_target_positions.savefig(str(config.FIG_DIR / 'target_positions'), dpi=300, bbox_inches='tight')
    fig_target_velocities.savefig(str(config.FIG_DIR / 'target_velocities'), dpi=300, bbox_inches='tight')
    fig_pixels.savefig(str(config.FIG_DIR / 'target_pixels'), dpi=300, bbox_inches='tight')

    # showing all plots
    plt.show()
    