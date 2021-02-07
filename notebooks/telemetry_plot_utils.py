# helper functions for plotting telemetry
import pickle
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable
import glob
from pathlib import Path

# load pickled data from robot telemetry output - it's one big dictionary as below
# out_dict = {'TIMESTAMP':timestamp,'DATA':self.telemetry, 'COURSE':self.course,
#                             'KP_VEL':self.kp_vel, 'KD_VEL':self.kd_vel, 'BETA':self.beta, 'ZETA':self.zeta}
def load_file(infile):
    try:
        with open(infile, 'rb') as fp:
            telemetry = pickle.load(fp)
    except ValueError as e:
        print(f'Missing item  in pickle {infile}, please create it and re-pickle')
    return telemetry

def get_data(file_name=None, x_offset=0, y_offset=0):
    files = glob.glob('../robot/sim/data/*.pkl')
    #print(files)
    if file_name is None:  # get the latest file
        telemetry = load_file(files[-1])
    elif type(file_name) == int:
        telemetry = load_file(files[file_name])
    else:  # try and match the filename passed in
        matches = [file for file in files if file_name in file]
        telemetry = load_file(matches[-1])
    df = fix_data(telemetry, x_offset=x_offset, y_offset=y_offset)
    return df, telemetry

def get_points_df(name='slalom', data=None, x_shift=0, y_shift=4.87-0.25):
    # get a list of all the points from a pathweaver file
    path_files = glob.glob('../robot/pathweaver/paths/*')
    matches = [file for file in path_files if name.lower() in file.lower()]

    if data is None:  # get the data from a file
        df_points = pd.DataFrame()
        if len(matches) > 0:
            df_points = pd.read_csv(matches[0], sep=',', header='infer')
            # fix the pathweaver offsets for the 2021 fields
            df_points['X'] = df_points['X'] + x_shift
            df_points['Y'] = df_points['Y'] + y_shift
        return df_points
    else:  # pass the data in
        pass

def get_points_df_from_trajectory_data_file(trajectory_data_file_name=None, data=None, x_shift=0, y_shift=4.87-0.25):
    # see if any of the paths are matches for the trajectory file name
    path_files = glob.glob('../robot/pathweaver/paths/*')
    path_file_names = [Path(file).name for file in path_files]

    matches = [file for file in path_files if Path(file).name in trajectory_data_file_name]

    if data is None:  # get the data from a file
        df_points = pd.DataFrame()
        if len(matches) > 0:
            df_points = pd.read_csv(matches[0], sep=',', header='infer')
            # fix the pathweaver offsets for the 2021 fields
            df_points['X'] = df_points['X'] + x_shift
            df_points['Y'] = df_points['Y'] + y_shift
            return df_points
        else:
            return None
    else:  # pass the data in ? maybe I don't need this option
        pass


# take data, massage it to get what we need to plot
def fix_data(telemetry, x_offset=0, y_offset=0):
    df = pd.DataFrame(telemetry['DATA'], columns=telemetry['DATA'][0].keys())
    df['DELTA'] = np.sqrt(np.asarray(df.diff()['RBT_X']**2) + np.asarray(df.diff()['RBT_Y']**2))
    df['RBT_X'] = df['RBT_X'] + x_offset
    df['RBT_Y'] = df['RBT_Y'] + y_offset
    df['TRAJ_X'] = df['TRAJ_X'] + x_offset
    df['TRAJ_Y'] = df['TRAJ_Y'] + y_offset
    # df['RADIANS'] = df['POSE_ROT'] * np.pi/180
    df['VEC_X'] = df['DELTA']* np.cos(df['RBT_TH'])
    df['VEC_Y'] = df['DELTA']* np.sin(df['RBT_TH'])
    df.at[0, 'VEC_X'] = df.at[1, 'VEC_X']
    df.at[0, 'VEC_Y'] = df.at[1, 'VEC_Y']
    df.at[0, 'DELTA'] = df.at[1, 'DELTA']
    df[(df.index) % 20 == 0]
    return df


# ----------------- PLOT UTILS -----------------
def plot_df(df, telemetry, arrows=True, guess_points=True, point_df=None, pathweaver=False, background='slalom', save=False, fname='odometry.png'):

    # generate a title
    label = f"Mapping odometry from {telemetry['COURSE']}: {telemetry['TIMESTAMP']} " \
            f"(vel = {telemetry['VELOCITY']}, kp_vel={telemetry['KP_VEL']:2.1f}, " \
            f"beta={telemetry['BETA']:2.1f}, zeta={telemetry['ZETA']:2.1f})"

    fig, ax = plt.subplots(figsize=(12, 10), dpi=100)
    font_size = 12
    # background image
    # ToDo - have this search for and grab the right image instead of getting it exact
    img = plt.imread('../robot/sim/2021-' + background + '.png')
    field_offset = 0.25
    ax.imshow(img, extent=[0-field_offset, 9.114+field_offset, 0-field_offset, 4.572+field_offset])

    # robot scatterplot and annotation
    scat = ax.scatter(x=df['RBT_X'], y=df['RBT_Y'], c=df['RBT_TH'], s=20, label='robot')

    if arrows:
        x = np.array(df['RBT_X'])
        y = np.array(df['RBT_Y'])
        ax.quiver(x, y, df['VEC_X'], df['VEC_Y'], df['RBT_TH'], headwidth=2, width=0.002)

    ax.text(df.iloc[[0]]['RBT_X'], df.iloc[[0]]['RBT_Y'] - .2, 'START', ha='center', size=font_size)
    ax.text(df.iloc[[-1]]['RBT_X'], df.iloc[[-1]]['RBT_Y'] + .2, 'FINISH', ha='center', size=font_size)

    # trajectory data
    traj = ax.scatter(x=df['TRAJ_X'], y=df['TRAJ_Y'], c=df['TRAJ_TH'], marker='x', label='trajectory')

    # trajectory generation data
    if guess_points:
        point_df = get_points_df_from_trajectory_data_file(trajectory_data_file_name=telemetry['COURSE'])

    if point_df is not None:
        labels = [str(i) for i in range(len(point_df))]
        x, y = np.asarray(point_df['X']), np.asarray(point_df['Y'])
        for i, txt in enumerate(labels):
            ax.annotate(txt, (x[i] - .1, y[i] - .3), size=font_size)
        ax.scatter(x, y, marker='o', c='r', s=120, linewidths=2, label='path point')

    # make a color bar to help visualize direction
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="3%", pad=0.05)
    cb = plt.colorbar(scat, cax=cax)
    cb.set_label('robot theta', rotation=270)

    # legends and labels
    ax.legend(loc='upper right', bbox_to_anchor=(0.97, 0.95), fontsize=font_size-2)
    leg = ax.get_legend()
    #leg.legendHandles[2].set_color('yellow')
    leg.legendHandles[2]._sizes = [50]
    ax.set_title(label, fontsize=font_size)
    ax.set_ylabel('y position on field', fontsize=font_size)
    ax.set_xlabel('x position on field', fontsize=font_size)

    plt.tight_layout()
    if save:
        plt.ioff()
        plt.savefig(fname, facecolor='w', bbox_inches='tight', dpi=100)
        plt.close()
    plt.show()

def velocity_plot(df):
    fig, (ax,ax2) = plt.subplots(2,1, figsize=(16, 6))
    # robot velocity scatterplot and annotation
    ax.plot(df['TIME'], df['RBT_VEL'], c='g', linestyle='-', label='robot vel')
    ax.plot(df['TIME'], df['RBT_RVEL'], c='g', linestyle='--', label='robot rvel')
    ax.plot(df['TIME'], df['RBT_LVEL'], c='g', linestyle='dotted', label='robot lvel')
    ax.plot(df['TIME'], df['RAM_VELX'], c='b', label='ramsete vel')
    ax.plot(df['TIME'], df['RAM_RVEL_SP'], c='b', linestyle='--', label='ramsete rsp')
    ax.plot(df['TIME'], df['RAM_LVEL_SP'], c='b', linestyle='dotted', label='ramsete lsp')

    #ax2.plot(df['TIME'], df['TRAJ_VEL'], c='b', label='traj vel')
    ax2.plot(df['TIME'], df['LFF'], c='r', linestyle='-', label='rff')
    ax2.plot(df['TIME'], df['RFF'], c='c', linestyle='-', label='lff')
    ax2.plot(df['TIME'], df['RPID'], c='r', linestyle='--', label='rpid')
    ax2.plot(df['TIME'], df['LPID'], c='c', linestyle='--', label='lpid')

    ax.set_ylabel('velocity (m/s)', fontsize=14)
    ax2.set_ylabel('motor output (V)', fontsize=14)
    ax2.set_ylim(-12,12)
    ax2.set_xlabel('time in command (s)', fontsize=14)
    ax.get_xaxis().set_visible(False)

    ax.legend(loc='lower right')
    ax2.legend(loc='lower left')
    plt.tight_layout()













