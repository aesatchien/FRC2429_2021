# helper functions for plotting telemetry
import pickle
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable
import glob


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


def plot_df(df, telemetry, arrows=True, point_df=None, background='slalom', save=False, fname='odometry.png'):

    # generate a title
    label = f"Mapping odometry from {telemetry['COURSE']}: {telemetry['TIMESTAMP']} " \
            f"(vel = {telemetry['VELOCITY']}, kp_vel={telemetry['KP_VEL']:2.1f}, " \
            f"beta={telemetry['BETA']:2.1f}, zeta={telemetry['ZETA']:2.1f}"

    fig, ax = plt.subplots(figsize=(18, 10))

    # background image
    # ToDo - have this search for and grab the right image instead of getting it exact
    img = plt.imread('../robot/sim/2021-' + background + '.png')
    ax.imshow(img, extent=[0, 9.63, 0, 5.05])

    # robot scatterplot and annotation
    scat = ax.scatter(x=df['RBT_X'], y=df['RBT_Y'], c=df['RBT_TH'], label='robot')

    if arrows:
        x = np.array(df['RBT_X'])
        y = np.array(df['RBT_Y'])
        ax.quiver(x, y, df['VEC_X'], df['VEC_Y'], df['RBT_TH'])

    plt.text(df.iloc[[0]]['RBT_X'], df.iloc[[0]]['RBT_Y'] - .2, 'START', ha='center', size=16)
    plt.text(df.iloc[[-1]]['RBT_X'], df.iloc[[-1]]['RBT_Y'] + .2, 'FINISH', ha='center', size=16)

    # trajectory data
    traj = ax.scatter(x=df['TRAJ_X'], y=df['TRAJ_Y'], c=df['TRAJ_TH'], marker='x', label='trajectory')

    # trajectory generation data
    if point_df is not None:
        labels = [str(i) for i in range(len(point_df))]
        x, y = np.asarray(point_df['X']), 4.87+np.asarray(point_df['Y'])
        for i, txt in enumerate(labels):
            ax.annotate(txt, (x[i] - .1, y[i] - .3), size=20)
        plt.scatter(x, y, marker='o', c='r', s=150, linewidths=3)

    # make a color bar to help visualize direction
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="3%", pad=0.05)
    cb = plt.colorbar(scat, cax=cax)
    cb.set_label('robot theta', rotation=270)

    # legends and labels
    ax.legend(loc='upper right', bbox_to_anchor=(0.95, 0.95), fontsize=14)
    ax.set_title(label, fontsize=16)
    ax.set_ylabel('y position on field', fontsize=16)
    ax.set_xlabel('x position on field', fontsize=16)
    plt.tight_layout()
    if save:
        plt.ioff()
        plt.savefig(fname, facecolor='w', bbox_inches='tight', dpi=100)
        plt.close()
    plt.show()

def get_data(file_name=None, x_offset=1.2, y_offset=0.9):
    files = glob.glob('../robot/sim/data/*')
    if file_name is None:  # get the latest file
        telemetry = load_file(files[-1])
    else:  # try and match the filename passed in
        matches = [file for file in files if file_name in file]
        telemetry = load_file(matches[0])
    df = fix_data(telemetry, x_offset=x_offset, y_offset=y_offset)
    return df, telemetry

def get_points_df(name='slalom', x_shift=0, y_shift=4.87 ):
    # get a list of all the points from a pathweaver file
    path_files = glob.glob('../robot/paths/*')
    matches = [file for file in path_files if name.lower() in file.lower()]
    df_points = pd.DataFrame()
    if len(matches) > 0:
        df_points = pd.read_csv(matches[0], sep=',', header='infer')
        # fix the pathweaver offsets for the 2021 fields
        df_points['X'] = df_points['X']
        df_points['Y'] = df_points['Y'] + y_shift
    return df_points












