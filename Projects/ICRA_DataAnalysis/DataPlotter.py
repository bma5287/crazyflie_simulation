## IMPORT MODULES
import pandas as pd
import numpy as np
from sklearn import linear_model
from scipy.interpolate import griddata
import scipy.odr

## IMPORT PLOTTING MODULES
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import matplotlib as mpl
from matplotlib import cm


## FULL DATAFRAME
df_raw = pd.read_csv("Projects/ICRA_DataAnalysis/Wide-Long_2-Policy/Wide-Long_2-Policy_Summary.csv")


## MAX LANDING RATE DATAFRAME
idx = df_raw.groupby(['vel_IC','phi_IC'])['landing_rate_4_leg'].transform(max) == df_raw['landing_rate_4_leg']
df_max = df_raw[idx].reset_index()






# ======================================
##        LANDING RATE DATA (RAW)
# ======================================

def plot_raw(dataName:str,color_data:str='landing_rate_4_leg',color_str=None,color_norm=None,polar=True):

    
    ## INITIALIZE FIGURE
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")

    ## DEFINE PLOT VARIABLES
    if polar==True:
        X = df_raw['vx']
        Y = df_raw['vz']

        ax.set_xlabel('Vel_x (m/s)')
        ax.set_ylabel('Vel_z (m/s)')

    else:
        X = df_raw['phi_IC']
        Y = df_raw['vel_IC']

        ax.set_xlabel('phi (deg)')
        ax.set_ylabel('Vel (m/s)')

        
    Z = df_raw[dataName]
    C = df_raw[color_data]

    ## ADJUST COLOR SCALING AND LABELS
    if color_str == None:
        color_str = color_data

    if color_norm == None:
        vmin = np.min(C)
        vmax = np.max(C)
    else:
        vmin = color_norm[0]
        vmax = color_norm[1]

    cmap = mpl.cm.jet
    norm = mpl.colors.Normalize(vmin=vmin,vmax=vmax)
    
    # CREATE PLOTS AND COLOR BAR
    ax.scatter(X,Y,Z,c=C,cmap=cmap,norm=norm)
    fig.colorbar(mpl.cm.ScalarMappable(cmap=cmap,norm=norm),label=color_str)


    # PLOT LIMITS AND INFO
    ax.set_zlabel(dataName)
    ax.set_title("Raw Data")
    fig.tight_layout()


def plot_best(dataName:str,color_data:str='landing_rate_4_leg',color_str=None,color_norm=None,polar=True):

    
    ## INITIALIZE FIGURE
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")

    ## DEFINE PLOT VARIABLES
    if polar==True:
        X = df_max['vx']
        Y = df_max['vz']

        ax.set_xlabel('Vel_x (m/s)')
        ax.set_ylabel('Vel_z (m/s)')

    else:
        X = df_max['phi_IC']
        Y = df_max['vel_IC']

        ax.set_xlabel('phi (deg)')
        ax.set_ylabel('Vel (m/s)')

        
    Z = df_max[dataName]
    C = df_max[color_data]

    ## ADJUST COLOR SCALING AND LABELS
    if color_str == None:
        color_str = color_data

    if color_norm == None:
        vmin = np.min(C)
        vmax = np.max(C)
    else:
        vmin = color_norm[0]
        vmax = color_norm[1]

    cmap = mpl.cm.jet
    norm = mpl.colors.Normalize(vmin=vmin,vmax=vmax)
    
    # CREATE PLOTS AND COLOR BAR
    ax.scatter(X,Y,Z,c=C,cmap=cmap,norm=norm)
    fig.colorbar(mpl.cm.ScalarMappable(cmap=cmap,norm=norm),label=color_str)


    # PLOT LIMITS AND INFO
    ax.set_zlabel(dataName)
    ax.set_title("Best Data")
    fig.tight_layout()





# ======================================
##   LANDING RATE SURFACE (BEST POLICY)
# ======================================
def plot_best_surface(dataName:str,color_data:str='landing_rate_4_leg',color_str=None,color_norm=None,polar=True):
    
    ## INITIALIZE FIGURE
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")

    ## DEFINE PLOT VARIABLES
    if polar==True:
        X = df_max['vx']
        Y = df_max['vz']

        ax.set_xlabel('Vel_x (m/s)')
        ax.set_ylabel('Vel_z (m/s)')

    else:
        X = df_max['phi_IC']
        Y = df_max['vel_IC']

        ax.set_xlabel('phi (deg)')
        ax.set_ylabel('Vel (m/s)')

        
    Z = df_max[dataName]
    C = df_max[color_data]

    ## ADJUST COLOR SCALING AND LABELS
    if color_str == None:
        color_str = color_data

    if color_norm == None:
        vmin = np.min(C)
        vmax = np.max(C)
    else:
        vmin = color_norm[0]
        vmax = color_norm[1]

    cmap = mpl.cm.jet
    norm = mpl.colors.Normalize(vmin=vmin,vmax=vmax)


    # CREATE PLOTS AND COLOR BAR
    ax.plot_trisurf(X,Y,Z,cmap=cmap,norm=norm,edgecolors='grey',linewidth=0.25)
    fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label=color_str)


    # PLOT LIMITS AND INFO
    ax.set_xlabel('X-Velocity (m/s)')
    ax.set_ylabel('Z-Velocity (m/s)')
    ax.set_zlabel(dataName)
    ax.set_title('Surface Best Data')



def plot_best_surface_smoothed(dataName:str,color_data:str='landing_rate_4_leg',color_str=None,color_norm=None,polar=True):
    ## INITIALIZE FIGURE
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")

    ## DEFINE PLOT VARIABLES
    if polar==True:
        X = df_max['vx']
        Y = df_max['vz']

        ax.set_xlabel('Vel_x (m/s)')
        ax.set_ylabel('Vel_z (m/s)')

    else:
        X = df_max['phi_IC']
        Y = df_max['vel_IC']

        ax.set_xlabel('phi (deg)')
        ax.set_ylabel('Vel (m/s)')

        
    Z = df_max[dataName]
    C = df_max[color_data]

    ## ADJUST COLOR SCALING AND LABELS
    if color_str == None:
        color_str = color_data

    if color_norm == None:
        vmin = np.min(C)
        vmax = np.max(C)
    else:
        vmin = color_norm[0]
        vmax = color_norm[1]

    # SOMETHING ABOUT DEFINING A GRID
    xi = np.linspace(X.min(),X.max(),(len(Z)//7))
    yi = np.linspace(Y.min(),Y.max(),(len(Z)//7))
    zi = griddata((X, Y), Z, (xi[None,:], yi[:,None]), method='linear')
    xig, yig = np.meshgrid(xi, yi)

    # DEFINE COLOR FORMAT AND LIMITS
    cmap = mpl.cm.jet
    norm = mpl.colors.Normalize(vmin=vmin,vmax=vmax)

    # CREATE PLOTS AND COLOR BAR
    surf = ax.plot_surface(xig, yig, zi,cmap=cmap,norm=norm,edgecolors='grey',linewidth=0.25)
    # surf = ax.plot_surface(xig, yig, zi,cmap=cmap,norm=norm)
    # surf = ax.contour(xig, yig, zi,cmap=cmap,norm=norm)

    fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label=color_str)

    # PLOT LIMITS AND INFO
    ax.set_zlabel(dataName)
    ax.set_title('Smoothed Best Data Surface')


# plt.show()

if __name__ == '__main__':
    # plot_raw(dataName='landing_rate_4_leg',color_data='landing_rate_4_leg',polar=True)
    # plot_best(dataName='landing_rate_4_leg',color_data='landing_rate_4_leg',polar=True)
    plot_best_surface(dataName='landing_rate_4_leg',color_data='landing_rate_4_leg',polar=True)
    plot_best_surface_smoothed(dataName='RREV_threshold',color_data='RREV_threshold',polar=False)


    plt.show()