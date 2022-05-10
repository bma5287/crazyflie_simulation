"""
Generate custom data that will box-in NN policy region to only cover collected data and no outside regions
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import sys
import os
import rospkg
import plotly.graph_objects as go



## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_projects'))
sys.path.insert(0,BASE_PATH)

DATA_PATH = f"{BASE_PATH}/crazyflie_projects/Policy_Mapping/Data_Logs/NL_Raw/NL_LR_Trials_Raw.csv"
df = pd.read_csv(DATA_PATH)

## SYSTEM CONSTANTS
Tau_max = 0.29 # Cutoff value for plots (Practically contact at this point (0.125s))
Tau_min = 0.17

## GENERATE MAX VELOCITY CURVE
theta_range = np.linspace(10,90,15)
vel_range = np.linspace(4,5,2)
Tau_range = np.linspace(0.2,0.3,3)
vel_range,theta_range,Tau_range = np.meshgrid(vel_range,theta_range,Tau_range)
vx_range = vel_range.flatten()*np.cos(np.radians(theta_range.flatten()))
vz_range = vel_range.flatten()*np.sin(np.radians(theta_range.flatten()))
Tau_range = Tau_range.flatten()

d_range = vz_range*Tau_range
OFy_range = -vx_range/d_range
LR_vmax = np.zeros_like(d_range)
My_vmax = np.zeros_like(d_range)

data_vmax = np.stack((OFy_range.flatten(),Tau_range.flatten(),d_range.flatten(),LR_vmax.flatten(),My_vmax.flatten()),axis=1)


## GENERATE MAX TAU WALL
OFy_range = np.linspace(-20,0,11)
d_range = np.linspace(0,1.5,11)
Tau_range = np.linspace(Tau_max,Tau_max+0.05,2)
Tau_range,OFy_range,d_range = np.meshgrid(Tau_range,OFy_range,d_range)
LR_wall = np.zeros_like(OFy_range)
My_wall = np.zeros_like(OFy_range)
data_wall_max = np.vstack((OFy_range.flatten(),Tau_range.flatten(),d_range.flatten(),LR_wall.flatten(),My_wall.flatten())).T

## GENERATE MIN TAU WALL
OFy_range = np.linspace(-20,0,11)
d_range = np.linspace(0,1.5,11)
Tau_range = np.linspace(Tau_min,Tau_min-0.05,2)
Tau_range,OFy_range,d_range = np.meshgrid(Tau_range,OFy_range,d_range)
LR_wall = np.zeros_like(OFy_range)
My_wall = np.zeros_like(OFy_range)
data_wall_min = np.vstack((OFy_range.flatten(),Tau_range.flatten(),d_range.flatten(),LR_wall.flatten(),My_wall.flatten())).T

## SAVE MANUFACTURED DATA TO CSV
data = np.vstack((data_vmax,data_wall_max,data_wall_min))
df_custom = pd.DataFrame(data,columns=["OFy_flip_mean","Tau_flip_mean","D_ceil_flip_mean","LR_4leg","My_mean"])
df_custom = df_custom.round(4)
df_custom.to_csv(f"{BASE_PATH}/crazyflie_projects/Policy_Mapping/Data_Logs/NL_Raw/Boundary.csv",index=False)




def plot_Cartesian(df,df_custom):


    fig = go.Figure()

    ## PLOT DATA POINTS
    fig.add_trace(
        go.Scatter3d(
            ## DATA
            x=-df_custom["OFy_flip_mean"]*df_custom["D_ceil_flip_mean"],
            y=df_custom["D_ceil_flip_mean"]/df_custom["Tau_flip_mean"],
            z=df_custom["D_ceil_flip_mean"],         

            ## MARKER
            mode='markers',
            marker=dict(
                size=3,
                color=df_custom["LR_4leg"],
                cmin=0,
                cmax=1,
                colorscale='Viridis',   # choose a colorscale
                opacity=1.0
            )
        )
    )
    
    fig.add_trace(
        go.Scatter3d(
            
            ## DATA
            x=-df["OFy_flip_mean"]*df["D_ceil_flip_mean"],
            y=df["D_ceil_flip_mean"]/df["Tau_flip_mean"],
            z=df["D_ceil_flip_mean"],         

            ## MARKER
            mode='markers',
            marker=dict(
                size=3,
                color=df["LR_4leg"],
                cmin=0,
                cmax=1,
                showscale=True,
                colorscale='Viridis',   # choose a colorscale
                opacity=1.0
            )
        )
    )

    fig.update_layout(
        scene=dict(
            xaxis_title='OFy [rad/s]',
            yaxis_title='Tau [s]',
            zaxis_title='D_ceiling [m]',
            xaxis_range=[-0.5,4],
            yaxis_range=[-0.5,4],
            zaxis_range=[0,1.5],
        ),
    )
    fig.show()

def plot_OFspace(df,df_custom):


    fig = go.Figure()

    ## PLOT DATA POINTS
    fig.add_trace(
        go.Scatter3d(
            ## DATA
            x=df_custom["OFy_flip_mean"],
            y=df_custom["Tau_flip_mean"],
            z=df_custom["D_ceil_flip_mean"],         

            ## MARKER
            mode='markers',
            marker=dict(
                size=3,
                color=df_custom["LR_4leg"],
                cmin=0,
                cmax=1,
                colorscale='Viridis',   # choose a colorscale
                opacity=1.0
            )
        )
    )
    
    fig.add_trace(
        go.Scatter3d(
            
            ## DATA
            x=df["OFy_flip_mean"],
            y=df["Tau_flip_mean"],
            z=df["D_ceil_flip_mean"],         

            ## MARKER
            mode='markers',
            marker=dict(
                size=3,
                color=df["LR_4leg"],
                cmin=0,
                cmax=1,
                showscale=True,
                colorscale='Viridis',   # choose a colorscale
                opacity=1.0
            )
        )
    )

    fig.update_layout(
        scene=dict(
            xaxis_title='OFy [rad/s]',
            yaxis_title='Tau [s]',
            zaxis_title='D_ceiling [m]',
            xaxis_range=[-20,1],
            yaxis_range=[0.4,0.1],
            zaxis_range=[0,1.2],
        ),
    )
    fig.show()

# df = df.query("LR_4leg >= 0.9")
plot_Cartesian(df,df_custom)
plot_OFspace(df,df_custom)





# ## PLOT SETUP
# cmap = mpl.cm.jet
# norm = mpl.colors.Normalize(vmin=0,vmax=1)

# fig = plt.figure()
# ax1 = fig.add_subplot(121,projection='3d')
# ax1.scatter(df["vx_flip_mean"],df["vz_flip_mean"],df["D_ceil_flip_mean"],c=df["LR_4leg"],cmap=cmap,norm=norm)
# # ax1.scatter(vx_range,vz_range,d_range)

# ax1.set_xlim(0,4)
# ax1.set_ylim(0,4)
# ax1.set_zlim(0,1.8) 

# ax1.set_xlabel("Vel_x [m/s]")
# ax1.set_ylabel("Vel_z [m/s]")
# ax1.set_zlabel("d_ceil [m]")

# ax2 = fig.add_subplot(122,projection='3d')
# ax2.scatter(df["OFy_flip_mean"],df["Tau_flip_mean"],df["D_ceil_flip_mean"],c=df["LR_4leg"],cmap=cmap,norm=norm)
# ax2.scatter(data_wall_max[:,0],data_wall_max[:,1],data_wall_max[:,2])
# ax2.scatter(data_wall_min[:,0],data_wall_min[:,1],data_wall_min[:,2])
# ax2.scatter(data_vmax[:,0],data_vmax[:,1],data_vmax[:,2])



# ax2.set_xlim(-15,0)
# ax2.set_ylim(0.4,0.1)
# ax2.set_zlim(0,1.8) 

# ax2.set_xlabel("OFy [rad/s]")
# ax2.set_ylabel("Tau [s]")
# ax2.set_zlabel("d_ceil [m")

# plt.show()