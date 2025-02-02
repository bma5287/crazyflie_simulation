## STANDARD IMPORTS
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import os

## MISC IMPORTS
import re

# os.system("clear")
np.set_printoptions(suppress=True)
mpl.rcParams['pdf.fonttype'] = 42
mpl.rcParams['ps.fonttype'] = 42

class Data_Parser:
    def __init__(self,dataPath,fileName,dataType='SIM'):

        ## ORGANIZE FILEPATH AND CREATE TRIAL DATAFRAME
        self.fileName = fileName
        self.dataPath = dataPath
        filepath = os.path.join(self.dataPath, self.fileName)

        self.trial_df = pd.read_csv(filepath,low_memory=False,comment="#")

        ## CLEAN UP TRIAL DATAFRAME
        # Drop row with "Note: ____________"
        self.dataType = dataType

        # Remove rows past final complete rollout
        final_valid_index = self.trial_df[self.trial_df['Error']=='Impact Data'].index.values[-1]
        self.trial_df = self.trial_df.iloc[:final_valid_index+1]

        # Round values to prevent floating point precision issues
        self.trial_df = self.trial_df.round(3) 

        # Convert string bools to actual bools
        self.trial_df = self.trial_df.replace({'False':False,'True':True})
        self.trial_df = self.trial_df.replace({'--':np.nan,'nan':np.nan})


        ## CREATE BASIC DF OF K_EP,K_RUN, & REWARD
        self.k_df = self.trial_df.iloc[:][['K_ep','K_run']]
        self.k_df = self.k_df.drop_duplicates()

        if self.dataType=='EXP':
            self.remove_FailedRuns()

        ## COLLECT BASIC TRIAL INFO
        self.n_rollouts = int(self.trial_df.iloc[-3]['a_Trg'])
        self.k_epMax = int(self.trial_df.iloc[-3]['K_ep'])
        self.k_runMax = int(self.trial_df.iloc[-3]['K_run'])

    def remove_FailedRuns(self):
        """Drop all rollouts in dataframe that were labelled as 'Failed Rollout'
        """        

        failedRuns_list = self.trial_df.query(f"Error=='Failed Rollout'").index.tolist()

        for failed_run in failedRuns_list[::-1]: # Work backwards down list

            idx_prev_run = self.k_df.query(f"index == {failed_run}").index.tolist()[0]-1
            if idx_prev_run < 0: # Catch for very first run
                idx_prev_run = 0

            idx_start = self.k_df.iloc[idx_prev_run]['index'] + 1 # True index of trial_df to start drop
            idx_end = failed_run + 1 # Make sure to include endpoint

            self.trial_df.drop(np.arange(idx_start,idx_end,dtype='int'),inplace=True)

        self.k_df = self.trial_df.iloc[:][['K_ep','K_run']].dropna()
        self.k_df.reset_index(inplace=True)

    def select_run(self,K_ep,K_run): ## Create run dataframe from K_ep and K_run
        """Create run dataframe from selected K_ep and K_run

        Args:
            K_ep (int): Episode number
            K_run (int): Run number

        Returns:
            Experiment:
                tuple: run_df,IC_df,__,__
            Sim:
                tuple: run_df,IC_df,Rot_df,impact_df
        """        

        ## CREATE RUN DATAFRAME
        run_df = self.trial_df[(self.trial_df['K_ep']==K_ep) & (self.trial_df['K_run']==K_run)]

        IC_df = run_df.iloc[[-3]]       # Create DF of initial conditions
        Rot_df = run_df.iloc[[-2]]     # Create DF of Rot conditions
        impact_df = run_df.iloc[[-1]]   # Create DF of impact conditions
        run_df = run_df[:-3]            # Drop special rows from dataframe

        return run_df,IC_df,Rot_df,impact_df

    def grab_rewardData(self):
        """Returns summary of rewards
        
        Data Type: Sim/Exp
            
        Returns:
            tuple: k_ep_r,rewards,k_ep_ravg,rewards_avg

            ===========================
            k_ep_r: reward episodes (np.array)
            rewards: array of rewards (np.array)
            k_ep_ravg: episode of averaged reward (np.array)
            rewards_avg: array of averaged rewards per episode (np.array)
        """        
        ## CREATE ARRAYS FOR REWARD, K_EP 
        reward_df = self.trial_df.iloc[:][['K_ep','mu','Trg_flag']].dropna() # Create df from K_ep/rewards and drop blank reward rows
        reward_df = reward_df.iloc[:][["K_ep","Trg_flag"]].astype('float')
        rewards_arr = reward_df.to_numpy()
        rewards = rewards_arr[:,1]
        k_ep_r = rewards_arr[:,0]

        ## CREATE ARRAYS FOR REWARD_AVG, K_EP
        rewardAvg_df = reward_df.groupby('K_ep').mean() # Group rows by K_ep value and take their mean 
        rewards_avg = rewardAvg_df.to_numpy()
        k_ep_ravg = reward_df.K_ep.unique() # Drops duplicate values so it matches dimension of rewards_avg (1 avg per ep and n ep)

        return k_ep_r,rewards,k_ep_ravg,rewards_avg

    def plot_rewardData(self,ymax=300):
        """Plot rewards for entire trial
        Data Type: Sim/Exp
        """        
        k_ep_r,rewards,k_ep_ravg,rewards_avg = self.grab_rewardData()

        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.scatter(k_ep_r,rewards,marker='_',color='black',alpha=0.5,label='Reward')
        ax.scatter(k_ep_ravg,rewards_avg,marker='o',color='red',label='Average Reward')
        
        

        ax.set_ylabel("Reward")
        ax.set_xlabel("K_ep")
        ax.set_xlim(-2,self.k_epMax+5)
        ax.set_ylim(-2,ymax)
        ax.set_title(f"Reward vs Episode | Rollouts per Episode: {self.n_rollouts}")
        ax.legend()
        ax.grid()

        fig.tight_layout()
        plt.show()

    ## POLICY FUNCTIONS
    def grab_policy(self,K_ep=0,K_run=0):
        """Returns policy from specific run

        Data Type: Sim/Exp

        Args:
            K_ep (int): Episode number
            K_run (int): Run number

        Returns:
            np.array: [RREV,G1,G2,...]
        """        
        run_df,IC_df,_,_ = self.select_run(K_ep,K_run)

        ## SELECT POLICY
        policy = IC_df.iloc[0]['policy']
        policy = np.fromstring(policy[1:-1], dtype=float, sep=' ')  # Convert str to np array

        return policy

    def grab_convg_data(self,K_ep=0,K_run=0):
        """Returns series of arrays for episodes, mu values, and sigma values

        Data Type: Sim/Exp


        Returns:
           k_ep_arr [np.array]: Array of episodes
           mu_arr [np.array]: Array of mu values across trial
           sigma_arr [np.array]: Array of sigma values across trial

        """        
        ## CLEAN AND GRAB DATA FOR MU & SIGMA
        policy_df = self.trial_df.iloc[:][['K_ep','mu','sigma']]
        policy_df = policy_df.dropna().drop_duplicates()
        k_ep_arr = policy_df.iloc[:]['K_ep'].to_numpy()

        

        ## CREATE ARRAYS FOR MU & SIGMA OVER TRIAL
        mu_arr = []
        for mu in policy_df.iloc[:]['mu']:
            mu = np.fromstring(mu[1:-1], dtype=float, sep=' ')
            mu_arr.append(mu)
        mu_arr = np.array(mu_arr)

        sigma_arr = []
        for sigma in policy_df.iloc[:]['sigma']:
            sigma = np.fromstring(sigma[1:-1], dtype=float, sep=' ')
            sigma_arr.append(sigma)
        sigma_arr = np.array(sigma_arr)

        return k_ep_arr,mu_arr,sigma_arr

    def plot_policy_convg(self): ## NEEDS UPDATED
        """Creates subplots to show convergence for policy gains
        """      

        ## GRAB CONVERGENCE DATA
        k_ep_arr,mu_arr,sigma_arr = self.grab_convg_data()

        

        num_col = mu_arr.shape[1] # Number of policy gains in mu [Currently 3]
        G_Labels = ['Tau_trigger','My','G2','G3','G4','G5'] # List of policy gain names
        colors = ['tab:blue','tab:orange','tab:green']
        # vx,vy,vz = self.grab_vel_d()

        # Vel = np.sqrt(vx**2 + vz**2)
        # phi = np.arctan2(vz,vx)


        ## CREATE SUBPLOT FOR MU 
        fig = plt.figure()
        ax = fig.add_subplot(111)
        for jj in range(num_col): # Iterate through gains and plot each
            ax.plot(k_ep_arr,mu_arr[:,jj], color=colors[jj],linestyle='--',label=G_Labels[jj])
            ax.plot(k_ep_arr,mu_arr[:,jj] + sigma_arr[:,jj], color=colors[jj])
            ax.plot(k_ep_arr,mu_arr[:,jj] - sigma_arr[:,jj], color=colors[jj])
            ax.fill_between(k_ep_arr,mu_arr[:,jj] + sigma_arr[:,jj],mu_arr[:,jj] - sigma_arr[:,jj],alpha=0.5)




        ax.set_ylabel('Policy Values')
        ax.set_xlabel('K_ep')
        ax.set_ylim(0,10) # Set lower ylim
        # ax.set_title(f'Policy Value vs Episode ($Vel_d$ = {Vel:.2f} | $\phi$ = {np.rad2deg(phi):.2f}$^{{\circ}}$)')
        ax.legend(ncol=num_col)
        ax.grid()
        fig.tight_layout()
        plt.show()

    def grab_finalPolicy(self):
        """Returns the final policy for a trial

        Data Type: Sim/Exp

        Returns:
           mu [np.array]: Final policy average
           sigma [np.array]: Final policy standard deviation

        """        
        
        ## FIND FINAL RUN DF
        K_ep = self.trial_df.iloc[-1]['K_ep']
        K_run = self.trial_df.iloc[-1]['K_run']
        run_df,IC_df,_,_ = self.select_run(K_ep,K_run)

        ## SELECT MU & SIGMA
        mu = IC_df.iloc[0]['mu']
        mu = np.fromstring(mu[1:-1], dtype=float, sep=' ')  # Convert str to np array
        

        sigma = IC_df.iloc[0]['sigma']
        sigma = np.fromstring(sigma[1:-1], dtype=float, sep=' ')
        

        return mu,sigma

    def grab_RLParams(self):
        """Returns initial RL Parameters

        Data Type: Sim/Exp

        Returns:
            mu [float]: Mean value
            sigma [float]: Standard deviation
        """        
        
        # CREATE INITIAL PARAMETER DATAFRAME
        param_df = self.trial_df.iloc[:][['alpha_mu','alpha_sig','mu','sigma']].dropna() 

        mu = param_df.iloc[0]['mu']
        mu = np.fromstring(mu[1:-1], dtype=float, sep=' ')  

        sigma = param_df.iloc[0]['sigma']
        sigma = np.fromstring(sigma[1:-1], dtype=float, sep=' ')  


        return mu,sigma
   


    ## STATE FUNCTIONS 
    def grab_stateData(self,K_ep,K_run,stateName: list):
        """Returns np.array of specified state

        Data Type: Sim/Exp

        Args:
            K_ep (int): Episode number
            K_run (int): Run number
            stateName (str): State name

        Returns:
            state [np.array]: Returned state values
        """        
        run_df,_,_,_ = self.select_run(K_ep,K_run)

        ## GRAB STATE DATA AND CONVERT TO NUMPY ARRAY
        state = run_df.iloc[:][stateName]
        state = state.to_numpy(dtype=np.float64)

        return state

    def plot_state(self,K_ep,K_run,stateName: list): 
        """Plot state data from run vs time

        Data Type: Sim/Exp

        Args:
            K_ep (int): Ep. Num
            K_run (int): Run. Num
            stateName list(str): State name from .csv
        """        

        ## PLOT STATE/TIME DATA
        fig = plt.figure()
        ax = fig.add_subplot(111)

        t = self.grab_stateData(K_ep,K_run,'t')
        t_norm = t - np.min(t) # Normalize time array

        ## GRAB STATE/TRIGGER DATA
        for state in stateName:

            color = next(ax._get_lines.prop_cycler)['color']

            ## PLOT STATE DATA
            state_vals = self.grab_stateData(K_ep,K_run,state)
            ax.plot(t_norm,state_vals,label=f"{state}",zorder=1,color=color)



            # ## MARK STATE DATA AT TRIGGER TIME
            # t_Rot,t_Rot_norm = self.grab_Rot_time(K_ep,K_run)
            # state_Rot = self.grab_Rot_state(K_ep,K_run,state)
            # ax.scatter(t_Rot_norm,state_Rot,label=f"{state}-Rot",zorder=2,
            #             marker='x',
            #             color=color,
            #             s=100)

            # # ## MARK STATE DATA AT IMPACT TIME
            # t_impact,t_impact_norm = self.grab_impact_time(K_ep,K_run)
            # state_Rot = self.grab_impact_state(K_ep,K_run,state)
            # ax.scatter(t_impact_norm,state_Rot,label=f"{state}-Impact",zorder=2,
            #             marker='s',
            #             color=color,
            #             s=50)

            ## MARK STATE DATA AT TRAJ START
            # if self.dataType == 'EXP':
            #     t_traj,t_traj_norm = self.grab_traj_start(K_ep,K_run)
            #     state_traj = self.grab_traj_state(K_ep,K_run,state)
            #     ax.scatter(t_traj_norm,state_traj,label=f"{state}-Traj",zorder=2,
            #                 marker='o',
            #                 color=color,
            #                 s=25)


        ax.set_xlabel("time [s]")
        ax.legend()
        ax.grid()

        plt.show()
  
    def grab_time(self,K_ep,K_run):
        run_df,_,_,_ = self.select_run(K_ep,K_run)

        ## GRAB STATE DATA AND CONVERT TO NUMPY ARRAY
        state = run_df.iloc[:]['t']
        t = state.to_numpy(dtype=np.float64)

        t_normalized = t - np.min(t)

        return t,t_normalized

    

    ## DESIRED IC FUNCTIONS
    def grab_vel_IC(self):
    
        """Return IC velocities

        Returns:
            vx_IC (float): Desired Rot x-velocity
            vy_IC (float): Desired Rot y-velocity
            vz_IC (float): Desired Rot z-velocity
        """        
        vel_df = self.trial_df[['mu','vx','vy','vz']].dropna()
        vx_IC,vy_IC,vz_IC = vel_df.iloc[0][['vx','vy','vz']]
        
        return vx_IC,vy_IC,vz_IC

    def grab_vel_IC_2D_angle(self):
        """Return IC velocities

        Returns:
            Vel_IC (float): Desired flight velocity
            phi_IC (float): Desired flight angle
        """        
        ## SELECT X,Y,Z LAUNCH VELOCITIES
        Vel_IC,phi_IC = self.trial_df.iloc[-3][['vx','vy']].astype('float').tolist()
        
        return Vel_IC,phi_IC
        

    ## TRIGGER FUNCTIONS
    def grab_Rot_time(self,K_ep,K_run):
        """Returns time of Rot

        Data Type: Sim/Exp

        Args:
            K_ep (int): Episode number
            K_run (int): Run number

        Returns:
            float: t_Rot,t_Rot_normalized
        """        

        run_df,_,Rot_df,_ = self.select_run(K_ep,K_run)
        if Rot_df.iloc[0]['Trg_flag'] == True: # If Impact Detected
            t_Rot = float(Rot_df.iloc[0]['t'])
            t_Rot_norm = t_Rot - float(run_df.iloc[0]['t']) # Normalize Rot time
        else:
            t_Rot = np.nan
            t_Rot_norm = np.nan

        return t_Rot,t_Rot_norm


    def grab_Rot_state(self,K_ep,K_run,stateName: list):
        """Returns desired state at time of Rot

        Data Type: Sim/Exp

        Args:
            K_ep (int): Episode number
            K_run (int): Run number
            stateName (str): State name

        Returns:
            float: state_Rot
        """        

        _,_,Rot_df,_ = self.select_run(K_ep,K_run)
        if Rot_df.iloc[0]['Trg_flag'] == True: # If Impact Detected

            state_Rot = float(Rot_df.iloc[0][stateName])
        else:
            state_Rot = np.nan



        return state_Rot
        

    ## IMPACT FUNCTIONS
    def grab_impact_time(self,K_ep,K_run):
        """Return time at impact

        Args:
            K_ep (int): Episode number
            K_run (int): Run number
            accel_threshold (float, optional): Threshold at which to declare impact. Defaults to -4.0.

        Returns:
            t_impact [float]: Time at impact
            t_impact_norm [float]: Normalized impact time 
        """        



        run_df,_,_,impact_df = self.select_run(K_ep,K_run)
        if impact_df.iloc[0]['Impact_flag'] == True: # If Impact Detected
            t_impact = float(impact_df.iloc[0]['t'])
            t_impact_norm = t_impact - float(run_df.iloc[0]['t']) # Normalize Rot time
        else:
            t_impact = np.nan
            t_impact_norm = np.nan

        return t_impact,t_impact_norm

    def grab_impact_state(self,K_ep,K_run,stateName):
        """Returns desired state at time of impact

        Data Type: Sim/Exp

        Args:
            K_ep (int): Episode number
            K_run (int): Run number
            stateName (str): State Name

        Returns:
            state_impact [float]: State at impact
        """        



        _,_,_,impact_df = self.select_run(K_ep,K_run)
        if impact_df.iloc[0]['Impact_flag'] == True: # If Impact Detected

            state_impact = float(impact_df.iloc[0][stateName])
        else:
            state_impact = np.nan


        return state_impact


    def trigger2impact(self,K_ep,K_run):
        t_Rot,_ = self.grab_Rot_time(K_ep,K_run)
        t_impact,_ = self.grab_impact_time(K_ep,K_run)

        t_delta = t_impact-t_Rot
        return t_delta


    def landing_conditions(self,K_ep,K_run):
        """Returns landing data for number of leg contacts, the impact leg, contact list, and if body impacted

        Args:
            K_ep (int): Episode number
            K_run (int): Run number

        Returns:
            leg_contacts (int): Number legs that joined to the ceiling
            contact_list (list): List of the order legs impacted the ceiling
            body_impact (bool): True if body impacted the ceiling
        """        
        _,_,_,impact_df = self.select_run(K_ep,K_run)
        body_impact = impact_df.iloc[0]['Trg_flag']
        leg_contacts = int(impact_df.iloc[0]['F_thrust'])
        contact_list = impact_df.iloc[0][['Theta_x','Theta_x_est','Theta_y','Theta_y_est']].to_numpy(dtype=np.int8)

        

        return leg_contacts,contact_list,body_impact

    def landing_rate(self,N:int=3,reward_cutoff:float=0.01):
        """Returns the landing rate for the final [N] episodes

        Args:
            N (int, optional): Final [N] episodes to analyze. Defaults to 3.
            reward_cutoff (float, optional): Reward values below this cutoff come from sim errors and not performance. Defaults to 3.00.

        Returns:
            landing_rate_4_leg (float): Four leg landing success percentage
            landing_rate_2_leg (float): Two leg landing success percentage
            contact_rate (float): Combined landing success percentage for either two leg or four leg landing
            contact_list (list,int): List of each conctact case for each run recorded
        """        

        ## CREATE ARRAY OF ALL EP/RUN COMBINATIONS FROM LAST 3 ROLLOUTS
        # Use reward to extract only the valid attempts and not simulation mishaps
        ep_df = self.trial_df.iloc[:][['K_ep','K_run','mu','Trg_flag']].dropna().drop(columns='mu')
        ep_df = ep_df.astype('float').query(f'Trg_flag >= {reward_cutoff}') 
        ep_arr = ep_df.iloc[-self.n_rollouts*N:].to_numpy() # Grab episode/run listing from past N rollouts

        ## ITERATE THROUGH ALL RUNS AND RECORD VALID LANDINGS
        four_leg_landing = 0
        two_leg_landing = 0
        one_leg_landing = 0
        failed_landing = 0
        contact_list = []
        
        for K_ep,K_run in ep_arr[:,:2]:
            
            ## RECORD LANDING CONDITIONS
            leg_contacts,_,body_impact = self.landing_conditions(K_ep, K_run)

            if leg_contacts >= 3 and not body_impact: 
                four_leg_landing += 1

            elif leg_contacts == 2 and not body_impact:
                two_leg_landing += 1

            contact_list.append(leg_contacts)

        ## CALC LANDING PERCENTAGE
        landing_rate_4leg = np.round(four_leg_landing/(N*self.n_rollouts),2)
        landing_rate_2leg = np.round(two_leg_landing/(N*self.n_rollouts),2)
        contact_rate = np.round((four_leg_landing + two_leg_landing)/(N*self.n_rollouts),2)
        

        return landing_rate_4leg,landing_rate_2leg,contact_rate,np.array(contact_list)

    def grab_trial_data(self,func,*args,N:int=3,reward_cutoff:float=0.05,landing_cutoff:int=3,**kwargs):

        ## CREATE ARRAY OF ALL EP/RUN COMBINATIONS FROM LAST 3 ROLLOUTS
        # Use reward to extract only the valid attempts and not simulation mishaps
        ep_df = self.trial_df.iloc[:][['K_ep','K_run','mu','Trg_flag']].dropna().drop(columns='mu')
        ep_df = ep_df.astype('float').query(f'Trg_flag >= {reward_cutoff}') 
        ep_arr = ep_df.iloc[-self.n_rollouts*N:].to_numpy() # Grab episode/run listing from past N rollouts

        ## ITERATE THROUGH ALL RUNS AND FINDING IMPACT ANGLE 
        var_list = []
        for K_ep,K_run in ep_arr[:,:2]:

            leg_contacts,_,body_contact = self.landing_conditions(K_ep, K_run)
            if leg_contacts >= landing_cutoff and body_contact == False: # IGNORE FAILED LANDINGS
                var_list.append(func(K_ep,K_run,*args,**kwargs))

        ## RETURN MEAN AND STD OF STATE
        trial_mean = np.nanmean(var_list)
        trial_std = np.nanstd(var_list)
        trial_arr = var_list

        return np.round(trial_mean,3),trial_std,trial_arr
    
            

if __name__ == "__main__":

    dataPath = f"/home/bhabas/catkin_ws/src/sar_simulation/crazyflie_logging/local_logs/"

    fileName = "Control_Playground--trial_24--NL.csv"
    trial = Data_Parser(dataPath,fileName)

    trial.plot_policy_convg()
