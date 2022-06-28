#!/usr/bin/env python3
import numpy as np
import os
import rospy
import time


from Crazyflie_env2 import CrazyflieEnv
from RL_agents.rl_EM import EPHE_Agent
from ExecuteFlight import executeFlight


os.system("clear")
np.set_printoptions(precision=2, suppress=True)


if __name__ == '__main__':
    ## INIT GAZEBO ENVIRONMENT
    env = CrazyflieEnv(gazeboTimeout=False)

    ## INIT LEARNING AGENT
    # Mu_Tau value is multiplied by 10 so complete policy is more normalized
    mu = np.array([[2.5], [5]])       # Initial mu starting point
    sigma = np.array([[0.5],[1.5]])   # Initial sigma starting point

    agent = EPHE_Agent(mu,sigma,n_rollouts=8)


    # ============================
    ##     FLIGHT CONDITIONS  
    # ============================

    ## CONSTANT VELOCITY LAUNCH CONDITIONS
    V_d = 2.5 # [m/s]
    phi = 90   # [deg]
    phi_rad = np.radians(phi)

    ## INITIALIALIZE LOGGING DATA
    trial_num = 24
    trial_name = f"{agent.agent_type}--Vd_{V_d:.2f}--phi_{phi:.2f}--trial_{int(trial_num):02d}--{env.modelInitials()}"
    env.filepath = f"{env.loggingPath}/{trial_name}.csv"
    env.createCSV(env.filepath)


    # ============================
    ##          Episode         
    # ============================

    for k_ep in range(0,5):

        
        # ## CONVERT AGENT ARRAYS TO LISTS FOR PUBLISHING
        # env.mu = agent.mu.flatten().tolist()                # Mean for Gaussian distribution
        # env.sigma = agent.sigma.flatten().tolist()          # Standard Deviation for Gaussian distribution

        # env.mu_1_list.append(env.mu[0])
        # env.mu_2_list.append(env.mu[1])

        # env.sigma_1_list.append(env.sigma[0])
        # env.sigma_2_list.append(env.sigma[1])

        
        ## PRE-ALLOCATE REWARD VEC AND OBTAIN THETA VALS
        training_arr = np.zeros(shape=(agent.n_rollouts,1)) # Array of reward values for training
        theta_i,epsilon_i = agent.get_theta()             # Generate sample policies from distribution

        ## PRINT EPISODE DATA
        print("=============================================")
        print("STARTING Episode # %d" %k_ep)
        print("=============================================")

        print( time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) )
        print(f"mu_0 = {agent.mu[0,0]:.3f}, \t sig_0 = {agent.sigma[0,0]:.3f}")
        print(f"mu_1 = {agent.mu[1,0]:.3f}, \t sig_1 = {agent.sigma[1,0]:.3f}")
        print('\n')
        

        print("theta_i = ")
        print(theta_i[0,:], "--> Tau")
        print(theta_i[1,:], "--> My")

        # ============================
        ##          Run 
        # ============================
        for k_run in range(0,agent.n_rollouts):


            ## INITIALIZE POLICY PARAMETERS: 
            Tau_thr = theta_i[0, k_run]    # Tau threshold 10*[s]
            My = theta_i[1, k_run]         # Policy Moment Action [N*mm]
            G2 = 0.0                        # Deprecated policy term

            env.reset()
            obs,reward,done,info = env.ParamOptim_Flight(Tau_thr/10,My,V_d,phi)

            ## ADD VALID REWARD TO TRAINING ARRAY
            training_arr[k_run] = reward
            
     

        ## =======  EPISODE COMPLETED  ======= ##
        # print(f"Episode # {k_ep:d} training, average reward {reward_avg:.3f}")
        agent.train(theta_i,training_arr,epsilon_i)


