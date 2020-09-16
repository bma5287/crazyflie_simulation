#!/usr/bin/env python3

import numpy as np
import time,copy
from crazyflie_env import CrazyflieEnv
from rl_syspepg import rlsysPEPGAgent_reactive
import matplotlib.pyplot as plt

import os

'''
mu = [5.267,-10.228,-4.713]
z -> 2.5 3.5 sseems to fail at lower vz
x -> -0.5 0.5

'''
## Initialize the environment
env = CrazyflieEnv(port_self=18050, port_remote=18060)
print("Environment done")

## Learning rates and agent
alpha_mu = np.array([[0.2],[0.2],[0.2]])
alpha_sigma = np.array([[0.1],[0.1],[0.1]])
agent = rlsysPEPGAgent_reactive(_alpha_mu=alpha_mu, _alpha_sigma=alpha_sigma, _gamma=0.95, _n_rollout=8)

## Define initial parameters for gaussian function
agent.mu_ = np.array([[5.27], [-10.23],[-4.71]])   # Initial estimates of mu: size (2 x 1)
agent.sigma_ = np.array([[0.5], [0.5],[0.5]])      # Initial estimates of sigma: size (2 x 1)
agent.mu_history_ = copy.copy(agent.mu_)  # Creates another array of self.mu_ and attaches it to self.mu_history_
agent.sigma_history_ = copy.copy(agent.sigma_)


h_ceiling = 1.5 # meters


username = 'bader' # change to system user
start_time0 = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
file_name = '/home/'+username+'/catkin_ws/src/src/crazyflie_simulation/4. rl/src/log/' + start_time0 + '.xls'
#file_log, sheet = env.create_xls(start_time=start_time0, sigma=sigma, alpha=alpha, file_name=file_name)


## Initial figure setup
plt.ion()  # interactive on
fig = plt.figure()
plt.grid()
plt.xlim([-10,100])
plt.ylim([0,150])
plt.xlabel("Episode")
plt.ylabel("Reward")
plt.title("Episode: %d Run: %d" %(0,0))
plt.show() 


# ============================
##          Episode 
# ============================
for k_ep in range(1000):


    print("=============================================")
    print("STARTING Episode # %d" %k_ep)
    print("=============================================")

    print( time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())) )
    mu = agent.mu_
    sigma = agent.sigma_
    print("RREV=%.3f, \t theta2=%.3f, \t theta3=%.3f" %(mu[0], mu[1],mu[2]))
    print("sig1=%.3f, \t sig2=%.3f, \t sig3=%.3f" %(sigma[0], sigma[1],sigma[2]))
    print()

    done = False
    reward = np.zeros(shape=(2*agent.n_rollout_,1))
    reward[:] = np.nan  # initialize reward to be NaN array, size n_rollout x 1
    theta_rl, epsilon_rl = agent.get_theta()
    print( "theta_rl = ")
    np.set_printoptions(precision=3, suppress=True)
    print(theta_rl[0,:], "--> RREV")
    print(theta_rl[1,:], "--> Gain")
    print(theta_rl[2,:], "--> omega_x Gain")

    ct = env.getTime()

    # ============================
    ##          Run 
    # ============================
    k_run = 0
    while k_run < 2*agent.n_rollout_:

            
        vz_ini = 2.75 + np.random.rand()   # [2.5 , 2.5]
        vx_ini = -0.5 + np.random.rand()  # [-0.5, 0.5]
        vy_ini = 0.0
        # try adding policy parameter for roll pitch rate for vy ( roll_rate = gain3*omega_x)

        print("\n!-------------------Episode # %d run # %d-----------------!" %(k_ep,k_run))
        print("RREV: %.3f \t gain1: %.3f \t gain2: %.3f" %(theta_rl[0,k_run], theta_rl[1,k_run],theta_rl[2,k_run]))
        print("Vz_ini: %.3f \t Vx_ini: %.3f \t Vy_ini: %.3f" %(vz_ini, vx_ini, vy_ini))

        state = env.reset()

        k_step = 0
        done_rollout = False
        start_time_rollout = env.getTime()
        start_time_pitch = None
        pitch_triggered = False
        state_history = None
        
        env.logDataFlag = True

        # ============================
        ##          Rollout 
        # ============================
        action = {'type':'vel', 'x':vx_ini, 'y':vy_ini, 'z':vz_ini, 'additional':0.0}
        #action = {'type':'pos', 'x':0.0, 'y':0.0, 'z':1.0, 'additional':0.0}

        env.step(action=action)
        
        RREV_trigger = theta_rl[0, k_run]


        while True:
            time.sleep(5e-4) # Time step size
            k_step = k_step + 1 # Time step
            ## Define current state
            state = env.state_current_
            
            position = state[1:4]
            orientation_q = state[4:8]
            vel = state[8:11]
            vz, vx, vy= vel[2], vel[0], vel[1]
            omega = state[11:14]

            d = h_ceiling - position[2]
            RREV, omega_y ,omega_x = vz/d, vx/d, vy/d

            qw = orientation_q[0]
            qx = orientation_q[1]
            qy = orientation_q[2]
            qz = orientation_q[3]
            # theta = np.arcsin( -2*(qx*qz-qw*qy) ) # obtained from matlab "edit quat2eul.m"

            ## Enable sticky feet and rotation
            if (RREV > RREV_trigger) and (pitch_triggered == False):
                start_time_pitch = env.getTime()
                env.enableSticky(1)

                # add term to adjust for tilt 
                q_d = theta_rl[1,k_run] * RREV + theta_rl[2,k_run]*omega_x
                # torque on x axis to adjust for vy
                r_d = 0.0 #r_d = theta_rl[3,k_run] * omega_y
                print('----- pitch starts -----')
                print( 'vz=%.3f, vx=%.3f, vy=%.3f,RREV=%.3f,omega_y=%.3f qd=%.3f' %(vz, vx,vy, RREV, omega_y, q_d) )   
                print("Pitch Time: %.3f" %start_time_pitch)
                
                env.delay_env_time(t_start=start_time_pitch,t_delay=30) # Artificial delay to mimic communication lag [ms]
                action = {'type':'rate', 'x':r_d, 'y':q_d, 'z':0.0, 'additional':0.0}
                env.step(action) # Start rotation and mark rotation triggered
                pitch_triggered = True

            ## If time since triggered pitch exceeds [0.7s]   
            if pitch_triggered and ((env.getTime()-start_time_pitch) > 0.7):
                print("Rollout Completed: Pitch Triggered")
                print("Time: %.3f Start Time: %.3f Diff: %.3f" %(env.getTime(), start_time_pitch,(env.getTime()-start_time_pitch)))
                done_rollout = True

            ## If time since rollout start exceeds [1.5s]
            if (env.getTime() - start_time_rollout) > 1.5:
                print("Rollout Completed: Time Exceeded")
                print("Time: %.3f Start Time: %.3f Diff: %.3f" %(env.getTime(), start_time_rollout,(env.getTime()-start_time_rollout)))
                done_rollout = True
            
           ## If nan is found in state vector repeat sim run
            if any(np.isnan(state)): # gazebo sim becomes unstable, relaunch simulation
                print("NAN found in state vector")
                env.logDataFlag = False
                env.close_sim()
                env.launch_sim()
                if k_run > 0:
                    k_run = k_run - 1
                break
            
            if (np.abs(position[0]) > 1.0) or (np.abs(position[1]) > 1.0):
                print("Reset improperly!!!!!")
                # env.pause()
                env.logDataFlag = False
                break
            
            ## Keep record of state vector every 10 time steps
            ## Each iteration changes state vector from [14,] into (state2)[14,1] 
            ##      First iteration: track_state = state2 vector
            ##      Every 10 iter: append track_state columns with current state2 vector 
            state2 = state[1:,np.newaxis]
            if state_history is None:
                state_history = state2 # replace w/ state_history variable for track_state
            else:
                if k_step%10==0:
                    state_history = np.append(state_history, state2, axis=1)

            if done_rollout:
                env.logDataFlag = False
                reward[k_run] = agent.calculate_reward(_state=state_history, _h_ceiling=h_ceiling)
                print("Reward = %d" %(reward[k_run]))
                print("!------------------------End Run------------------------! \n")
                ## Episode Plotting
                plt.plot(k_ep,reward[k_run],marker = "_", color = "black", alpha = 0.5) 
                plt.title("Episode: %d Run: %d" %(k_ep, k_run+1))
                # If figure gets locked on fullscreen, press ctrl+f untill it's fixed (there's lag due to process running)
                plt.draw()
                plt.pause(0.001)
                # fig.canvas.flush_events()
                
                k_run = k_run + 1
                #print( 'x=%.3f, y=%.3f, z=%.3f' %(position[0], position[1], position[2]) )
                break

    if not any( np.isnan(reward) ):
        print("Episode # %d training, average reward %.3f" %(k_ep, np.mean(reward)))
        agent.train(_theta = theta_rl, _reward=reward, _epsilon=epsilon_rl)

        plt.plot(k_ep,np.mean(reward),'ro')
        plt.draw()
        plt.pause(0.001)

        # env.add_record_xls(file_log=file_log, sheet=sheet, file_name=file_name,
        #     k_ep=k_ep, start_time1=start_time11, start_time2=start_time12,
        #     vz_ini=vz_ini, vx_ini=vx_ini, state=state, action=action[0],
        #     reward=reward, info=info, theta=theta)

