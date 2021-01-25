import numpy as np
import copy
import scipy.io
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from math import asin,pi,ceil,floor


# Parent Evolutionary Strategy Class
# pepg and cma inherit common methods 

# this needs to be reorganized much better, still lots of repetition
# maybe just use one class for each algorithm with options?
class ES:
    def __init__(self,gamma=0.8, n_rollout = 6):
        # ovveride this for necessary hyperparameters
        self.gamma, self.n_rollout = gamma, n_rollout

    
    def calcReward_effortMinimization(self,state_hist,FM_hist,h_ceiling,pad_contacts):

        ## DEFINE STATES
        t_hist = state_hist[0,:]
        wy_hist = state_hist[12,:]
        My_hist = FM_hist[2,:]
        z_hist = state_hist[3,:]


        ## INIT INTEGRATION VARIABLES
        pitch_hist = np.zeros_like(t_hist)
        pitch_sum = 0
        W_My = 0 
        prev = t_hist[0]

        ## INTEGRATE FOR W_My AND FINAL PITCH ANGLE
        # Integrate omega_y over time to get full rotation estimate
        # This accounts for multiple revolutions that euler angles can't
        for ii,wy in enumerate(wy_hist):
            pitch_sum += wy*(180/np.pi)*(t_hist[ii]-prev) # [deg]
            pitch_hist[ii] = pitch_sum # [deg]

            W_My += My_hist[ii]*wy*(t_hist[ii]-prev) # [N*mm]
            prev = t_hist[ii]



        ## r_contact Calc
        num_contacts = np.sum(pad_contacts)
        if num_contacts == 3 or num_contacts == 4:
            r_contact = 7
        elif num_contacts == 2:
            r_contact = 2
        elif num_contacts == 1:
            r_contact = 1
        else:
            r_contact = 0

       
        
        ## r_theta Calc
        if -170 < np.min(pitch_hist) <= 0:
            r_theta = 5*(-1/170*np.min(pitch_hist))      
        elif -195 <= np.min(pitch_hist) <= -170:
            r_theta = 5
        else:
            r_theta = 0
        

        ## r_W Calc
        r_W = 10*np.exp(-W_My/2.0)

        ## r_height Calc
        r_h = np.max(z_hist/h_ceiling)*10
        

        
        R = r_W*(r_contact+r_theta)*r_h + 0.001
        return R
    


    def calcReward_pureLanding(self,state_hist,h_ceiling,pad_contacts,body_contact): # state_hist is size 14 x timesteps
        ## DEFINE STATES
        t_hist = state_hist[0,:]
        wy_hist = state_hist[12,:]
        z_hist = state_hist[3,:]


        ## INIT INTEGRATION VARIABLES
        pitch_hist = np.zeros_like(t_hist)
        pitch_sum = 0
        prev = t_hist[0]

        ## INTEGRATE FOR W_My AND FINAL PITCH ANGLE
        # Integrate omega_y over time to get full rotation estimate
        # This accounts for multiple revolutions that euler angles can't
        for ii,wy in enumerate(wy_hist):
            pitch_sum += wy*(180/np.pi)*(t_hist[ii]-prev) # [deg]
            pitch_hist[ii] = pitch_sum # [deg]

            prev = t_hist[ii]



        ## r_contact Calc
        num_contacts = np.sum(pad_contacts)
        if body_contact == False:
            if num_contacts == 3 or num_contacts == 4:
                r_contact = 7
            elif num_contacts == 2:
                r_contact = 2
            elif num_contacts == 1:
                r_contact = 1
            else:
                r_contact = 0
        else:
            r_contact = 0


       
        
        ## r_theta Calc
        if -170 < np.min(pitch_hist) <= 0:
            r_theta = 5*(-1/170*np.min(pitch_hist))      
        elif -250 <= np.min(pitch_hist) <= -170:
            r_theta = 5
        else:
            r_theta = 0
        

        

        ## r_height Calc
        r_h = np.max(z_hist/h_ceiling)*10
        

        
        R = (r_contact+r_theta)*r_h + 0.001
        print(f"Reward: r_c: {r_contact:.3f} | r_theta: {r_theta:.3f} | r_h: {r_h:.3f} | pitch sum: {np.min(pitch_hist):.2f}")
        return R

    def get_baseline(self, span):
        # should be the same
        if self.reward_history.size < span:
            b = np.mean(self.reward_history)
        else:
            b = np.mean(self.reward_history[-span:])

        return b

class rlsysPEPGAgent_reactive(ES):
    def __init__(self, alpha_mu, alpha_sigma, mu,sigma,gamma=0.95, n_rollouts = 6):
        self.gamma = gamma
        self.n_rollouts = n_rollouts
        self.agent_type = 'SyS-PEPG_reactive'

        self.alpha_mu =  alpha_mu
        self.alpha_sigma = alpha_sigma
        
        self.mu = mu
        self.sigma = sigma

        self.mu_history = copy.copy(self.mu)  # Creates another array of self.mu and attaches it to self.mu_history
        self.sigma_history = copy.copy(self.sigma)
        self.reward_history = np.array([0])

    def get_theta(self):
        zeros = np.zeros_like(self.mu)
        epsilon = np.random.normal(zeros, abs(self.sigma), [zeros.size, self.n_rollouts//2]) # theta is size of  mu.size x n_runPerEp

        theta_plus = self.mu + epsilon
        theta_minus = self.mu - epsilon

        theta = np.append(theta_plus, theta_minus, axis=1)


        ## Caps values of RREV to be greater than zero and Gain to be neg so
        # the system doesnt fight itself learning which direction to rotate
        theta[theta<=0] = 0.001


        return theta, epsilon

    def train(self, theta, reward, epsilon):
        reward_plus = reward[0:self.n_rollouts//2]
        reward_minus = reward[self.n_rollouts//2:]
        epsilon = epsilon
        b = self.get_baseline(span=3)

        m_reward = 21.0 # max reward
  
        ## Decaying Learning Rate:
        #self.alpha_mu = self. * 0.9
        #self.alpha_sigma = self.alpha_sigma * 0.9

        T = epsilon
        S = (T*T - self.sigma*self.sigma)/abs(self.sigma)
        r_T = (reward_plus - reward_minus) / (2*m_reward - reward_plus - reward_minus)
        r_S = ((reward_plus + reward_minus)/2 - b) / (m_reward - b)



        self.mu = self.mu + self.alpha_mu*(np.dot(T,r_T))
        self.sigma = self.sigma + self.alpha_sigma*np.dot(S,r_S)

        self.sigma[self.sigma<=0] = 0.05 #  If sigma oversteps negative then assume convergence
        
        self.mu_history = np.append(self.mu_history, self.mu, axis=1)
        self.sigma_history = np.append(self.sigma_history, self.sigma, axis=1)
        self.reward_history = np.append(self.reward_history, np.mean(reward))





class rlsysPEPGAgent_adaptive(rlsysPEPGAgent_reactive):
    def __init__(self):
        self.agent_type = 'SyS-PEPG_adaptive'
    def train(self, theta, reward, epsilon):
        reward_avg = np.mean(reward)
        if len(self.reward_history == 1):
            self.reward_history[0] = reward_avg
        else:
            self.reward_history = np.append(self.reward_history, reward_avg) 

        reward_plus = reward[0:self.n_rollouts//2]
        reward_minus = reward[self.n_rollouts//2:]
        epsilon = epsilon
        b = self.get_baseline(span=3)

        m_reward = 21.0 # max reward
        
       
        ## Decaying Learning Rate:
        #self.alpha_mu = self. * 0.9
        #self.alpha_sigma = self.alpha_sigma * 0.9

        T = epsilon
        S = (T*T - self.sigma*self.sigma)/abs(self.sigma)
        r_T = (reward_plus - reward_minus) / (2*m_reward - reward_plus - reward_minus)
        r_S = ((reward_plus + reward_minus)/2 - b) / (m_reward - b)

         # Defeine learning rate scale depending on reward recieved
        #lr_scale = 1.0 - reward_avg/m_reward # try squaring?
        lr_scale = 1.0 - b/m_reward
        explore_factor = 1.0 # determines how much faster sigma alpha decreases than mu alpha
        b2 = self.get_baseline(5)

        print(len(self.reward_history))
        print(self.reward_history.size)

        # burst exploration if algorithim gets stuck at local optimum
        # CAUSES OVERSHOOTING
        if len(self.reward_history) > 20:
            if b2 < m_reward*0.7: # increase sigma alot of reward low
                lr_scale = 5.0*(1.0 - b2/m_reward)
            elif b2 < m_reward*(0.85): # increase sigma slightly of close to max
                lr_scale = 2.0*(1.0 - b2/m_reward)

        lr_sigma = (lr_scale**explore_factor)/explore_factor # adjust sigma alpha
        # update new learning rates
        # These only depend on the current episode
        self.alpha_mu = np.array([[lr_scale],[lr_scale] ,[lr_scale]])#0.95
        self.alpha_sigma = np.array([[lr_sigma],[lr_sigma] ,[lr_sigma]])  #0.95
        print(self.alpha_mu,self.alpha_sigma)
        print(np.dot(T,r_T),np.dot(S,r_S))
        self.mu = self.mu + self.alpha_mu*(np.dot(T,r_T))
        self.sigma = self.sigma + self.alpha_sigma*np.dot(S,r_S)

        self.sigma[self.sigma<=0] = 0.05 #  If sigma oversteps negative then assume convergence
        
        self.mu_history = np.append(self.mu_history, self.mu, axis=1)
        self.sigma_history = np.append(self.sigma_history, self.sigma, axis=1)
        self.reward_history = np.append(self.reward_history, np.mean(reward))


