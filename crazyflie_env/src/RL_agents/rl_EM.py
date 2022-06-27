import numpy as np
import copy
import scipy.stats 
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from math import asin,pi,ceil,floor
# from RL_agents.rl_syspepg import ES

class rlEM_PEPGAgent():
    ## CITED HERE: DOI 10.1007/s10015-015-0260-7
    def __init__(self,mu=[0,0,0],sigma=[0,0,0], n_rollouts = 6):
        
        self.agent_type = 'EM_PEPG'
        self.n_rollouts = n_rollouts
        self.mu = np.array(mu).reshape(-1,1)
        self.sigma = np.array(sigma).reshape(-1,1)

        self.n_rollouts = n_rollouts
        self.d = len(self.mu)
        self.alpha_mu, self.alpha_sigma  = np.array([[0],[0]]),np.array([[0],[0]])


        self.mu_history = copy.copy(self.mu)  # Creates another array of self.mu and attaches it to self.mu_history
        self.sigma_history = copy.copy(self.sigma)
        self.reward_history = np.array([0])

    def get_theta(self):
        theta = np.zeros((len(self.mu),self.n_rollouts))
        
        lower_limit = [0.0,0.0] # Lower and Upper limits for truncated normal distribution 
        upper_limit = [6.0,10.0]
                                # 9.10 N*mm is the upper limit for My_d
        eps = 1e-12

        for ii,mu_ii in enumerate(self.mu):
            theta[ii,:] = scipy.stats.truncnorm.rvs((lower_limit[ii]-mu_ii)/(self.sigma[ii]+eps),
                (upper_limit[ii]-mu_ii)/(self.sigma[ii]+eps),loc=mu_ii,scale=self.sigma[ii],size=self.n_rollouts)      


        return theta,0

    def train(self,theta,reward,epsilon):

        summary = np.concatenate((np.transpose(theta),reward),axis=1)
        summary = np.transpose(summary[summary[:,self.d].argsort()[::-1]])
        print(summary)

        k = ceil(self.n_rollouts/3)
        k = 3 # Select the top 3 rollouts for training

        S_theta = (summary[0:self.d,0:k].dot(summary[self.d,0:k].reshape(k,1)))
        S_reward = np.sum(summary[self.d,0:k])

        S_diff = np.square(summary[0:self.d,0:k] - self.mu).dot(summary[self.d,0:k].reshape(k,1))

        
        self.mu = S_theta/(S_reward + 0.001)
        self.sigma = np.sqrt(S_diff/(S_reward + 0.001))

        '''S_theta = np.dot(theta,reward)
        S_reward = np.sum(reward)
        S_diff = np.square(theta - self.mu).dot(reward)'''

    
    def calcReward_Impact(self,env):

        ## CALC R_1 FROM MAXIMUM HEIGHT ACHIEVED
        R_1 = 1/env.d_ceil_max

        ## DISTANCE REWARD (Gaussian Function)
        A = 0.1
        mu = 0
        sig = 1
        R_1 = A*np.exp(-1/2*np.power((env.d_ceil_max-mu)/sig,2))

        ## CALC R2 FROM THE IMPACT ANGLE
        eul_y_impact = env.eulCF_impact[1]
        # Center axis [theta_2] is limited to +- 90deg while [theta_1 & theta_3] can rotate to 180deg 
        # https://graphics.fandom.com/wiki/Conversion_between_quaternions_and_Euler_angles
        # https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/Quaternions.pdf
        

        if -180 <= eul_y_impact <= -90:
            R_2 = 1.0
        elif -90 < eul_y_impact <= 0:
            R_2 = -1/90*eul_y_impact
        elif 0 < eul_y_impact <= 90:
            R_2 = 1/90*eul_y_impact
        elif 90 < eul_y_impact <= 180:
            R_2 = 1.0
        else:
            R_2 = 0

        R_2 *= 0.2


        ## CALC R_4 FROM NUMBER OF LEGS CONNECT

        if env.pad_connections >= 3: 
            if env.BodyContact_flag == False:
                R_4 = 0.7
            else:
                R_4 = 0.4
            
        elif env.pad_connections == 2: 
            if env.BodyContact_flag == False:
                R_4 = 0.3
            else:
                R_4 = 0.2
                
        elif env.pad_connections == 1:
            R_4 = 0.1
        
        else:
            R_4 = 0.0
                


        R_total = R_1 + R_2 + R_4 + 0.001
        return R_total

class rlEM_AdaptiveAgent(rlEM_PEPGAgent):

    def train(self,theta,reward,epsilon):
        reward_mean = np.mean(reward) # try baseline
        print(len(self.reward_history))
        if len(self.reward_history == 1):
            self.reward_history[0] = reward_mean
        else:
            self.reward_history = np.append(self.reward_history, reward_mean)

        print(reward_mean)
        reward_b = np.zeros_like(reward)
        b = self.get_baseline(span=3)
        print(b)
        for count,r in enumerate(reward):
            reward_b[count] = max(0,r - b)

        summary = np.concatenate((theta.T,reward,reward_b),axis=1)
        summary = (summary[summary[:,self.d].argsort()[::-1]]).T
        print(summary)


        S_theta = summary[0:self.d,:]@summary[self.d+1,:].reshape(self.n_rollouts,1)
        print("S theta = ", S_theta)
        S_reward = np.sum(summary[self.d+1,:])
        S_diff = np.square(summary[0:self.d,:] - self.mu)@(summary[self.d+1,:].reshape(self.n_rollouts,1))
        print("S_sigma = ",S_diff)

        sii = np.sqrt(S_diff/(S_reward + 0.001))
        #sij = np.sign(S_ij)*np.sqrt(abs(S_ij)/(S_reward + 0.001)) # no need for sqrt
        #sij = S_ij/(S_reward + 0.001) # this should be correct update term from derivation

        self.mu =  S_theta/(S_reward +0.001)
        self.sigma = np.array([[sii[0,0]]])

        self.mu_history = np.append(self.mu_history, self.mu, axis=1)
        self.sigma_history = np.append(self.sigma_history, self.sigma, axis=1)




class EPHE_Agent():  # EM Policy Hyper Paramter Exploration
     def __init__(self, mu, sigma, n_rollouts = 8, adaptive = True, lamda = 1):
         self.n_rollouts = n_rollouts
         self.agent_type = 'EPHE'

         self.mu = mu
         self.sigma = sigma

         self.adaptive = adaptive
         self.n_rollouts = n_rollouts  # total runs per episode
         self.d = len(self.mu) # problem dimension
         self.alpha_mu, self.alpha_sigma  = np.array([[0],[0]]),np.array([[0],[0]])  # placeholder for compatibility with pepg agent


         if lamda < 1 or lamda > self.n_rollouts:
             raise ValueError("lamda must be between 1 and 2*n_rollout")
         else:
             self.lamda = lamda

         self.mu_history = copy.copy(self.mu)  # Creates another array of self.mu and attaches it to self.mu_history
         self.sigma_history = copy.copy(self.sigma)
         self.reward_history = None

     def get_theta(self):
         theta = np.zeros((self.d,self.n_rollouts))

         for dim in range(0,self.d):
             theta[dim,:] = np.random.normal(self.mu[dim,0],self.sigma[dim,0],[1,self.n_rollouts])

         for k_n in range(self.n_rollouts):
             for dim in range(self.d):
                 if theta[dim,k_n] < 0: 
                     theta[dim,k_n] = 0.001 

         return theta, 0


     def train(self,theta,reward,epsilon):
         reward_mean = np.mean(reward)

         if self.reward_history is None:  # initizlize np array for first rollout, o.w. append 
             self.reward_history = np.array(reward_mean)
         else:
             self.reward_history = np.append(self.reward_history, reward_mean)

         reward_b = np.zeros_like(reward)
         b = self.get_baseline(span=3)

         for count,r in enumerate(reward):  # reward used for adaptive case
             reward_b[count] = max(0,r - b)


         summary = np.concatenate((theta.T,reward,reward_b),axis=1)   # sort rollouts from highest to lowest reward
         summary = (summary[summary[:,self.d].argsort()[::-1]]).T

         if not self.adaptive:   # consider base reward for top lamda members if not adaptive
             summary[self.d+1,:self.lamda] = summary[self.d,:self.lamda]
             summary[self.d+1,self.lamda:] = np.zeros((1,self.n_rollouts-self.lamda))

         print(summary)


         S_theta = summary[0:self.d,:]@summary[self.d+1,:].reshape(self.n_rollouts,1)   # sigma and mu update terms
         S_reward = np.sum(summary[self.d+1,:])
         S_diff = np.square(summary[0:self.d,:] - self.mu)@(summary[self.d+1,:].reshape(self.n_rollouts,1))


         self.mu =  S_theta/(S_reward +0.001)
         self.sigma = np.sqrt(S_diff/(S_reward + 0.001))

         self.mu_history = np.append(self.mu_history, self.mu, axis=1)
         self.sigma_history = np.append(self.sigma_history, self.sigma, axis=1)





#  if __name__ == "__main__":
#      np.set_printoptions(precision=2, suppress=True)
#      mu = np.array([[1.0],[-4.0]])
#      sigma = np.array([[0.1],[1.5]])

#      agent = EPHE_Agent(mu,sigma)

#      theta,epsilon = agent.get_theta()
#      reward = np.array([[1.0],[2.0],[0.3],[4.0],[2.5],[1.2] ])

#      reward = np.array([[1.0],[2.0],[0.3],[4.0],[2.5],[1.2],[0.3],[4.0],[2.5],[1.2]  ])
#      agent.train(theta,reward,0)
