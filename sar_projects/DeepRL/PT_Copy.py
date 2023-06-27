## STANDARD IMPORTS
import os
from datetime import datetime,timedelta
import numpy as np
import pandas as pd
import torch as th
import yaml
import pandas as pd
import csv
import time 
import rospy
import rospkg

## PLOTTING IMPORTS
import matplotlib.pyplot as plt
import matplotlib as mpl
import plotly.graph_objects as go
from scipy.interpolate import griddata


## SB3 Imports
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import *
from stable_baselines3.common import utils

## DEFINE BASE PATH
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_env'))

## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%m_%d-%H:%M")

class RewardCallback(BaseCallback):
    def __init__(self, check_freq: int, save_freq: int, model_dir: str, verbose=1):
        """ Callback which monitors training progress and saves the model every [save_freq] timesteps
        and checks for highest mean reward every [check_freq] episodes.

        Args:
            check_freq (int): Number of episodes to average rewards over
            save_freq (int): Number of timesteps to save model
            model_dir (str): Model/replay_buffer save directory
            verbose (int, optional): Verbose. Defaults to 1.
        """        
        super(RewardCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.save_freq = save_freq
        self.model_dir = model_dir

        self.reward_avg = [0]
        self.reward_std = [0]

        self.episode_rewards = []
        self.highest_reward = -np.inf

    def _on_step(self) -> bool:

        ## SAVE MODEL EVERY N TIMESTEPS
        if self.num_timesteps % self.save_freq == 0 and self.num_timesteps >= 500:

            ## SAVE NEWEST MODEL AND REPLAY BUFFER
            model = os.path.join(self.model_dir, f"{self.num_timesteps}_step_model")
            replay_buff = os.path.join(self.model_dir, f"{self.num_timesteps}_step_replay_buff")

            self.model.save(model)
            self.model.save_replay_buffer(replay_buff)

        ## CALC MEAN/STD OF REWARDS EVERY N EPISODES
        if self.locals["dones"].item():
            
            episode_reward = self.locals["rewards"]
            self.episode_rewards.append(episode_reward)

            if len(self.episode_rewards) >= self.check_freq:

                mean_reward = np.mean(self.episode_rewards)
                std_reward = np.std(self.episode_rewards)

                self.reward_avg.append(mean_reward)
                self.reward_std.append(std_reward)

                ## SAVE MODEL IF HIGHEST MEAN REWARD ACHIEVED
                if mean_reward > self.highest_reward and self.num_timesteps >= 20e3:

                    ## SAVE HIGHEST REWARD
                    self.highest_reward = mean_reward
                    print(f"New high score ({self.highest_reward:.3f}), model saved.")

                    ## SAVE NEWEST MODEL AND REPLAY BUFFER
                    newest_model = os.path.join(self.model_dir, f"{self.num_timesteps}_step_model")
                    newest_replay_buff = os.path.join(self.model_dir, f"{self.num_timesteps}_step_replay_buff")

                    self.model.save(newest_model)
                    self.model.save_replay_buffer(newest_replay_buff)

                ## RESET EPISODIC REWARD LIST
                self.episode_rewards = []

            ## TB LOGGING VALUES
            self.logger.record('custom/K_ep',self.training_env.envs[0].env.K_ep)
            self.logger.record('custom/Reward',episode_reward.item())
            self.logger.record('custom/Reward_avg',self.reward_avg[-1])


class Policy_Trainer_DeepRL():
    def __init__(self,env,log_dir,log_name):
        self.env = env
        self.log_dir = log_dir

        ## LOADED MODEL
        if log_name[-2] == "_": 
            self.log_name = log_name[:-2]
            latest_run_id = utils.get_latest_run_id(log_dir,log_name)
            self.TB_log_path = os.path.join(log_dir,f"{self.log_name}_{latest_run_id}")
            self.model_dir = os.path.join(self.TB_log_path,"models")

        ## NEW MODEL
        else: 
            self.log_name = log_name
            self.TB_log_path = os.path.join(log_dir,f"{self.log_name}_0")
            self.model_dir = os.path.join(self.TB_log_path,"models")

        ## GENERATE LOG/MODEL DIRECTORY
        if not os.path.exists(self.TB_log_path):
            os.makedirs(self.TB_log_path,exist_ok=True)

        if not os.path.exists(self.model_dir):
            os.makedirs(self.model_dir,exist_ok=True)

    def create_model(self,gamma=0.999,learning_rate=0.002,net_arch=[8,8]):
        """Create Soft Actor-Critic agent used in training

        Args:
            gamma (float, optional): Discount factor. Defaults to 0.999.
            learning_rate (float, optional): Learning Rate. Defaults to 0.002.
            net_arch (list, optional): Network layer sizes and architechure. Defaults to [12,12].
        """        

        self.model = SAC(
            "MlpPolicy",
            env=self.env,
            gamma=gamma,
            learning_rate=learning_rate,
            policy_kwargs=dict(activation_fn=th.nn.ReLU,net_arch=dict(pi=net_arch, qf=[256,256])),
            verbose=1,
            device='cpu',
            tensorboard_log=self.log_dir
        ) 

        # self.save_config_file()

        return self.model
    
    def load_model(self,model_dir,model_name,t_step):
        """Loads current model and replay buffer from the selected time step

        Args:
            t_step (int): Timestep of the selected model
        """        

        ## MODEL PATHS
        model_path = os.path.join(model_dir,model_name,"models",f"{t_step}_step_model")
        replay_buff_path = os.path.join(model_dir,model_name,"models",f"{t_step}_step_replay_buff")

        ## LOAD MODEL AND REPLAY BUFFER
        self.model = SAC.load(
            model_path,
            env=self.env,
            device='cpu',
            tensorboard_log=self.log_dir
        )
        self.model.load_replay_buffer(replay_buff_path)

    def load_params_from_model(self,model_dir,model_name,t_step):

        ## MODEL PATHS
        model_path = os.path.join(model_dir,model_name,"models",f"{t_step}_step_model")

        ## LOAD PREV MODEL AND REPLAY BUFFER
        prev_model = SAC.load(
            model_path,
            env=None,
            device='cpu',
        )

        ## TRANSFER PARAMETERS TO CURRENT MODEL
        self.model.policy.actor = prev_model.actor
        self.model.policy.critic = prev_model.critic
   
    def train_model(self,check_freq=10,save_freq=5e3,reset_timesteps=False,total_timesteps=2e6):
        """Script to train model via DeepRL method

        Args:
            check_freq (int): Number of episodes to average rewards over. Defaults to 10.
            save_freq (int): Number of timesteps to save model. Defaults to 5e3.
            reset_timesteps (bool, optional): Reset starting timestep to zero. Defaults to True. 
                                              Set to False to resume training from previous model.
            total_timesteps (int, optional): Timesteps to cutoff model training. Defaults to 2e6.
        """        
        
        reward_callback = RewardCallback(check_freq=check_freq,save_freq=save_freq,model_dir=self.model_dir)

        self.model.learn(
            total_timesteps=int(total_timesteps),
            tb_log_name=self.log_name,
            callback=reward_callback,
            reset_num_timesteps=reset_timesteps,
        )

    def save_config_file(self):

        config_path = os.path.join(self.TB_log_path,"Config.yaml")

        data = dict(
            PLANE_SETTINGS = dict(
                Plane_Model = self.env.Plane_Model,
                Plane_Angle = self.env.Plane_Angle,
                Plane_Pos = dict(
                    X = self.env.Plane_Pos[0],
                    Y = self.env.Plane_Pos[1],
                    Z = self.env.Plane_Pos[2]
                ),
            ),

            SAR_SETTINGS = dict(
                SAR_Type = self.env.SAR_Type,
                SAR_Config = self.env.SAR_Config,
            ),

            ENV_SETTINGS = dict(
                Environment = self.env.Env_Name,
                Vel_Limts = self.env.Vel_range,
                Phi_Limits = self.env.Phi_range,
            ),

            LEARNING_MODEL = dict(
                Policy = self.model.policy.__class__.__name__,
                Observation_Layer = self.model.policy.observation_space.shape[0],
                Network_Layers = self.model.policy.net_arch,
                Action_Layer = self.model.policy.action_space.shape[0]*2,
                Action_Space_High = self.model.policy.action_space.high.tolist(),
                Action_Space_Low = self.model.policy.action_space.low.tolist(),
                Gamma = self.model.gamma,
                Learning_Rate = self.model.learning_rate,
                Activation_Function = "",
            )


        )

        with open(config_path, 'w') as outfile:
            yaml.dump(data,outfile,default_flow_style=False,sort_keys=False)

    def test_policy(self,Vel=None,Phi=None,episodes=10):
        """Test the currently loaded policy for a given set of velocity and launch angle conditions

        Args:
            Vel (float, optional): Flight velocity [m/s]. 
            Phi (float, optional): Flight angle [deg]. 
            episodes (int, optional): Number of landing tests. Defaults to 1.
        """        

        for ep in range(episodes):
            obs,_ = self.env.reset(Vel=Vel,Phi=Phi)
            done = False
            while not done:
                action,_ = self.model.predict(obs)
                obs,reward,done,_,_ = self.env.step(action)

            print(f"Episode {ep}: Reward {reward:.3f}")

    def policy_output(self,obs):
        """Returns direct output of policy network in form of action_mean and action_log_std.
        Policy gaussians are made from action_mean and action_std. 

        Args:
            obs (list): Observation vector returned by environment

        Returns:
            action_mean: Action distribution means
            action_log_std: Action distribution log_stds (exponentiate to get normal std values)
        """        

        # CAP THE STANDARD DEVIATION OF THE ACTOR
        LOG_STD_MAX = 2
        LOG_STD_MIN = -20
        actor = self.model.policy.actor
        obs = th.FloatTensor([obs])

        ## PASS OBS THROUGH NN
        latent_pi = actor.latent_pi(obs)
        mean_actions = actor.mu(latent_pi)
        log_std = actor.log_std(latent_pi)
        log_std = th.clamp(log_std, LOG_STD_MIN, LOG_STD_MAX)

        ## CONVERT LOG_STD TO STD
        action_log_std = log_std
        action_log_std = action_log_std.detach().numpy()[0]

        ## GRAB ACTION DISTRIBUTION MEAN
        action_mean = mean_actions
        action_mean = action_mean.detach().numpy()[0]

        return action_mean,action_log_std
    
    def save_NN_Params(self):
        """Save NN parameters to C header file for upload to crazyflie

        Args:
            SavePath (str): Path to save header file
        """        

        FileName = f"NN_Layers_{self.env.modelInitials}_DeepRL.h"
        f = open(os.path.join(self.TB_log_path,FileName),'a')
        f.truncate(0) ## Clears contents of file

        date_time = datetime.now().strftime('%m/%d-%H:%M')
        f.write(f"// Filename: {FileName} Time: {date_time}\n")
        f.write("static char NN_Params_DeepRL[] = {\n")
        
        NN_size = np.array([3]).reshape(-1,1)

        ## SAVE SCALER ARRAY VALUES
        np.savetxt(f,NN_size,
                    fmt='"%.0f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{NN_size.shape[0]},"\t"{NN_size.shape[1]},"',
                    footer='"*"\n')

        ## EXTEND SCALER ARRAY DIMENSIONS
        scaler_means = np.zeros(3).reshape(-1,1)
        scaler_stds = np.ones(3).reshape(-1,1)
        
        ## SAVE SCALER ARRAY VALUES
        np.savetxt(f,scaler_means,
                    fmt='"%.5f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{scaler_means.shape[0]},"\t"{scaler_means.shape[1]},"',
                    footer='"*"\n')

        np.savetxt(f,scaler_stds,
                    fmt='"%.5f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{scaler_stds.shape[0]},"\t"{scaler_stds.shape[1]},"',
                    footer='"*"\n')

        ## SAVE PARAMETERS OF LATENT_PI LAYERS
        for module in self.model.actor.latent_pi.modules():
            if isinstance(module, th.nn.modules.linear.Linear):
                W = module.weight.detach().numpy()
                np.savetxt(f,W,
                    fmt='"%.5f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{W.shape[0]},"\t"{W.shape[1]},"',
                    footer='"*"\n')


                b = module.bias.detach().numpy().reshape(-1,1)
                np.savetxt(f,b,
                    fmt='"%.5f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{b.shape[0]},"\t"{b.shape[1]},"',
                    footer='"*"\n')

        ## SAVE PARAMETERS FOR MU/LOG_STD LAYER
        for module in self.model.actor.mu.modules():
            W_mu = module.weight.detach().numpy()
            b_mu = module.bias.detach().numpy().reshape(-1,1)

        for module in self.model.actor.log_std.modules():
            W_log_std = module.weight.detach().numpy()
            b_log_std = module.bias.detach().numpy().reshape(-1,1)

        ## STACK WEIGHTS AND BIASES TO MAKE ONE COHESIVE LAYER INSTEAD OF SB3 DEFAULT SPLIT
        W = np.vstack((W_mu,W_log_std))
        b = np.vstack((b_mu,b_log_std))


        np.savetxt(f,W,
            fmt='"%.5f,"',
            delimiter='\t',
            comments='',
            header=f'"{W.shape[0]},"\t"{W.shape[1]},"',
            footer='"*"\n')

        np.savetxt(f,b,
            fmt='"%.5f,"',
            delimiter='\t',
            comments='',
            header=f'"{b.shape[0]},"\t"{b.shape[1]},"',
            footer='"*"\n')


        f.write("};")
        f.close()

if __name__ == '__main__':


    ## IMPORT ENVIRONMENTS
    from Envs.SAR_Sim_DeepRL2 import SAR_Sim_DeepRL
    from Envs.CF_Env_2D_2 import CF_Env_2D
    # from Envs.CF_Env_2D_3 import CF_Env_2D




    # # START TRAINING NEW DEEP RL MODEL 
    env = SAR_Sim_DeepRL(GZ_Timeout=False,Vel_range=[1.0,3.0],Phi_range=[0,90])
    # env = CF_Env_2D(Vel_range=[1.0,3.0],Phi_range=[0,90])
    log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs/{env.Env_Name}"
    log_name = f"Test_Log_Cur"    

    PolicyTrainer = Policy_Trainer_DeepRL(env,log_dir,log_name)
    PolicyTrainer.create_model()
    PolicyTrainer.load_params_from_model(log_dir,"Test_Log_Prev_0",t_step=69000)
    # PolicyTrainer.load_model(log_dir,"Test_Log_Prev",t_step=80917)
    # env.RENDER = True
    # PolicyTrainer.test_policy(episodes=30)
    PolicyTrainer.train_model(save_freq=5000)

