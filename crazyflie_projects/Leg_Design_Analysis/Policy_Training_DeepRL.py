## STANDARD IMPORTS
import os
from datetime import datetime
import numpy as np
import pandas as pd
import torch as th

## SB3 Imports
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import *

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg,os
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(1,'/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_env')
sys.path.insert(1,BASE_PATH)
from crazyflie_projects.DeepRL.Policy_Training_DeepRL import Policy_Trainer_DeepRL


## IMPORT ENVIRONMENTS
from Envs.CrazyflieEnv_DeepRL import CrazyflieEnv_DeepRL


## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%m_%d-%H:%M")



if __name__ == '__main__':

    ## INITIATE ENVIRONMENT
    env = CrazyflieEnv_DeepRL(GZ_Timeout=True)
    # env = None


    # # LOAD DEEP RL MODEL
    # load_model_name = f"SAC--10_12-11:40--NL_0"

    # log_dir = f"{BASE_PATH}/crazyflie_projects/DeepRL/TB_Logs/CF_Gazebo"
    # leg_config = "NL"
    # log_name = f"SAC--{current_time}--{leg_config}"

    # policy_path = os.path.join(log_dir,load_model_name)
    # model_path = os.path.join(log_dir,load_model_name,f"models/{2}000_steps.zip")
    # model = SAC.load(model_path,env=env,device='cpu')
    # model.load_replay_buffer(f"{log_dir}/{load_model_name}/models/replay_buff.pkl")


    ## CREATE NEW DEEP RL MODEL 
    log_dir = f"{BASE_PATH}/crazyflie_projects/DeepRL/TB_Logs/CF_Gazebo"
    leg_config = "NL"
    log_name = f"SAC--{current_time}--{leg_config}"
    model = SAC(
        "MlpPolicy",
        env=env,
        gamma=0.999,
        learning_rate=0.002,
        policy_kwargs=dict(activation_fn=th.nn.ReLU,net_arch=[12,12]),
        verbose=1,
        device='cpu',
        tensorboard_log=log_dir
    ) 

    
    Policy = Policy_Trainer_DeepRL(env,model,leg_config)
    Policy.train_model(log_name,log_dir,reset_timesteps=True)

    # Policy.save_NN_Params(policy_path)
    # Policy.plotPolicyRegion(iso_level=1.5)
