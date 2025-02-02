## IMPORT ENVIRONMENTS
from RL_Manager import RL_Training_Manager
from Envs.SAR_Sim_DeepRL import SAR_Sim_DeepRL
from Envs.SAR_2D_DeepRL import SAR_2D_Env

## STANDARD IMPORTS
import os
from datetime import datetime
import rospkg
import argparse
import json

## DEFINE BASE PATH
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_env'))
LOG_DIR = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs" 


## ARGUMENT PARSER
parser = argparse.ArgumentParser(description='Policy Pre-Training Script')
parser.add_argument('--GroupName',      help='Log group name', default='')
parser.add_argument('--TrainConfig',    help='Path to training config file', required=True)
parser.add_argument('--S3_Upload',      help='Upload to S3', default=False, type=bool)
args = parser.parse_args()

## LOAD TRAINING MODEL CONFIGURATION
with open(args.TrainConfig, 'r') as TrainConfig_File:
    TrainConfig = json.load(TrainConfig_File)


## UPDATE SAR TYPE AND SAR CONFIG IN BASE SETTINGS FILE
Base_Settings_yaml = f"{BASE_PATH}/sar_config/Base_Settings.yaml"
with open(Base_Settings_yaml, 'r') as file:
    lines = file.readlines()

for i, line in enumerate(lines):
    if line.strip().startswith('SAR_Type:'):
        lines[i] = f"  SAR_Type: '{TrainConfig['SAR_SETTINGS']['SAR_Type']}'\n"
    elif line.strip().startswith('SAR_Config:'):
        lines[i] = f"  SAR_Config: '{TrainConfig['SAR_SETTINGS']['SAR_Config']}'\n"

with open(Base_Settings_yaml, 'w') as file:
    file.writelines(lines)

if __name__ == '__main__':

    ## LOGGING CONFIGURATION
    LogName = TrainConfig['LogName']
    PlaneAngle = TrainConfig['ENV_KWARGS']['Plane_Angle_range'][0]
    
    ## SELECT ENVIRONMENT
    if TrainConfig['ENV_Type'] == "SAR_2D_Env":
        env = SAR_2D_Env
    elif TrainConfig['ENV_Type'] == "SAR_Sim_DeepRL":
        env = SAR_Sim_DeepRL

    ## SET UP TRAINING CONDITIONS FROM CONFIG
    env_kwargs = TrainConfig['ENV_KWARGS']

    ## CREATE RL MANAGER
    RL_Manager = RL_Training_Manager(SAR_2D_Env,args.GroupName,TrainConfig['LogName'],env_kwargs=env_kwargs,S3_Upload=args.S3_Upload)    
    RL_Manager.fix_impact_leg_count()
