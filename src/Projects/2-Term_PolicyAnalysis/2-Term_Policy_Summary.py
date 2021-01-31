import os,fnmatch
import numpy as np
import pandas as pd
os.system("clear")

## CREATE LINK TO DATAPATH MODULE
import sys
sys.path.insert(0,'/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/utility')
from data_analysis import DataFile


dataPath = "/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/Wide-Short_Vx-0.0_2Policy/"

## GENERATE INITIAL LIST VARIABLES
test_list = []
df_list = []



## ITER OVER ALL FILES IN DIR
for fileName in os.listdir(dataPath): # Iter over all files in dir
    
        
    trial = DataFile(dataPath,fileName)
    print(fileName)
    
    vx_d,_,vz_d = trial.v_d
    trial_num = fileName[-5]
    landing_rate = trial.landing_rate()

    alpha_mu,alpha_sigma,mu_ini,sigma_ini = trial.grab_RLPararms()
    

    policy,sigma = trial.grab_finalPolicy()
    RREV_trigger,G1 = policy
    RREV_sig,G1_sig = sigma

    My_d = trial.grab_My_d_trial()
    impact_eul = trial.grab_impact_eul_trial('eul_y')


    df_list.append((
        vz_d,vx_d,trial_num,landing_rate,
        RREV_trigger,G1,
        RREV_sig,G1_sig,
        My_d,impact_eul,
        alpha_mu,alpha_sigma,
        mu_ini,sigma_ini,
        ))


            
master_df = pd.DataFrame(df_list,columns=(
    'vz_d','vx_d','trial_num','landing_rate',
    'RREV_trigger','G1',
    'RREV_sig','G1_sig',
    'My_d','impact_eul',
    'alpha_mu','alpha_sigma',
    'mu_ini','sigma_ini',
))
master_df.sort_values(['vz_d','vx_d','trial_num'],ascending=[1,1,1],inplace=True)
print(master_df)

master_df.to_csv('2-Term_Policy_Summary.csv',index=False)
