import os,fnmatch
import send2trash
import numpy as np
os.system("clear")

## CREATE LINK TO DATAPATH MODULE
import sys
sys.path.insert(0,'/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/utility')
from data_analysis import DataFile


dataPath = '/home/bhabas/catkin_ws/src/crazyflie_simulation/src/crazyflie_rl/src/log/Wide-Short_PureMoment_Data_1-29-21/'


redoList = []
fileList = []
test_list = []

## DEFINE DATA RANGE TO ITERATE OVER
vz_array = np.arange(4.0,2.0,-0.25)
vx_array = np.arange(0,3.0,0.25)

# vz_array = np.array([3.0])
# vx_array = np.array([0.0])

## GENERATE TEST ARRAY
for vz_d in vz_array:       # Limits: [1.5,3.5]
    for vx_d in vx_array:   # Limits: [0,3.0]
        test_list.append([vz_d,vx_d])
test_arr = np.asarray(test_list)



## ITERATE THROUGH ALL COMBINATIONS
for vz,vx in test_arr:
    
    ## CREATE LIST OF FILES THAT MATCH IC Vz,Vx
    for fileName in os.listdir(dataPath):
        if fnmatch.fnmatch(fileName, f'*Vz_{vz:.2f}--Vx_{vx:.2f}*'):
            fileList.append(fileName)

    ## SORT FILE LIST TO ASCENDING TRIAL NUMBER ORDER
    fileList = sorted(fileList)

    ## ITERATE THROUGH FILE LIST
    for fileName in fileList:
        filepath = dataPath+fileName
        
        ## CREATE TRIAL OBJECT FOR CURRENT FILE
        trial = DataFile(dataPath,fileName)
        
        try:
            
            ## IF BROKEN RUN THEN JUST REMOVE
            if trial.rewardAvg_trial() <= 15:
                raise Exception
            if trial.k_epMax <= 15:
                raise  Exception
            if trial.landing_rate() <= 0.2:
                raise Exception

            # print(trial.rewardAvg_trial())
            # trial.plotSummary()
            
            # SHOW FILENAME AND SUMMARY
            print(fileName)
            
        except: ## IF ERROR DELETE FILE
            print(f"Deleting File: {fileName}")
            redoList.append([fileName])

            send2trash.send2trash(filepath)


    # ## INPUT TRIAL NUMBERS TO BE DISCARDED
    # str = input('Input spaced trial numbers to be deleted: ')
    # try:
    #     trialList = [int(i) for i in str.split(' ')]
    #     for trial in trialList:
    #         ## APPEND TRIAL TO LIST TO BE REDONE AND SAVE CURRENT ARRRAY TO CSV
    #         # np.savetxt("runAgain_List.csv", np.asarray(redoList), delimiter=",")

    #         ## RECREATE FILE NAME AND DELETE FILE
    #         filepath = f"{dataPath}EM_PEPG--Vz_{vz:.2f}--Vx_{vx:.2f}--trial_{trial}.csv"
    #         send2trash.send2trash(filepath)
    # except:
    #     pass


# ## PRINT REDO ARRAY AND SAVE TO CSV FILE
print(np.asarray(redoList))
np.savetxt("runAgain_List.csv", np.asarray(redoList), delimiter=",")
    


