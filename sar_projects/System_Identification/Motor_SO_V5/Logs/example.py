import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
import os


import sys,rospkg
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_projects'))


RPM_TO_RAD_PER_SEC = 2*np.pi/60
NUM_BATT_CELLS = 6


## DATAPATHS
dataPath = f"{BASE_PATH}/sar_projects/System_Identification/Motor_SO_V5/Logs/"
fileName = f"StepsTestV2_2023-12-14_112823.csv"
filePath = os.path.join(dataPath,fileName)

## LOAD DATA
df = pd.read_csv(filePath,comment="#")

df["Time (s)"] = df["Time (s)"] + 551

df.to_csv("StepsTestV2_2023-12-14_112823_Copy.csv",index=False)