#!/bin/bash

## FIND SIMULATION PATH
DEEP_RL_PATH=$(find ~/catkin_ws/src -name 'sar_simulation' -type d | head -n 1)/sar_projects/DeepRL

## 90 DEG TRAINING AND DATA COLLECTION
python $DEEP_RL_PATH/T2_Policy_FineTuning.py \
    --GroupName IM_3D/aRot90/K0.4_DR1.0/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/IM_3D_Sim/aRot90_K0.4_DR1.0/C2/IM_C2_90deg_S2D.json \
    --PT_GroupName IM_2D_aRot90/90deg \
    --PT_TrainConfig $DEEP_RL_PATH/Config_Files/IM_2D_Sim/aRot90/C1/IM_C1_90deg_S2D.json \
    --t_step_load 150000 \
    --S3_Upload true && \





python $DEEP_RL_PATH/T3_Policy_Data_Collection.py \
    --GroupName IM_3D/aRot90/K0.4_DR1.0/90deg \
    --TrainConfig $DEEP_RL_PATH/Config_Files/IM_3D_Sim/aRot90_K0.4_DR1.0/C2/IM_C2_90deg_S2D.json \
    --S3_Upload true \
    --t_step_load 85500
