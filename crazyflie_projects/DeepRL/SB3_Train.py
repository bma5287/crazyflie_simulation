import os
from datetime import datetime
from stable_baselines3 import A2C,PPO

from Tau_Trigger_Env import Tau_Trigger_Env


## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%H_%M")

## INITIATE ENVIRONMENT
env = Tau_Trigger_Env()
env.reset()

## CREATE MODEL AND LOG DIRECTORY
models_dir = f"crazyflie_projects/DeepRL/models/{env.env_name}/PPO-{current_time}"
log_dir = "/tmp/logs"

if not os.path.exists(models_dir):
    os.makedirs(models_dir)

if not os.path.exists(log_dir):
    os.makedirs(log_dir)



model = PPO("MlpPolicy",env,verbose=1,tensorboard_log=log_dir) 

## TRAIN MODEL AND SAVE MODEL EVERY N TIMESTEPS
TIMESTEPS = 10_000
for i in range(1,60):

    if i == 1:
        reset_timesteps = True
    else:
        reset_timesteps = False

    model.learn(
        total_timesteps=TIMESTEPS, 
        reset_num_timesteps=reset_timesteps, 
        tb_log_name=f"{env.env_name}-PPO-{current_time}"
    )
    model.save(f"{models_dir}/{TIMESTEPS*i//1000:d}.zip")


## RENDER TRAINED MODEL FOR N EPISODES
episodes = 10
for ep in range(episodes):
    obs = env.reset()
    done = False
    while not done:
        env.render()
        action,_ = model.predict(obs)
        obs,reward,done,info = env.step(action)

env.close()