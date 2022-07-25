from datetime import datetime
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import *
import torch as th

from CF_Env_2D import CF_Env_2D
from CF_Env_2D_dTau import CF_Env_2D_dTau
from CF_Env_2D_Simple import CF_Env_2D_Simple

## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%H-%M")

## INITIATE ENVIRONMENT
env = CF_Env_2D()

class CheckpointSaveCallback(BaseCallback):

    def __init__(self, save_freq: int, log_dir: str, log_name: str, verbose: int = 0):
        super(CheckpointSaveCallback, self).__init__(verbose)
        self.save_freq = save_freq
        self.log_dir = os.path.join(log_dir,log_name+"_0")
        self.model_dir = os.path.join(self.log_dir,"models")
        self.replay_dir = self.log_dir

    def _init_callback(self) -> None:
        # Create folder if needed
        if self.log_dir is not None:
            os.makedirs(self.log_dir, exist_ok=True)
            os.makedirs(self.model_dir,exist_ok=True)


    def _on_step(self) -> bool:
        if self.n_calls % self.save_freq == 0:
            model_path = os.path.join(self.model_dir, f"{self.num_timesteps}_steps")
            self.model.save(model_path)

            replay_buff_path = os.path.join(self.model_dir, f"replay_buff")
            self.model.save_replay_buffer(replay_buff_path)

            if self.verbose > 1:
                print(f"Saving model checkpoint to {model_path}")
        return True

    def _on_rollout_end(self) -> None:
        self.logger.record('time/K_ep',self.training_env.envs[0].env.k_ep)
        return True

## CREATE MODEL AND LOG DIRECTORY
log_dir = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/DeepRL/logs/{env.env_name}/"
log_name = f"SAC-{current_time}"
checkpoint_callback = CheckpointSaveCallback(save_freq=1000,log_dir=log_dir,log_name=log_name)
    
policy_kwargs = dict(activation_fn=th.nn.ReLU,
                     net_arch=[12, 12])
model = SAC(
    "MlpPolicy",
    env=env,
    gamma=0.999,
    learning_rate=0.001,
    policy_kwargs=policy_kwargs,
    verbose=1,
    device='cpu',
    tensorboard_log=log_dir
) 

# model.set_parameters(
#     load_path_or_dict=f"{log_dir}/CF_Env_2D/SAC-15-23_0/models/117500_steps.zip",
#     device='cpu'
# )

# model = SAC.load(
#     path=f"{log_dir}/SAC-14-43_0/models/10000_steps.zip",
#     env=env,
#     device='cpu'
#     )
# model.load_replay_buffer(
#     path=f"{log_dir}/SAC-14-43_0/models/replay_buff.pkl")

model.learn(
    total_timesteps=1e6,
    tb_log_name=log_name,
    callback=checkpoint_callback,
    reset_num_timesteps=False
)