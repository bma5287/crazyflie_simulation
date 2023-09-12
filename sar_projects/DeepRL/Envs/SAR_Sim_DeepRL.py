#!/usr/bin/env python3

import gymnasium as gym
import numpy as np
from gymnasium import spaces

import rospy
import time
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

from sar_env import SAR_Sim_Interface

EPS = 1e-6 # Epsilon (Prevent division by zero)


class SAR_Sim_DeepRL(SAR_Sim_Interface,gym.Env):

    def __init__(self,GZ_Timeout=True,My_range=[-8.0,8.0],Vel_range=[1.5,3.5],Phi_rel_range=[0,90],Plane_Angle_range=[180,180],Tau_0=0.4):
        """
        Args:
            GZ_Timeout (bool, optional): Determines if Gazebo will restart if it freezed. Defaults to False.
            My_range (list, optional): Range of body moment actions (N*mm). Defaults to [0.0,8.0].
            Vel_range (list, optional): Range of flight velocities (m/s). Defaults to [1.5,3.5].
            Phi_range (list, optional): Range of flight angles (Deg). Defaults to [0,90].
            Tau_0 (float, optional): Flight position will start at this Tau value. Defaults to 0.4.
        """   
        SAR_Sim_Interface.__init__(self, GZ_Timeout)
        gym.Env.__init__(self)

        ## ENV CONFIG SETTINGS
        self.Env_Name = "SAR_Sim_DeepRL_Env"

        ## DOMAIN RANDOMIZATION SETTINGS
        self.Mass_std = 0.5e-3  # [kg]
        self.Iyy_std = 1.5e-6   # [kg*m^2]

        ## TESTING CONDITIONS
        self.Tau_0 = Tau_0          
        self.Vel_range = Vel_range  
        self.Phi_rel_range = Phi_rel_range
        self.Plane_Angle_range = Plane_Angle_range
        self.My_range = My_range

        ## RESET INITIAL VALUES
        self.K_ep = 0
        self.Trg_threshold = 0.5
        self.D_min = np.inf
        self.Tau_trg = np.inf
        self.Done = False

        ## DEFINE OBSERVATION SPACE
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32)
        self.obs_trg = np.zeros(self.observation_space.shape,dtype=np.float32) # Obs values at triggering

        ## DEFINE ACTION SPACE
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        self.action_trg = np.zeros(self.action_space.shape,dtype=np.float32) # Action values at triggering

    def reset(self, seed=None, options=None, V_mag=None, Phi_rel=None):
        """Resets the full pose and twist state of the robot and starts flight with specificed flight conditions.

        Args:
            vel (float, optional): Desired flight velocity [m/s]. Default to None for randomized conditions.
            phi (float, optional): Desired flight angle [deg]. Default to None for randomized conditions.

        Returns:
            observation array: 
        """        

        ## DETACH ROBOT AND UNPAUSE PHYSICS
        self.pause_physics(False)
        self.SendCmd('GZ_StickyPads',cmd_flag=0)

        ## SET PLANE POSE
        Plane_Angle_Low = self.Plane_Angle_range[0]
        Plane_Angle_High = self.Plane_Angle_range[1]
        self.setPlanePose([0,1.5,2],np.random.uniform(Plane_Angle_Low,Plane_Angle_High))

        ## UPDATE INERTIA VALUES (DOMAIN RANDOMIZATION)
        self.Iyy = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Iyy") + np.random.normal(0,self.Iyy_std)
        self.Mass = rospy.get_param(f"/SAR_Type/{self.SAR_Type}/Config/{self.SAR_Config}/Mass") + np.random.normal(0,self.Mass_std)
        # self.setModelInertia(Mass=self.Mass,Inertia=[self.Ixx,self.Iyy,self.Izz])

        ## RESET ROBOT STATE
        self.SendCmd('Tumble',cmd_flag=0)
        self.SendCmd('Ctrl_Reset')
        self.reset_pos()
        self.sleep(0.1)

        self.SendCmd('Tumble',cmd_flag=1)
        self.SendCmd('Ctrl_Reset')
        self.reset_pos()
        self.sleep(1.0) # Give time for drone to settle


        ## ACTIVATE STICKY PADS AND PAUSE PHYSICS
        self.SendCmd('GZ_StickyPads',cmd_flag=1)
        self.pause_physics(True)

        


        ## RESET RECORDED VALUES
        self.Done = False
        self.D_min = np.inf       # Reset max distance from landing surface [m]
        self.Tau_trg = np.inf     # Reset Tau triggering value [s]
        self.obs_trg = np.zeros_like(self.observation_space.high)
        self.action_trg = np.zeros_like(self.action_space.high)

        ## RESET LOGGING CONDITIONS 
        self.K_ep += 1
        self.eventCaptureFlag_flip = False      # Ensures flip data recorded only once
        self.eventCaptureFlag_impact = False    # Ensures impact data recorded only once 

        

        ## RESET POSITION RELATIVE TO LANDING SURFACE (BASED ON STARTING TAU VALUE)
        # (Derivation: Research_Notes_Book_3.pdf (6/22/23))
        r_PO = np.array(self.Plane_Pos)                             # Plane Position w/r to origin
        n_hat,t_x,t_y = self._calc_PlaneNormal(self.Plane_Angle)    # Plane direction vectors

        ## SAMPLE VELOCITY AND FLIGHT ANGLE
        if V_mag == None or Phi_rel == None:
            V_mag,Phi_rel = self._sample_flight_conditions()

        else:
            V_mag = V_mag           # Flight velocity
            Phi_rel = Phi_rel   # Flight angle  

        ## RELATIVE VEL VECTORS
        V_perp = V_mag*np.sin(np.deg2rad(Phi_rel))
        V_tx = V_mag*np.cos(np.deg2rad(Phi_rel))
        V_ty = 0
        V_BP = np.array([V_tx,V_ty,V_perp])

        ## ROTATION MATRIX FROM PLANE TO WORLD COORDS
        R_PO = np.vstack((t_x,t_y,n_hat)).T 

        ## CALCULATE GLOBAL VEL VECTORS
        V_BO = R_PO.dot(V_BP)
        V_hat = V_BO/np.linalg.norm(V_BO)   # Flight Velocity unit vector


        ## CALC STARTING DISTANCE WHERE POLICY IS MONITORED
        if ((Phi_rel <= 0) or (Phi_rel >= 180)):
            D_perp = 0.1
            self.Done = True

        elif (0 < Phi_rel <= 5) or (175 <= Phi_rel < 180):
            Tau_0 = 1.5
            D_perp = max((Tau_0*V_perp),0.1) # Ensure a reasonable minimum perp distance [m]
            
        elif (5 < Phi_rel < 20) or (160 < Phi_rel < 175):
            Tau_0 = self.Tau_0
            D_perp = (self.Tau_0*V_perp) 
        else:
            Tau_0 = self.Tau_0
            D_perp = (self.Tau_0*V_perp) 


        ## CALC DISTANCE REQUIRED TO SETTLE ON DESIRED VELOCITY
        t_settle = 1.5              # Time for system to settle on flight conditions
        D_settle = t_settle*V_mag     # Flight settling distance

        ## INITIAL POSITION RELATIVE TO PLANE
        r_BP = -(D_perp/(V_hat.dot(n_hat)+EPS) + D_settle)*V_hat

        ## INITIAL POSITION IN GLOBAL COORDS
        r_BO = r_PO + r_BP 

        ## LAUNCH QUAD W/ DESIRED VELOCITY
        self.Vel_Launch(r_BO,V_BO)
        self.iter_step(t_settle*1e3)
        

        ## RESET/UPDATE RUN CONDITIONS
        self.start_time_rollout = self.getTime()
        self.start_time_pitch = np.nan
        self.start_time_impact = np.nan
        self.start_time_ep = time.time()

        return self._get_obs(), {}
    
    def step(self, action):

        ########## PRE-FLIP TRIGGER ##########
        if action[0] < self.Trg_threshold:

            ## GRAB CURRENT OBSERVATION
            obs = self._get_obs()

            ## CHECK FOR DONE
            self.Done = bool(
                self.Done
                or (self.impact_flag or self.BodyContact_flag)  # BODY CONTACT W/O FLIP TRIGGER
            )       

            ## UPDATE MINIMUM DISTANCE
            D_perp = obs[2]
            if not self.Done:
                if D_perp <= self.D_min:
                    self.D_min = D_perp 


            terminated = self.Done
            truncated = bool(self.t - self.start_time_rollout > 3.5) # EPISODE TIMEOUT (SIM TIME)

            ## CALCULATE REWARD
            reward = 0

            ## UPDATE SIM 
            self.iter_step() ## WATCH PLACEMENT OF THIS?

            ## UDPATE OBSERVATION
            obs = self._get_obs()

        ########## POST-FLIP TRIGGER ##########
        elif action[0] >= self.Trg_threshold:

            ## GRAB CURRENT OBSERVATION
            obs = self._get_obs()   # Return this observation because reward and future 
                                    # are defined by action taken here. Not obs at end of episode.

            ## SAVE TRIGGERING OBSERVATION AND ACTIONS
            self.obs_trg = obs
            self.Tau_trg = obs[0]
            self.action_trg = action

            ## COMPLETE REST OF SIMULATION
            self._finish_sim(action)

            terminated = self.Done = True
            truncated = False
            
            ## CALCULATE REWARD
            reward = self._CalcReward()

        if terminated or truncated:
            self._RL_Publish()

        return (
            obs,
            reward,
            terminated,
            truncated,
            {},
        )


    def render(self):
        ## DO NOTHING ##
        return


    def close(self):
        ## DO NOTHING ##
        return
    

    def _finish_sim(self,action):
        """This function continues the remaining steps of the simulation at full speed 
        since policy actions only continue up until flip trigger.

        Args:
            action (np.array): Action to be performed by controller
        """        

        ## SCALE ACTION
        scaled_action = 0.5 * (action[1] + 1) * (self.My_range[1] - self.My_range[0]) + self.My_range[0]

        ## SEND FLIP ACTION TO CONTROLLER
        Tau_max = 100 # Automatically trigger policy
        self.SendCmd("Policy",[Tau_max,scaled_action,0],cmd_flag=1) # Body rotational moment [N*mm]

        ## RUN REMAINING STEPS AT FULL SPEED
        self.pause_physics(False)

        while not self.Done:

            ## START IMPACT TERMINATION TIMERS
            if ((self.impact_flag or self.BodyContact_flag) and self.eventCaptureFlag_impact == False):
                self.start_time_impact = self.getTime()
                self.eventCaptureFlag_impact = True

            ## CHECK IF TIMERS ARE EXCEEDED
            self.Done = bool(
                self.Done
                or self.t - self.start_time_rollout > 3.5   # EPISODE TIMEOUT (SIM TIME)
                or self.t - self.start_time_impact > 1.0    # IMPACT TIMEOUT (SIM TIME)
                or (self.V_perp <= -5.0 and self.D_perp >= 1.0) # IF SAR IS FALLING
            )

            ## UPDATE MINIMUM DISTANCE
            if not self.Done: # Need live (and rough) updates here so use self.D_perp from controller
                if self.D_perp <= self.D_min:
                    self.D_min = self.D_perp 


    def _get_obs(self):

        ## CREATE SERVICE MESSAGE
        model_srv = GetModelStateRequest()
        model_srv.model_name = self.SAR_Config 
        resp = self.callService('/gazebo/get_model_state',model_srv,GetModelState)

        pos = resp.pose.position
        vel = resp.twist.linear
        n_hat,t_x,t_y = self._calc_PlaneNormal(self.Plane_Angle)

        ## UPDATE POS AND VEL
        r_BO = np.array([pos.x,pos.y,pos.z])
        V_BO = np.array([vel.x,vel.y,vel.z])

        ## CALC DISPLACEMENT FROM PLANE CENTER
        r_PO = np.array(self.Plane_Pos)

        ## CALC DISPLACEMENT FROM PLANE CENTER
        r_PO = np.array(self.Plane_Pos)
        r_PB = r_PO - r_BO

        ## CALC RELATIVE DISTANCE AND VEL
        D_perp = r_PB.dot(n_hat) + 1e-6
        V_perp = V_BO.dot(n_hat) + 1e-6
        V_tx = V_BO.dot(t_x)
        V_ty = V_BO.dot(t_y)

        ## CALC OPTICAL FLOW VALUES
        Theta_x = np.clip(V_tx/D_perp,-20,20)
        Theta_y = np.clip(V_ty/D_perp,-20,20)
        Tau = np.clip(D_perp/V_perp,0,3)

        def normalize(val,min_val,max_val):

            norm_val = (val-min_val)/(max_val-min_val)

            return norm_val
            
        Tau_norm = normalize(Tau,0,3)
        Theta_x_norm = normalize(Theta_x,-20,20)
        D_perp_norm = normalize(D_perp,0,4)
        Plane_Angle_norm = normalize(self.Plane_Angle,0,180)


        return np.array([Tau,Theta_x,D_perp,self.Plane_Angle],dtype=np.float32)

    def _sample_flight_conditions(self):
        """This function samples the flight velocity and angle from the supplied range.
        Velocity is sampled from a uniform distribution. Phi is sampled from a set of 
        uniform distributions which are weighted such that edge cases are only sampled 10% of the time.
        Poor performance on edge cases can cause poor learning convergence.

        Returns:
            V_mag,Phi_rel: Sampled flight velocity and flight angle
        """        

        ## SAMPLE VEL FROM UNIFORM DISTRIBUTION IN VELOCITY RANGE
        Vel_Low = self.Vel_range[0]
        Vel_High = self.Vel_range[1]
        V_mag = np.random.uniform(low=Vel_Low,high=Vel_High)

        ## SAMPLE RELATIVE PHI FROM A WEIGHTED SET OF UNIFORM DISTRIBUTIONS
        Phi_rel_Low = self.Phi_rel_range[0]
        Phi_rel_High = self.Phi_rel_range[1]
        Phi_rel_Range = Phi_rel_High-Phi_rel_Low

        Dist_Num = np.random.choice([0,1,2],p=[0.1,0.8,0.1]) # Probability of sampling distribution

        if Dist_Num == 0: # Low Range
            Phi_rel = np.random.default_rng().uniform(low=Phi_rel_Low, high=Phi_rel_Low + 0.1*Phi_rel_Range)
        elif Dist_Num == 1: # Medium Range
            Phi_rel = np.random.default_rng().uniform(low=Phi_rel_Low + 0.1*Phi_rel_Range, high=Phi_rel_High - 0.1*Phi_rel_Range)
        elif Dist_Num == 2: # High Range
            Phi_rel = np.random.default_rng().uniform(low=Phi_rel_High - 0.1*Phi_rel_Range, high=Phi_rel_High)
       
        return V_mag,Phi_rel

    def _calc_PlaneNormal(self,Plane_Angle):

        Plane_Angle_rad = np.deg2rad(Plane_Angle)

        ## PRE-INIT ARRAYS
        t_x =   np.array([1,0,0],dtype=np.float64)
        t_y =   np.array([0,1,0],dtype=np.float64)
        n_hat = np.array([0,0,1],dtype=np.float64)

        ## DEFINE PLANE TANGENT UNIT-VECTOR
        t_x[0] = -np.cos(Plane_Angle_rad)
        t_x[1] = 0
        t_x[2] = -np.sin(Plane_Angle_rad)

        ## DEFINE PLANE TANGENT UNIT-VECTOR
        t_y[0] = 0
        t_y[1] = 1
        t_y[2] = 0

        ## DEFINE PLANE NORMAL UNIT-VECTOR
        n_hat[0] = np.sin(Plane_Angle_rad)
        n_hat[1] = 0
        n_hat[2] = -np.cos(Plane_Angle_rad)

        return n_hat,t_x,t_y

    def _CalcReward(self):

        ## DISTANCE REWARD 
        R_dist = np.clip(1/np.abs(self.D_min + 1e-3),0,15)/15
        
        ## TAU TRIGGER REWARD
        R_tau = np.clip(1/np.abs(self.Tau_trg - 0.2),0,15)/15


        ## IMPACT ANGLE REWARD # (Derivation: Research_Notes_Book_3.pdf (6/21/23))
        Beta_d = 150 # Desired impact angle [= 180 deg - gamma [deg]] TODO: Change to live value?
        Beta_rel = -180 + self.Rot_Sum_impact + self.Plane_Angle

        if Beta_rel < -180:
            R_angle = 0
        elif -180 <= Beta_rel < 180:
            R_angle = 0.5*(-np.cos(np.deg2rad(Beta_rel*180/Beta_d))+1)
        elif Beta_rel > 180:
            R_angle = 0

        ## PAD CONTACT REWARD
        if self.pad_connections >= 3: 
            if self.BodyContact_flag == False:
                R_legs = 1.0
            else:
                R_legs = 0.3

        elif self.pad_connections == 2: 
            if self.BodyContact_flag == False:
                R_legs = 0.6
            else:
                R_legs = 0.1
                
        else:
            R_legs = 0.0

        self.reward_vals = [R_dist,R_tau,R_angle,R_legs,0]
        self.reward = 0.05*R_dist + 0.1*R_tau + 0.2*R_angle + 0.65*R_legs


        return self.reward

if __name__ == "__main__":

    env = SAR_Sim_DeepRL(GZ_Timeout=False,My_range=[-8.0,8.0],Vel_range=[3.0,3.0],Phi_rel_range=[20,160],Plane_Angle_range=[180,180])

    for ep in range(20):

        V_mag = 1
        Phi_rel = 179

        obs,_ = env.reset(V_mag=V_mag,Phi_rel=Phi_rel)

        Done = False
        while not Done:

            action = env.action_space.sample() # obs gets passed in here
            action[0] = 0
            obs,reward,terminated,truncated,_ = env.step(action)
            Done = terminated or truncated

        print(f"Episode: {ep} \t Reward: {reward:.3f} \t Reward_vec: ",end='')
        print(' '.join(f"{val:.2f}" for val in env.reward_vals))
