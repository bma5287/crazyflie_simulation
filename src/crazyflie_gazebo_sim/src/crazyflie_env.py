import numpy as np
import pandas as pd
import pyautogui,getpass

import socket, struct
from threading import Thread
import struct


import time
import os, subprocess, signal
import rospy
import csv

from socket import timeout

from sensor_msgs.msg import LaserScan, Image, Imu
from global_state_pkg.msg import GlobalState 
import message_filters
from cv_bridge import CvBridge

class CrazyflieEnv:
    def __init__(self,port_local=18050, port_Ctrl=18060):
        print("[STARTING] CrazyflieEnv is starting...")
        self.timeout = False # Timeout flag for reciever thread
        self.state_current = np.zeros(18)
        self.isRunning = True
        
        ## INIT ROS NODE FOR ENVIRONMENT 
        # NOTE: Can only have one node in a rospy process
        rospy.init_node("crazyflie_env_node") 
        self.launch_sim() 
    

        ## INIT RL SOCKET (AKA SERVER)
        self.RL_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Create UDP(DGRAM) Socket
        self.RL_port = port_local
        self.addr_RL = ("", self.RL_port) # Local address open to all incoming streams ("" = 0.0.0.0)
        self.RL_socket.bind(self.addr_RL) # Bind socket to specified port (0.0.0.0:18050)

        # Specify send/receive buffer sizes    
        self.RL_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 144) # State array from Ctrl [18 doubles]
        self.RL_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 40) # Action commands to Ctrl [5 doubles]
        self.RL_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
    
        ## INIT CONTROLLER ADDRESS ##
        self.port_Ctrl = port_Ctrl # Controller Server Port
        self.addr_Ctrl = ("", self.port_Ctrl) # Controller Address

        # kill -9 $(lsof -i:18050 -t)
        # if threads dont terminte when program is shut, you might need to use lsof to kill it

        self.global_state = GlobalState
        self.global_stateThread = Thread(target=self.global_stateSub,args=())
        self.global_stateThread.daemon=True
        self.global_stateThread.start()



   
        # =========== Sensors =========== #
        self.laser_msg = LaserScan # Laser Scan Message Variable
        self.laser_dist = 0
        # Start Laser data reciever thread
        self.laserThread = Thread(target = self.lsrThread, args=())
        self.laserThread.daemon=True
        self.laserThread.start()

        print("[COMPLETED] Environment done")



    def global_stateSub(self): # Subscriber for receiving global state info
        self.state_Sub = rospy.Subscriber('/global_state',GlobalState,self.global_stateCallback)
        rospy.spin()

    def global_stateCallback(self,data):
        gs_msg = data # gs_msg <= global_state_msg

        ## SET TIME VALUE FROM TOPIC
        t_temp = gs_msg.header.stamp.secs
        ns_temp = gs_msg.header.stamp.nsecs
        t = t_temp+ns_temp*1e-9 # (seconds + nanoseconds)
        
        ## SIMPLIFY STATE VALUES FROM TOPIC
        global_pos = gs_msg.global_pose.position
        global_quat = gs_msg.global_pose.orientation
        global_vel = gs_msg.global_twist.linear
        global_omega = gs_msg.global_twist.angular
        
        if global_quat.w == 0: # If zero at startup set quat.w to one to prevent errors
            global_quat.w = 1

        ## SET STATE VALUES FROM TOPIC
        position = [global_pos.x,global_pos.y,global_pos.z]
        orientation_q = [global_quat.w,global_quat.x,global_quat.y,global_quat.z]
        velocity = [global_vel.x,global_vel.y,global_vel.z]
        omega = [global_omega.x,global_omega.y,global_omega.z]
        ms = [gs_msg.motorspeeds[0],gs_msg.motorspeeds[1],gs_msg.motorspeeds[2],gs_msg.motorspeeds[3]]

        ## COMBINE INTO COMPREHENSIVE LIST
        self.state_current = [t] + position + orientation_q +velocity + omega + ms ## t (float) -> [t] (list)

        
        
        
        

    
    # ============================
    ##    Sensors/Gazebo Topics
    # ============================


    def lsrThread(self): # Thread for recieving laser scan messages
        print('[STARTING] Laser distance thread is starting...')
        self.laser_sub = rospy.Subscriber('/zranger2/scan',LaserScan,self.scan_callback)
        rospy.spin()

    def scan_callback(self,data): # callback function for laser subsriber
        self.laser_msg = data
        if  self.laser_msg.ranges[0] == float('Inf'):
            # sets dist to 4 m if sensor reads Infinity (no walls)
            self.laser_dist = 4 # max sesnsor dist
        else:
            self.laser_dist = self.laser_msg.ranges[0]


    # ============================
    ##       Sim Operation
    # ============================

    def close_sim(self):
            os.killpg(self.controller_p.pid, signal.SIGTERM)
            os.killpg(self.gazebo_p.pid, signal.SIGTERM)


    def delay_env_time(self,t_start,t_delay):
        # delay time defined in ms
        while (self.getTime() - t_start < t_delay/10000):
                    pass

    def __del__(self):
        self.isRunning = False
        self.RL_socket.close()


    def enableSticky(self, enable): # enable=0 disable sticky, enable=1 enable sticky
        header = 11
        msg = struct.pack('5d', header, enable, 0, 0, 0)
        self.RL_socket.sendto(msg, self.addr_Ctrl)
        time.sleep(0.001) # This ensures the message is sent enough times Gazebo catches it or something
        # the sleep time after enableSticky(0) must be small s.t. the gazebo simulation is satble. Because the simulation after joint removed becomes unstable quickly.

    def getTime(self):
        return self.state_current[0]

    def get_state(self,STATE): # function for thread that will continually read current state
        # Note: This can further be consolidated into global_stateCallback() **but I need a break and errors are hard
        while True:
            state = self.state_current
            qw = state[4]
            if qw==0: # Fix for zero-norm error during initialization where norm([qw,qx,qy,qz]=[0,0,0,0]) = undf
                state[4] = 1
            STATE[:] = state #.tolist() # Save to global array for access across multi-processes

    def launch_sim(self):
       
        # if getpass.getuser() == 'bhabas':
        #     pyautogui.moveTo(x=2500,y=0) 

        self.gazebo_p = subprocess.Popen( # Gazebo Process
            "gnome-terminal --disable-factory -- ~/catkin_ws/src/crazyflie_simulation/src/crazyflie_gazebo_sim/src/utility/launch_gazebo.bash", 
            close_fds=True, preexec_fn=os.setsid, shell=True)
        time.sleep(5)

        self.controller_p = subprocess.Popen( # Controller Process
            "gnome-terminal --disable-factory --geometry 81x33 -- ~/catkin_ws/src/crazyflie_simulation/src/crazyflie_gazebo_sim/src/utility/launch_controller.bash", 
            close_fds=True, preexec_fn=os.setsid, shell=True)
        time.sleep(1)

    def pause(self): #Pause simulation
        os.system("rosservice call gazebo/pause_physics")
        return self.state_current

    def resume(self): #Resume simulation
        os.system("rosservice call gazebo/unpause_physics")
        return self.state_current


    def reset_pos(self): # Disable sticky then places spawn_model at origin
        self.enableSticky(0)
        os.system("rosservice call gazebo/reset_world")
        return self.state_current          
            
    def step(self,action,ctrl_vals=[0,0,0],ctrl_flag=1): # Controller works to attain these values
        if action =='home': # default desired values/traj.
            header = 0
        elif action =='pos':  # position (x,y,z) 
            header = 1
        elif action =='vel':  # velocity (vx,vy,vz)
            header = 2
        elif action =='att':  # attitude: orientation (heading/yaw, pitch, roll/bank)
            header = 3
        elif action =='omega': # rotation rate (w_x:roll,w_y:pitch,w_z:yaw)
            header = 4
        elif action =='stop': # cut motors
            header = 5
        elif action =='gains': # assign new control gains
            header = 6
        else:
            header = 404
            print("no such action")
        cmd = struct.pack('5d', header, ctrl_vals[0], ctrl_vals[1], ctrl_vals[2], ctrl_flag) # Send command
        self.RL_socket.sendto(cmd, self.addr_Ctrl)
        time.sleep(0.01) # continually sends message during this time to ensure connection
        cmd = struct.pack('5d',8,0,0,0,1) # Send blank command so controller doesn't keep redefining values
        self.RL_socket.sendto(cmd, self.addr_Ctrl)

        #self.queue_command.put(buf, block=False)

        reward = 0
        done = 0
        info = 0
        return self.state_current, reward, done, info

    def cmd_send(self):
        while True:
            # Converts input number into action name
            cmd_dict = {0:'home',1:'pos',2:'vel',3:'att',4:'omega',5:'stop',6:'gains'}
            val = float(input("\nCmd Type (0:home,1:pos,2:vel,3:att,4:omega,5:stop,6:gains): "))
            action = cmd_dict[val]

            if action=='home' or action == 'stop': # Execute home or stop action
                ctrl_vals = [0,0,0]
                ctrl_flag = 1
                self.step(action,ctrl_vals,ctrl_flag)

            elif action=='gains': # Execture Gain changer
                
                vals = input("\nControl Gains (kp_x,kd_x,kp_R,kd_R): ") # Take in comma-seperated values and convert into list
                vals = [float(i) for i in vals.split(',')]
                ctrl_vals = vals[0:3]
                ctrl_flag = vals[3]

                self.step(action,ctrl_vals,ctrl_flag)
                
            elif action == 'omega': # Execture Angular rate action

                ctrl_vals = input("\nControl Vals (x,y,z): ")
                ctrl_vals = [float(i) for i in ctrl_vals.split(',')]
                ctrl_flag = 1.0

                self.step('omega',ctrl_vals,ctrl_flag)


            else:
                ctrl_vals = input("\nControl Vals (x,y,z): ")
                ctrl_vals = [float(i) for i in ctrl_vals.split(',')]
                ctrl_flag = float(input("\nController On/Off (1,0): "))
                self.step(action,ctrl_vals,ctrl_flag)

   



   

    

    # ============================
    ##       Data Recording 
    # ============================
    def create_csv(self,file_name,record=False):
        self.file_name = file_name
        self.record = record

        if self.record == True:
            with open(self.file_name, mode='w') as state_file:
                state_writer = csv.writer(state_file, delimiter=',',quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([
                'k_ep','k_run',
                'alpha_mu','alpha_sig',
                'mu','sigma', 'policy',
                't','x','y','z',
                'qx','qy','qz','qw',
                'vx','vy','vz',
                'wx','wy','wz',
                'gamma','reward','reward_avg','n_rollouts',
                'RREV','OF_x','OF_y',"","","","", # Place holders
                'error'])

    def IC_csv(self,agent,state,k_ep,k_run,policy,v_d,omega_d,reward,error_str):
        if self.record == True:
            with open(self.file_name, mode='a') as state_file:
                state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([
                    k_ep,k_run,
                    agent.alpha_mu.T,agent.alpha_sigma.T,
                    agent.mu.T,agent.sigma.T,policy.T,
                    "","","","", # t,x,y,z
                    "", "", "", "", # qx,qy,qz,qw
                    v_d[0],v_d[1],v_d[2], # vx,vy,vz
                    omega_d[0],omega_d[1],omega_d[2], # wx,wy,wz
                    agent.gamma,np.around(reward,2),"",agent.n_rollout,
                    "","","", # Sensor readings
                    "","","","", # Place holders
                    error_str])


    def append_csv(self,agent,state,k_ep,k_run,sensor):
        if self.record == True:
            state = np.around(state,3)
            sensor = np.around(sensor,3)

            with open(self.file_name, mode='a') as state_file:
                state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([
                    k_ep,k_run,
                    "","", # alpha_mu,alpha_sig
                    "","","", # mu,sig,policy
                    state[0],state[1],state[2],state[3], # t,x,y,z
                    state[4], state[5], state[6], state[7], # qx,qy,qz,qw
                    state[8], state[9],state[10], # vx,vy,vz
                    state[11],state[12],state[13], # wx,wy,wz
                    "","","","", # gamma, reward, "", n_rollout
                    sensor[0],sensor[1],sensor[2],
                    "","","","", # Place holders
                    ""]) # error


    def append_csv_blank(self): 
        if self.record == True:
            with open(self.file_name, mode='a') as state_file:
                state_writer = csv.writer(state_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                state_writer.writerow([])

    def load_csv(self,datapath,k_ep):
        ## Load csv and seperate first run of the selected episode
        df = pd.read_csv(datapath)
        ep_data = df.loc[ (df['k_ep'] == k_ep) & (df['k_run'] == 0)]
        
        ## Create a list of the main values to be loaded
        vals = []
        for k in range(2,6):
            val = ep_data.iloc[0,k]
            val_edited = np.fromstring(val[2:-2], dtype=float, sep=' ')
            val_array = val_edited.reshape(1,-1).T
            vals.append(val_array)

        alpha_mu,alpha_sig, mu, sigma = vals

        return alpha_mu,alpha_sig,mu,sigma