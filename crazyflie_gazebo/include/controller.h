#include <iostream>
#include <thread>
#include <cmath>        // std::abs
#include <math.h>       
#include "math3d.h"
#include "nml.h"

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <unistd.h>


// ROS Includes
#include <ros/ros.h>
#include "crazyflie_msgs/CtrlData.h"
#include "crazyflie_msgs/ImpactData.h"
#include "crazyflie_msgs/RLCmd.h"
#include "crazyflie_msgs/RLData.h"
#include "crazyflie_msgs/PadConnect.h"
#include "crazyflie_msgs/Policy_Values.h"
#include "crazyflie_msgs/MS.h"


#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "gazebo_msgs/SetPhysicsProperties.h"



using namespace std;

#define PWM_MAX 60000
#define f_MAX (16.5)
#define g2Newton (9.81f/1000.0f)
#define Newton2g (1000.0f/9.81f)




class Controller
{
    public:
        // CONSTRUCTOR TO START PUBLISHERS AND SUBSCRIBERS (Similar to Python's __init__() )
        Controller(ros::NodeHandle *nh){
            ctrl_Publisher = nh->advertise<crazyflie_msgs::CtrlData>("/ctrl_data",1);
            MS_Publisher = nh->advertise<crazyflie_msgs::MS>("/MS",1);

            // NOTE: tcpNoDelay() removes delay where system is waiting for datapackets to be fully filled before sending;
            // instead of sending data as soon as it is available to match publishing rate (This is an issue with large messages like Odom or Custom)
            // Queue lengths are set to '1' so only the newest data is used


            // BODY SENSORS
            OF_Subscriber = nh->subscribe("/cf1/OF_sensor",1,&Controller::OFCallback,this,ros::TransportHints().tcpNoDelay()); 
            imu_Subscriber = nh->subscribe("/cf1/imu",1,&Controller::imuCallback,this);
                
            // ENVIRONMENT SENSORS
            globalState_Subscriber = nh->subscribe("/env/vicon_state",1,&Controller::vicon_stateCallback,this,ros::TransportHints().tcpNoDelay());
            ceilingFT_Subcriber = nh->subscribe("/env/ceiling_force_sensor",5,&Controller::ceilingFTCallback,this,ros::TransportHints().tcpNoDelay());


            // COMMANDS AND INFO
            RLCmd_Subscriber = nh->subscribe("/rl_ctrl",50,&Controller::GTC_Command,this);
            RLData_Subscriber = nh->subscribe("/rl_data",5,&Controller::RLData_Callback,this,ros::TransportHints().tcpNoDelay());
            SimSpeed_Client = nh->serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");
        

            



            // INIT VARIABLES TO DEFAULT VALUES (PREVENTS RANDOM VALUES FROM MEMORY)

            _RREV = 0.0;
            _OF_x = 0.0;
            _OF_y = 0.0;

            

            ros::param::get("/CEILING_HEIGHT",_H_CEILING);
            ros::param::get("/LANDING_SLOWDOWN",_LANDING_SLOWDOWN_FLAG);
            ros::param::get("/SIM_SPEED",_SIM_SPEED);
            ros::param::get("/SIM_SLOWDOWN_SPEED",_SIM_SLOWDOWN_SPEED);
            ros::param::get("/CF_MASS",_CF_MASS);
            ros::param::get("/CTRL_DEBUG_SLOWDOWN", _CTRL_DEBUG_SLOWDOWN);
            ros::param::get("/POLICY_TYPE",_POLICY_TYPE);
            Policy_Type _POLICY_TYPE = (Policy_Type)_POLICY_TYPE; // Cast ROS param (int) to enum (Policy_Type)

            // COLLECT CTRL GAINS FROM CONFIG FILE
            ros::param::get("P_kp_xy",P_kp_xy);
            ros::param::get("P_kd_xy",P_kd_xy);
            ros::param::get("P_ki_xy",P_ki_xy);
            ros::param::get("i_range_xy",i_range_xy);

            ros::param::get("P_kp_z",P_kp_z);
            ros::param::get("P_kd_z",P_kd_z);
            ros::param::get("P_ki_z",P_ki_z);
            ros::param::get("i_range_z",i_range_z);

            ros::param::get("R_kp_xy",R_kp_xy);
            ros::param::get("R_kd_xy",R_kd_xy);
            ros::param::get("R_ki_xy",R_ki_xy);
            ros::param::get("i_range_R_xy",i_range_R_xy);
            
            ros::param::get("R_kp_z",R_kp_z);
            ros::param::get("R_kd_z",R_kd_z);
            ros::param::get("R_ki_z",R_ki_z);
            ros::param::get("i_range_R_z",i_range_R_z);



            m = _CF_MASS;
            h_ceiling = _H_CEILING;
            

            // FILE* input = fopen("/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_gazebo/src/data/NN_Layers.data", "r");
            // // LAYER 1
            // nml_mat *W1 = nml_mat_fromfilef(input);
            // nml_mat *b_1 = nml_mat_fromfilef(input);

            // // LAYER 2
            // nml_mat *W2 = nml_mat_fromfilef(input);
            // nml_mat *b_2 = nml_mat_fromfilef(input);

            // // LAYER 3
            // nml_mat *W3 = nml_mat_fromfilef(input);
            // nml_mat *b_3 = nml_mat_fromfilef(input);
            // fclose(input);


            
            

            

            // W[0] = W1;
            // W[1] = W2;
            // W[2] = W3;

            // b[0] = b_1;
            // b[1] = b_2;
            // b[2] = b_3;
        }

        // DEFINE FUNCTION PROTOTYPES
        void Load();
        void controllerGTC();
        void controllerGTCReset();
        void controllerGTCTraj();
        void GTC_Command(const crazyflie_msgs::RLCmd::ConstPtr &msg);

        void vicon_stateCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void OFCallback(const nav_msgs::Odometry::ConstPtr &msg);           
        void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

        void RLData_Callback(const crazyflie_msgs::RLData::ConstPtr &msg);

        void ceilingFTCallback(const crazyflie_msgs::ImpactData::ConstPtr &msg);
        void adjustSimSpeed(float speed_mult);
        float NN_Policy(nml_mat* X, nml_mat* W[], nml_mat* b[]);
        // bool NN_flip(nml_mat*)








    private:
        // DEFINE PUBLISHERS AND SUBSCRIBERS
        ros::Publisher ctrl_Publisher;
        ros::Publisher MS_Publisher;

        // SENSORS
        ros::Subscriber globalState_Subscriber;
        ros::Subscriber OF_Subscriber;
        ros::Subscriber imu_Subscriber;


        ros::Subscriber ceilingFT_Subcriber;


        // COMMANDS AND INFO
        ros::Subscriber RLCmd_Subscriber;
        ros::Subscriber RLData_Subscriber;
        ros::ServiceClient SimSpeed_Client;


        


        // INITIALIZE ROS MSG VARIABLES
        geometry_msgs::Point _position; 
        geometry_msgs::Vector3 _velocity;
        geometry_msgs::Quaternion _quaternion;
        geometry_msgs::Vector3 _omega;
        geometry_msgs::Vector3 _accel;



        // DEFINE THREAD OBJECTS
        std::thread controllerThread;


        

        // =============================
        //  INITIALIZE CLASS VARIABLES
        // =============================


        // SYSTEM PARAMETERS
        float m = 0.037f;       // Crazyflie Mass [g]
        float g = 9.81f;        // Gravity [m/s^2]
        struct mat33 J;         // Rotational Inertia Matrix [kg*m^2]

        float d = 0.040f;       // COM to Prop [m]
        float dp = 0.028284;    // COM to Prop along x-axis [m]
                                // [dp = d*sin(45 deg)]

        float const kf = 2.2e-8f;       // Thrust Coeff [N/(rad/s)^2]
        float const c_tf = 0.00618f;    // Moment Coeff [Nm/N]


        float h_ceiling = 2.10f;    // [m]
        float dt = (1.0f/500.0f);
        bool _isRunning; // Keeps controller running in while loop
        

        

        nml_mat* W[3];
        nml_mat* b[3];


        

        // ROS SPECIFIC VALUES
        int impact_flag;
        int slowdown_type = 0;
        float _H_CEILING;
        bool _LANDING_SLOWDOWN_FLAG;
        float _SIM_SPEED; 
        float _SIM_SLOWDOWN_SPEED;
        float _CF_MASS;
        int _CTRL_DEBUG_SLOWDOWN;


        // INITIAL CONTROLLER GAIN VALUES (OVERWRITTEN BY CONFIG FILE)
        // XY POSITION PID
        float P_kp_xy = 0.5f;
        float P_kd_xy = 0.4f;
        float P_ki_xy = 0.0f;
        float i_range_xy = 0.3f;

        // Z POSITION PID
        float P_kp_z = 1.2f;
        float P_kd_z = 0.35f;
        float P_ki_z = 0.0f;
        float i_range_z = 0.25f;

        // XY ATTITUDE PID
        float R_kp_xy = 0.004f;
        float R_kd_xy = 0.0017f;
        float R_ki_xy = 0.0f;
        float i_range_R_xy = 1.0f;

        // Z ATTITUDE PID
        float R_kp_z = 0.003f;
        float R_kd_z = 0.001f;
        float R_ki_z = 0.0f;
        float i_range_R_z = 0.5f;

        
        
        bool _TEST_FLAG = false;

        ros::Time t_flip;


        double _RREV = 0.0;  // [rad/s]
        double _OF_x = 0.0;  // [rad/s]
        double _OF_y = 0.0;  // [rad/s]

        // STATE VALUES AT FLIP TRIGGER
        float RREV_tr = 0.0f;
        float OF_x_tr = 0.0f;
        float OF_y_tr = 0.0f;

        struct vec statePos_tr = {0.0f,0.0f,0.0f};         // Pos [m]
        struct vec stateVel_tr = {0.0f,0.0f,0.0f};         // Vel [m/s]
        struct quat stateQuat_tr = {0.0f,0.0f,0.0f,1.0f};  // Orientation
        struct vec stateOmega_tr = {0.0f,0.0f,0.0f};       // Angular Rate [rad/s]


        float f_thrust_g_tr = 0.0f;
        float f_roll_g_tr = 0.0f;
        float f_pitch_g_tr = 0.0f;
        float f_yaw_g_tr = 0.0f;

        float F_thrust_flip = 0.0f;
        float M_x_flip = 0.0f;
        float M_y_flip = 0.0f;
        float M_z_flip = 0.0f;


        // POLICY VARIABLES
        float RREV_thr = 100.0f;
        float G1 = 0.0f;
        float G2 = 0.0f;

        bool policy_armed_flag = false;
        typedef enum 
        {
            RL = 0,
            NN = 1
        } Policy_Type;
        int _POLICY_TYPE;

        // CONTROLLER PARAMETERS
        bool attCtrlEnable = false;
        bool tumbled = false;
        bool tumble_detection = true;
        bool motorstop_flag = false;
        bool errorReset = false;
        bool flip_flag = false;
        bool onceFlag = false;

        // TRAJECTORY VARIABLES
        float s_0 = 0.0f;
        float v = 0.0f;
        float a = 0.0f;
        float t = 0.0f;
        float T = 0.0f;
        uint8_t traj_type = 0;
        bool execute_traj = false;


        // INIT CTRL GAIN VECTORS
        struct vec Kp_p;    // Pos. Proportional Gains
        struct vec Kd_p;    // Pos. Derivative Gains
        struct vec Ki_p;    // Pos. Integral Gains  
        struct vec Kp_R;    // Rot. Proportional Gains
        struct vec Kd_R;    // Rot. Derivative Gains
        struct vec Ki_R;    // Rot. Integral Gains


        // CONTROLLER GAIN FLAGS
        float kp_xf = 1; // Pos. Gain Flag
        float kd_xf = 1; // Pos. Derivative Gain Flag
        // float ki_xf = 1; // Pos. Integral Flag
        // float kp_Rf = 1; // Rot. Gain Flag
        // float kd_Rf = 1; // Rot. Derivative Gain Flag
        // float ki_Rf = 1; // Rot. Integral Flag

        // STATE VALUES
        double _t = 0.0;
        struct vec statePos = {0.0f,0.0f,0.0f};         // Pos [m]
        struct vec stateVel = {0.0f,0.0f,0.0f};         // Vel [m/s]
        struct quat stateQuat = {0.0f,0.0f,0.0f,1.0f};  // Orientation
        struct vec stateOmega = {0.0f,0.0f,0.0f};       // Angular Rate [rad/s]

        struct mat33 R; // Orientation as rotation matrix
        struct vec stateEul = {0.0f,0.0f,0.0f}; // Pose in Euler Angles [YZX Notation]

        // OPTICAL FLOW STATES
        float RREV = 0.0f;  // [rad/s]
        float OF_x = 0.0f;  // [rad/s]
        float OF_y = 0.0f;  // [rad/s] 

        


        // DESIRED STATES
        struct vec x_d = {0.0f,0.0f,0.0f};  // Pos-desired [m]
        struct vec v_d = {0.0f,0.0f,0.0f};  // Vel-desired [m/s]
        struct vec a_d = {0.0f,0.0f,0.0f};  // Acc-desired [m/s^2]

        struct quat quat_d = {0.0f,0.0f,0.0f,1.0f}; // Orientation-desired [qx,qy,qz,qw]
        struct vec eul_d = {0.0f,0.0f,0.0f};        // Euler Angle-desired [rad? deg? TBD]
        struct vec omega_d = {0.0f,0.0f,0.0f};      // Omega-desired [rad/s]
        struct vec domega_d = {0.0f,0.0f,0.0f};     // Ang. Acc-desired [rad/s^2]

        struct vec b1_d = {1.0f,0.0f,0.0f};    // Desired body x-axis in global coord. [x,y,z]
        struct vec b2_d;    // Desired body y-axis in global coord.
        struct vec b3_d;    // Desired body z-axis in global coord.
        struct vec b3;      // Current body z-axis in global coord.

        struct mat33 R_d;   // Desired rotational matrix from b_d vectors
        struct vec e_3 = {0.0f, 0.0f, 1.0f}; // Global z-axis

        // STATE ERRORS
        struct vec e_x;     // Pos-error [m]
        struct vec e_v;     // Vel-error [m/s]
        struct vec e_PI;    // Pos. Integral-error [m*s]

        struct vec e_R;     // Rotation-error [rad]
        struct vec e_w;     // Omega-error [rad/s]
        struct vec e_RI;    // Rot. Integral-error [rad*s]

        struct vec F_thrust_ideal;           // Ideal thrust vector
        float F_thrust = 0.0f;               // Desired body thrust [N]
        float F_thrust_max = 0.64f;          // Max possible body thrust [N}]
        struct vec M;                        // Desired body moments [Nm]
        struct vec M_d = {0.0f,0.0f,0.0f};   // Desired moment [N*mm]
        float Moment_flag = false;


        // MOTOR THRUSTS
        float f_thrust; // Motor thrust - Thrust [N]
        float f_roll;   // Motor thrust - Roll   [N]
        float f_pitch;  // Motor thrust - Pitch  [N]
        float f_yaw;    // Motor thrust - Yaw    [N]


        float f_thrust_g = 0.0; // Motor thrust - Thrust [g]
        float f_roll_g = 0.0;   // Motor thrust - Roll   [g]
        float f_pitch_g = 0.0;  // Motor thrust - Pitch  [g]
        float f_yaw_g = 0.0;    // Motor thrust - Yaw    [g]



        // MOTOR VARIABLES
        uint32_t M1_pwm = 0; 
        uint32_t M2_pwm = 0; 
        uint32_t M3_pwm = 0; 
        uint32_t M4_pwm = 0; 

        // MOTOR SPEEDS
        float MS1 = 0.0;
        float MS2 = 0.0;
        float MS3 = 0.0;
        float MS4 = 0.0;

        float motorspeed[4]; // Motorspeed sent to plugin

        // TEMPORARY CALC VECS/MATRICES
        struct vec temp1_v; 
        struct vec temp2_v;
        struct vec temp3_v;
        struct vec temp4_v;
        struct mat33 temp1_m;  

        struct vec P_effort;
        struct vec R_effort;

        struct mat33 RdT_R; // Rd' * R
        struct mat33 RT_Rd; // R' * Rd
        struct vec Gyro_dyn;
        
                

        

        

        

        


        






};


void Controller::Load()
{
    cout << setprecision(3);
    cout << fixed;
    _isRunning = true;





    


    // START COMMUNICATION THREADS
    controllerThread = std::thread(&Controller::controllerGTC, this);


}


void Controller::vicon_stateCallback(const nav_msgs::Odometry::ConstPtr &msg){


    // Follow msg names from message details - "rqt -s rqt_msg" 
    
    // SET STATE VALUES INTO CLASS STATE VARIABLES
    _t = msg->header.stamp.toSec();
    _position = msg->pose.pose.position; 
    _velocity = msg->twist.twist.linear;
        
}


void Controller::OFCallback(const nav_msgs::Odometry::ConstPtr &msg){

    const geometry_msgs::Point position = msg->pose.pose.position; 
    const geometry_msgs::Vector3 velocity = msg->twist.twist.linear;

    
    double d = _H_CEILING-position.z; // h_ceiling - height

    // SET SENSOR VALUES INTO CLASS VARIABLES
    // _RREV = msg->RREV;
    // _OF_x = msg->OF_x;
    // _OF_y = msg->OF_y;

    _RREV = velocity.z/d;
    _OF_x = -velocity.y/d;
    _OF_y = -velocity.x/d;
}

void Controller::imuCallback(const sensor_msgs::Imu::ConstPtr &msg){
    _quaternion = msg->orientation;
    _omega = msg->angular_velocity;
    _accel = msg->linear_acceleration;
}

void Controller::RLData_Callback(const crazyflie_msgs::RLData::ConstPtr &msg){

    if (msg->reset_flag == true){

        controllerGTCReset();

    }
}

void Controller::ceilingFTCallback(const crazyflie_msgs::ImpactData::ConstPtr &msg)
{
    impact_flag = msg->impact_flag;
}

float Sigmoid(float x)
{
    return 1/(1+exp(-x));
}

float Controller::NN_Policy(nml_mat* X, nml_mat* W[], nml_mat* b[])
{
    
    //LAYER 1
    //Sigmoid(W*X+b)
    nml_mat *WX = nml_mat_dot(W[0],X); 
    nml_mat_add_r(WX,b[0]);
    nml_mat *a1 = nml_mat_funcElement(WX,Sigmoid);

    // LAYER 2
    // Sigmoid(W*X+b)
    nml_mat *WX2 = nml_mat_dot(W[1],a1); 
    nml_mat_add_r(WX2,b[1]);
    nml_mat *a2 = nml_mat_funcElement(WX2,Sigmoid);

    // LAYER 3
    // (W*X+b)
    nml_mat *WX3 = nml_mat_dot(W[2],a2); 
    nml_mat_add_r(WX3,b[2]);
    nml_mat *a3 = nml_mat_cp(WX3);

    float y_output = (float)a3->data[0][0];

    nml_mat_free(WX);
    nml_mat_free(a1);
    nml_mat_free(WX2);
    nml_mat_free(a2);
    nml_mat_free(WX3);
    nml_mat_free(a3);



    return y_output;

}

void Controller::adjustSimSpeed(float speed_mult)
{
    gazebo_msgs::SetPhysicsProperties srv;
    srv.request.time_step = 0.001;
    srv.request.max_update_rate = (int)(speed_mult/0.001);


    geometry_msgs::Vector3 gravity_vec;
    gravity_vec.x = 0.0;
    gravity_vec.y = 0.0;
    gravity_vec.z = -9.8066;
    srv.request.gravity = gravity_vec;

    gazebo_msgs::ODEPhysics ode_config;
    ode_config.auto_disable_bodies = false;
    ode_config.sor_pgs_precon_iters = 0;
    ode_config.sor_pgs_iters = 50;
    ode_config.sor_pgs_w = 1.3;
    ode_config.sor_pgs_rms_error_tol = 0.0;
    ode_config.contact_surface_layer = 0.001;
    ode_config.contact_max_correcting_vel = 0.0;
    ode_config.cfm = 0.0;
    ode_config.erp = 0.2;
    ode_config.max_contacts = 20;

    srv.request.ode_config = ode_config;

    SimSpeed_Client.call(srv);
}


// Converts thrust in Newtons to their respective PWM values
static inline int32_t thrust2PWM(float f) 
{
    // Conversion values calculated from self motor analysis
    float a = 3.31e4;
    float b = 1.12e1;
    float c = 8.72;
    float d = 3.26e4;

    float s = 1.0f; // sign of value
    int32_t f_pwm = 0;

    s = f/fabsf(f);
    f = fabsf(f);
    
    f_pwm = a*tanf((f-c)/b)+d;

    return s*f_pwm;

}      

// Converts thrust in PWM to their respective Newton values
static inline float PWM2thrust(int32_t M_PWM) 
{
    // Conversion values calculated from PWM to Thrust Curve
    // Linear Fit: Thrust [g] = a*PWM + b
    float a = 3.31e4;
    float b = 1.12e1;
    float c = 8.72;
    float d = 3.26e4;

    float f = b*atan2f(M_PWM-d,a)+c;
    // float f = (a*M_PWM + b); // Convert thrust to grams

    if(f<0)
    {
      f = 0;
    }

    return f;
}


// Limit PWM value to accurate portion of motor curve (0 - 60,000)
uint16_t limitPWM(int32_t value)
{
  if(value > PWM_MAX)
  {
    value = PWM_MAX;
  }
  else if(value < 0)
  {
    value = 0;
  }

  return (uint16_t)value;
}

static inline void printvec(struct vec v){

    std::cout << v.x << "\t" << v.y << "\t" << v.z;
    
}

