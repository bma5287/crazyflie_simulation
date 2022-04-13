/* 
This script subscribes to the ROS topics with the raw data from the crazyflie,
organizes it, and then republishes the data in an organized manner that 
is easy to use.
*/

// STANDARD INCLUDES
#include <stdio.h>
#include <iostream>
#include <boost/circular_buffer.hpp>
#include <math.h>       /* sqrt */
#include <thread>

// ROS INCLUDES
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/SetPhysicsProperties.h>

// CUSTOM INCLUDES
#include "crazyflie_msgs/CF_StateData.h"
#include "crazyflie_msgs/CF_FlipData.h"
#include "crazyflie_msgs/CF_ImpactData.h"
#include "crazyflie_msgs/CF_MiscData.h"

#include "crazyflie_msgs/CtrlData.h"
#include "crazyflie_msgs/CtrlDebug.h"
#include "crazyflie_msgs/RLCmd.h"
#include "crazyflie_msgs/RLData.h"
#include "crazyflie_msgs/PadConnect.h"

#include "crazyflie_msgs/activateSticky.h"

#define formatBool(b) ((b) ? "True" : "False")

class CF_DataConverter
{
    public:
        // CONSTRUCTOR TO START PUBLISHERS AND SUBSCRIBERS (Similar to Python's __init__())
        CF_DataConverter(ros::NodeHandle* nh)
        {

            // INITIALIZE SUBSCRIBERS
            CTRL_Data_Sub = nh->subscribe("/CTRL/data", 1, &CF_DataConverter::CtrlData_Callback, this, ros::TransportHints().tcpNoDelay());
            CTRL_Debug_Sub = nh->subscribe("/CTRL/debug", 1, &CF_DataConverter::CtrlDebug_Callback, this, ros::TransportHints().tcpNoDelay());
            RL_CMD_Sub = nh->subscribe("/RL/cmd",5,&CF_DataConverter::RL_CMD_Callback,this,ros::TransportHints().tcpNoDelay());
            RL_Data_Sub = nh->subscribe("/RL/data",5,&CF_DataConverter::RL_Data_Callback,this,ros::TransportHints().tcpNoDelay());
            Surface_FT_Sub = nh->subscribe("/ENV/Surface_FT_sensor",5,&CF_DataConverter::SurfaceFT_Sensor_Callback,this,ros::TransportHints().tcpNoDelay());
            Surface_Contact_Sub = nh->subscribe("/ENV/BodyContact",5,&CF_DataConverter::Surface_Contact_Callback,this,ros::TransportHints().tcpNoDelay());
            PadConnect_Sub = nh->subscribe("/ENV/Pad_Connections",5,&CF_DataConverter::Pad_Connections_Callback,this,ros::TransportHints().tcpNoDelay());

            // INITIALIZE MAIN PUBLISHERS
            StateData_Pub = nh->advertise<crazyflie_msgs::CF_StateData>("/CF_DC/StateData",1);
            MiscData_Pub =  nh->advertise<crazyflie_msgs::CF_MiscData>("/CF_DC/MiscData",1);
            FlipData_Pub =  nh->advertise<crazyflie_msgs::CF_FlipData>("/CF_DC/FlipData",1);
            ImpactData_Pub = nh->advertise<crazyflie_msgs::CF_ImpactData>("/CF_DC/ImpactData",1);   

            // GAZEBO SERVICES
            GZ_SimSpeed_Client = nh->serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");

            
            fPtr = fopen(filePath, "w");

            CF_DataConverter::LoadParams();
            Time_start = ros::Time::now();

            CF_DataConverter::adjustSimSpeed(SIM_SPEED);
            BodyCollision_str = MODEL_NAME + "::crazyflie_BaseModel::crazyflie_body::body_collision";

            CF_DCThread = std::thread(&CF_DataConverter::MainLoop, this);


        }

        void create_CSV()
        {
            fprintf(fPtr,"k_ep,k_run,");
            fprintf(fPtr,"t,");
            fprintf(fPtr,"NN_flip,NN_policy,");
            fprintf(fPtr,"mu,sigma,policy,");

            // INTERNAL STATE ESTIMATES (CF)
            fprintf(fPtr,"x,y,z,");
            fprintf(fPtr,"vx,vy,vz,");
            fprintf(fPtr,"qx,qy,qz,qw,");
            fprintf(fPtr,"wx,wy,wz,");
            fprintf(fPtr,"eul_x,eul_y,eul_z,");

            // MISC RL LABELS
            fprintf(fPtr,"flip_flag,impact_flag,");

            //  MISC INTERNAL STATE ESTIMATES
            fprintf(fPtr,"Tau,OF_x,OF_y,RREV,d_ceil,");
            fprintf(fPtr,"F_thrust,Mx,My,Mz,");
            fprintf(fPtr,"M1_thrust,M2_thrust,M3_thrust,M4_thrust,");
            fprintf(fPtr,"M1_pwm,M2_pwm,M3_pwm,M4_pwm,");


            // SETPOINT VALUES
            fprintf(fPtr,"x_d.x,x_d.y,x_d.z,");
            fprintf(fPtr,"v_d.x,v_d.y,v_d.z,");
            fprintf(fPtr,"a_d.x,a_d.y,a_d.z,");

            // MISC VALUES
            fprintf(fPtr,"Volts,Error,");

            fflush(fPtr);

        }

        void append_CSV()
        {
            fprintf(fPtr,"--,--,");
            fprintf(fPtr,"%.3f,",Time.toSec());
            fprintf(fPtr,"%.3f,%.3f,",NN_flip,NN_policy);
            fprintf(fPtr,"--,--,--,");

            // // INTERNAL STATE ESTIMATES (CF)
            fprintf(fPtr,"%.3f,%.3f,%.3f,",Pose.position.x,Pose.position.y,Pose.position.z);
            fprintf(fPtr,"%.3f,%.3f,%.3f,",Twist.linear.x,Twist.linear.y,Twist.linear.z);
            fprintf(fPtr,"%.3f,%.3f,%.3f,%.3f,",Pose.orientation.x,Pose.orientation.y,Pose.orientation.z,Pose.orientation.w);
            fprintf(fPtr,"%.3f,%.3f,%.3f,",Twist.angular.x,Twist.angular.y,Twist.angular.z);
            fprintf(fPtr,"%.3f,%.3f,%.3f,",Eul.x,Eul.y,Eul.z);


            // MISC RL LABELS
            fprintf(fPtr,"%s,%s,",formatBool(flip_flag),formatBool(impact_flag));

            // MISC INTERNAL STATE ESTIMATES
            fprintf(fPtr,"%.3f,%.3f,%.3f,%.3f,%.3f,",Tau,OFx,OFy,RREV,D_ceil);
            fprintf(fPtr,"%.3f,%.3f,%.3f,%.3f,",FM[0],FM[1],FM[2],FM[3]);
            fprintf(fPtr,"%.3f,%.3f,%.3f,%.3f,",MotorThrusts[0],MotorThrusts[1],MotorThrusts[2],MotorThrusts[3]);
            fprintf(fPtr,"%u,%u,%u,%u,",MS_PWM[0],MS_PWM[1],MS_PWM[2],MS_PWM[3]);


            // SETPOINT VALUES
            fprintf(fPtr,"%.3f,%.3f,%.3f,",x_d.x,x_d.y,x_d.z);
            fprintf(fPtr,"%.3f,%.3f,%.3f,",v_d.x,v_d.y,v_d.z);
            fprintf(fPtr,"%.3f,%.3f,%.3f,",a_d.x,a_d.y,a_d.z);


            // MISC VALUES
            fprintf(fPtr,"%.3f,%s",V_battery,"--");
            fprintf(fPtr,"\n");
            fflush(fPtr);

        }


        // FUNCTION PRIMITIVES
        void CtrlData_Callback(const crazyflie_msgs::CtrlData &ctrl_msg);
        void CtrlDebug_Callback(const crazyflie_msgs::CtrlDebug &ctrl_msg);

        void RL_CMD_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg);
        void RL_Data_Callback(const crazyflie_msgs::RLData::ConstPtr &msg);
        void SurfaceFT_Sensor_Callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
        void Surface_Contact_Callback(const gazebo_msgs::ContactsState &msg);
        void Pad_Connections_Callback(const crazyflie_msgs::PadConnect &msg);

        void Publish_StateData();
        void Publish_FlipData();
        void Publish_ImpactData();
        void Publish_MiscData();

        void MainLoop();
        void activateStickyFeet();
        void LoadParams();
        void consoleOuput();
        void checkSlowdown();
        void adjustSimSpeed(float speed_mult);
        void decompressXY(uint32_t xy, float xy_arr[]);
        void quat2euler(float quat[], float eul[]);

    private:

        // SUBSCRIBERS
        ros::Subscriber CTRL_Data_Sub;
        ros::Subscriber CTRL_Debug_Sub;
        ros::Subscriber RL_CMD_Sub;
        ros::Subscriber RL_Data_Sub;
        ros::Subscriber Surface_FT_Sub;
        ros::Subscriber Surface_Contact_Sub;
        ros::Subscriber PadConnect_Sub;

        // PUBLISHERS
        ros::Publisher StateData_Pub;
        ros::Publisher FlipData_Pub;
        ros::Publisher ImpactData_Pub;
        ros::Publisher MiscData_Pub;

        // SERVICES
        ros::ServiceClient GZ_SimSpeed_Client;

        // MESSAGES
        crazyflie_msgs::CF_StateData StateData_msg;
        crazyflie_msgs::CF_FlipData FlipData_msg;
        crazyflie_msgs::CF_ImpactData ImpactData_msg;
        crazyflie_msgs::CF_MiscData MiscData_msg;

        std::thread CF_DCThread;
        std::string BodyCollision_str;
        uint32_t tick = 1;
        ros::Time Time_start;

        FILE* fPtr;
        char filePath[100] = "/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_logging/local_logs/Example.csv";
    // fprintf(fPtr, "This is testing for fprintf...%d\n",555);
    // fputs("This is testing for fputs...\n", fPtr);
        
        // ===================
        //     ROS PARAMS
        // ===================

        std::string MODEL_NAME;

        // DEFAULT INERTIA VALUES FOR BASE CRAZYFLIE
        float CF_MASS = 34.4e3; // [kg]
        float Ixx = 15.83e-6f;  // [kg*m^2]
        float Iyy = 17.00e-6f;  // [kg*m^2]
        float Izz = 31.19e-6f;  // [kg*m^2]

        int SLOWDOWN_TYPE = 0;
        bool LANDING_SLOWDOWN_FLAG;
        float SIM_SPEED; 
        float SIM_SLOWDOWN_SPEED;
        int POLICY_TYPE = 0;
        
        float P_kp_xy,P_kd_xy,P_ki_xy;
        float P_kp_z,P_kd_z,P_ki_z;
        float R_kp_xy,R_kd_xy,R_ki_xy;     
        float R_kp_z,R_kd_z,R_ki_z;



        // ===================
        //     FLIGHT DATA
        // ===================

        ros::Time Time;

        geometry_msgs::Pose Pose;
        geometry_msgs::Twist Twist;
        geometry_msgs::Vector3 Eul;

        double Tau;
        double OFx;
        double OFy;
        double RREV;
        double D_ceil;

        boost::array<double, 4> FM;
        boost::array<double, 4> MotorThrusts;
        boost::array<uint16_t, 4> MS_PWM;

        double Tau_thr;
        double G1;

        double NN_flip;
        double NN_policy;

        geometry_msgs::Vector3 x_d;
        geometry_msgs::Vector3 v_d;
        geometry_msgs::Vector3 a_d;

        // ==================
        //     FLIP DATA
        // ==================

        bool flip_flag = false;
        bool OnceFlag_flip = false;

        ros::Time Time_tr;

        geometry_msgs::Pose Pose_tr;
        geometry_msgs::Twist Twist_tr;
        geometry_msgs::Vector3 Eul_tr;


        double Tau_tr;
        double OFx_tr;
        double OFy_tr;
        double RREV_tr;
        double D_ceil_tr;

        boost::array<double, 4> FM_tr;

        double NN_tr_flip;
        double NN_tr_policy;


        // ===================
        //     IMPACT DATA
        // ===================

        bool impact_flag = false;
        bool BodyContact_flag = false;
        bool OnceFlag_impact = false;
        double impact_thr = 0.1; // Impact threshold [N]

        ros::Time Time_impact;
        geometry_msgs::Vector3 Force_impact;
        geometry_msgs::Pose Pose_impact;
        geometry_msgs::Twist Twist_impact;
        geometry_msgs::Vector3 Eul_impact;


        double impact_force_x = 0.0; // Max impact force in X-direction [N]
        double impact_force_y = 0.0; // Max impact force in Y-direction [N]
        double impact_force_z = 0.0; // Max impact force in Z-direction [N]
        double impact_force_resultant = 0.0; // Current impact force magnitude

        // CIRCULAR BUFFERES TO LAG IMPACT STATE DATA (WE WANT STATE DATA THE INSTANT BEFORE IMPACT)
        boost::circular_buffer<geometry_msgs::Pose> Pose_impact_buff {5};
        boost::circular_buffer<geometry_msgs::Twist> Twist_impact_buff {5};

        // ==================
        //     MISC DATA
        // ==================

        double V_battery = 4.0;

        uint8_t Pad1_Contact = 0;
        uint8_t Pad2_Contact = 0;
        uint8_t Pad3_Contact = 0;
        uint8_t Pad4_Contact = 0;

        uint8_t Pad_Connections = 0;

        // ====================
        //     DEBUG VALUES
        // ====================

        bool Motorstop_Flag = false;
        bool Pos_Ctrl_Flag = false;
        bool Vel_Ctrl_Flag = false;
        bool Traj_Active_Flag = false;
        bool Tumble_Detection = false;
        bool Tumbled_Flag = false;
        bool Moment_Flag = false;
        bool Policy_Armed_Flag = false;
        bool Sticky_Flag = false;


        


};

void CF_DataConverter::LoadParams()
{

    // COLLECT MODEL PARAMETERS
    ros::param::get("/MODEL_NAME",MODEL_NAME);
    ros::param::get("/CF_MASS",CF_MASS);
    ros::param::get("/Ixx",Ixx);
    ros::param::get("/Iyy",Iyy);
    ros::param::get("/Izz",Izz);
    ros::param::get("/POLICY_TYPE",POLICY_TYPE);

    // DEBUG SETTINGS
    ros::param::get("/SIM_SPEED",SIM_SPEED);
    ros::param::get("/SIM_SLOWDOWN_SPEED",SIM_SLOWDOWN_SPEED);
    ros::param::get("/LANDING_SLOWDOWN_FLAG",LANDING_SLOWDOWN_FLAG);

    ros::param::get("P_kp_xy",P_kp_xy);
    ros::param::get("P_kd_xy",P_kd_xy);
    ros::param::get("P_ki_xy",P_ki_xy);

    ros::param::get("P_kp_z",P_kp_z);
    ros::param::get("P_kd_z",P_kd_z);
    ros::param::get("P_ki_z",P_ki_z);

    ros::param::get("R_kp_xy",R_kp_xy);
    ros::param::get("R_kd_xy",R_kd_xy);
    ros::param::get("R_ki_xy",R_ki_xy);
    
    ros::param::get("R_kp_z",R_kp_z);
    ros::param::get("R_kd_z",R_kd_z);
    ros::param::get("R_ki_z",R_ki_z);

}






