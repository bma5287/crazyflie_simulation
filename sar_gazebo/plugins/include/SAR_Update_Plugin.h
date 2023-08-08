#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>


#include <ros/ros.h>
#include <thread>
#include "sar_msgs/Cam_Settings.h"

#define Deg2Rad M_PI/180
#define Y_AXIS 0    // First index on leg joint
#define Z_AXIS 1    // Second index on leg joint


namespace gazebo {

    class SAR_Update_Plugin: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            void Update_Camera();
            void Update_Hinge_Params();
            bool Service_Callback(sar_msgs::Cam_Settings::Request &req, sar_msgs::Cam_Settings::Response &res);


        private:

            // SDF PARAMS
            std::string Joint_Name;

            // CONFIG PARAMS
            std::string SAR_Type;
            std::string SAR_Config;
            std::string Cam_Config;

            // GAZEBO POINTERS
            physics::ModelPtr Base_Model_Ptr;
            physics::ModelPtr Config_Model_Ptr;
            physics::LinkPtr Camera_Link_Ptr;
            physics::LinkPtr SAR_Body_Ptr;
            physics::JointPtr Joint_Ptr;
            physics::InertialPtr Inertial_Ptr;
            physics::WorldPtr World_Ptr;

            double K_Pitch;
            double DR_Pitch;
            double C_Pitch;

            double K_Yaw;
            double DR_Yaw;
            double C_Yaw;

            double Leg_Angle;

            double Iyy_Leg;
            double Ixx_Leg;
            double Izz_Leg;

            physics::LinkPtr Leg_1_LinkPtr;
            physics::LinkPtr Leg_2_LinkPtr;
            physics::LinkPtr Leg_3_LinkPtr;
            physics::LinkPtr Leg_4_LinkPtr;

            physics::JointPtr Hinge_1_JointPtr;
            physics::JointPtr Hinge_2_JointPtr;
            physics::JointPtr Hinge_3_JointPtr;
            physics::JointPtr Hinge_4_JointPtr;



            // SENSOR POINTERS
            sensors::SensorPtr Sensor_Ptr;
            sensors::CameraSensor* Camera_Ptr;

            // CAMERA CONFIG SETTINGS
            double X_Offset;    // [m]
            double Y_Offset;    // [m]
            double Z_Offset;    // [m]
            double Pitch_Angle; // [deg]
            double FPS;         // [1/s]

            // POSE UPDATES
            ros::NodeHandle nh;
            ros::ServiceServer Cam_Update_Service;
    };

}
