#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>


#include <ros/ros.h>
#include <thread>
#include "sar_msgs/Cam_Settings.h"


namespace gazebo {

    class Cam_Update_Plugin: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            void Camera_init();
            void Update_Camera();
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
            physics::LinkPtr Camera_Link_Ptr;
            physics::LinkPtr SAR_Body_Ptr;
            physics::JointPtr Joint_Ptr;

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
