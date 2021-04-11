
/*
*  Name: gazebo_transport_to_ros_topic.cpp
*  Author: Joseph Coombe
*  Date: 11/22/2017
*  Edited: 11/27/2017
*  Description:
*   Subscribe to a Gazebo transport topic and publish to a ROS topic
*/

// Gazebo dependencies
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>

// ROS dependencies
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include "crazyflie_rl/RLCmd.h"

#include <iostream>

ros::Publisher pub;
ros::Subscriber RLCmd_Subscriber;

double ros_rate=100;
bool home = false;

double ceiling_ft_z = 0.0;
double ceiling_ft_x = 0.0;


void torquesCb(const ConstWrenchStampedPtr &_msg)
{

  if (home){
    ceiling_ft_z = 0.0;
    ceiling_ft_x = 0.0;
    home = false;
  }
  // Record max force experienced
  if (_msg->wrench().force().z() > ceiling_ft_z){
    ceiling_ft_z = _msg->wrench().force().z();
  }

  if (_msg->wrench().force().x() > ceiling_ft_x){
    ceiling_ft_x = _msg->wrench().force().x();
  }


  // std::cout << "Received msg: " << std::endl;
  // std::cout << _msg->DebugString() << std::endl;
  geometry_msgs::WrenchStamped msgWrenchedStamped;
  // try WrenchStamped msgWrenchedStamped;
  msgWrenchedStamped.header.stamp = ros::Time::now();
  msgWrenchedStamped.wrench.force.x = ceiling_ft_x;
  msgWrenchedStamped.wrench.force.y = _msg->wrench().force().y();
  msgWrenchedStamped.wrench.force.z = ceiling_ft_z;
  msgWrenchedStamped.wrench.torque.x = _msg->wrench().torque().x();
  msgWrenchedStamped.wrench.torque.y = _msg->wrench().torque().y();
  msgWrenchedStamped.wrench.torque.z = _msg->wrench().torque().z();
  pub.publish(msgWrenchedStamped);
 
}

void RLCmd_Callback(const crazyflie_rl::RLCmd::ConstPtr &msg)
{

  if(msg->cmd_type == 0)
  {
    home = true;
  }

}

int main(int argc, char **argv)
{
    
  ROS_INFO("Starting gazebo_transport_to_ros_topic node");

  // Load Gazebo
  gazebo::client::setup(argc, argv);
  ROS_INFO("Starting gazebo");

  // Load ROS
  ros::init(argc, argv, "gazebo_transport_to_ros_topic");
  
  // Create Gazebo node and init
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Create ROS node and init
  ros::NodeHandle nh;

  // Get ROS params
  std::string gazebo_transport_topic_to_sub= "/gazebo/default/ceiling_plane/joint_01/force_torque/wrench";
  std::string ros_topic_to_pub="/ceiling_force_sensor";
  
  //double ros_rate;
  //ros::param::get("~gazebo_transport_topic_to_sub", gazebo_transport_topic_to_sub);
  //ros::param::get("~ros_rate", ros_rate);

  pub = nh.advertise<geometry_msgs::WrenchStamped>(ros_topic_to_pub, 10);
  RLCmd_Subscriber = nh.subscribe("/rl_ctrl",50,RLCmd_Callback);
  
  ROS_INFO("Starting Publisher");
  

  // Listen to Gazebo force_torque sensor topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe(gazebo_transport_topic_to_sub , torquesCb);
  ros::Rate loop_rate(ros_rate); 
  ROS_INFO("Starting Subscriber ");
  
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    
  }
  gazebo::shutdown();
  return 0;
}