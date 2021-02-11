#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <iostream>

#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>

class angularConverter
{
public:
    float rpm;
    void VelocityReceiver(const std_msgs::Float32ConstPtr &msg);
};

void angularConverter::VelocityReceiver(const std_msgs::Float32ConstPtr &msg)
{
    rpm = msg->data*60/(2* M_PI);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rotation");
    ROS_INFO("Started tb/loading_motor/rotation node");
    ros::NodeHandle nh("~");
    ros::Rate rate(60);

    angularConverter listener;
    ros::Subscriber feedbackReceiver = nh.subscribe<std_msgs::Float32>("/tb/loading_motor/shaft_angular_velocity", 10,
                                                                       &angularConverter::VelocityReceiver, &listener);

    ros::Publisher rotationPublish = nh.advertise<std_msgs::Float32>("/tb/loading_motor/actual_rpm", 10);

    std_msgs::Float32 rpm_msg;
    rpm_msg.data = 0;

  while(ros::ok())
  {
      ros::spinOnce();
      rpm_msg.data = listener.rpm;
      rotationPublish.publish(rpm_msg);
      rate.sleep();

  }
  return 0;
}
