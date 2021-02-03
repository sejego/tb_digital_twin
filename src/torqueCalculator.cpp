//
// Created by sejego on 2/3/21.
//
#include <ros.h>
#include <iostream>
#include <tb_digital_twin/Power.h>
#include <std_msgs/Float32.h>

class TorqueCalculator
{
private:
    const int angularVelocity = 157; // rad/s or 1500 rpm
    float electricalPower;

    float convertToElectricalTorque()
    {
        return (electricalPower / angularVelocity);
    }


public:
    void powerListener(const tb_digital_twin::Power::ConstPtr &msg)
    {
        /**/
    }
    void efficiencyListener(const sstd_msgs::Float32ConstPtr &msg)
    {
        /**/
    }
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rotation");
    ROS_INFO("Started tb/loading_motor/rotation node");
    ros::NodeHandle nh("~");
    ros::Rate rate(60);

    TorqueCalculator torqueCalculator;
    ros::Subscriber powerReceiver = nh.subscribe<tb_digital_twin::Power>("tb/loading_motor/motor_power/electrical_power", 100,
                                                                       &TorqueCalculator::powerListener, &torqueCalculator);
    ros::Subscriber powerReceiver = nh.subscribe<std_msgs::Float32>("tb/loading_motor/motor_power/electrical_power", 100,
                                                                    &TorqueCalculator::efficiencyListener, &torqueCalculator);

    ros::Publisher rotationPublish = nh.advertise<std_msgs::Float32>("tb/loading_motor/torque", 100);

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