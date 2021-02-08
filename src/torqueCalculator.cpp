//
// Created by sejego on 2/3/21.
//
#include <ros/ros.h>
#include <iostream>
#include <tb_digital_twin/Power.h>
#include <std_msgs/Float32.h>

class TorqueCalculator
{
private:
    //const int angularVelocity = 157; // rad/s or 1500 rpm
    bool velocityUpdated = false;
    bool powerUpdated = false;
    bool efficiencyUpdated = false;

    float electricalPower, efficiency, angularVelocity;
    float mechanicalTorque, electricalTorque = 0;

    float calculateElectricalTorque()
    {
        return (electricalPower / angularVelocity);
    }

    float calculateMechanicalTorque()
    {
        return (electricalTorque * efficiency);
    }


public:
    float getMechanicalTorque()
    {
        return mechanicalTorque;
    }

    float getElectricalTorque()
    {
        return electricalTorque;
    }

    void powerListener(const tb_digital_twin::Power::ConstPtr &msg)
    {
        electricalPower = msg->total;
        powerUpdated = true;
        if(powerUpdated && velocityUpdated)
        {
            electricalTorque = calculateElectricalTorque();
            powerUpdated = false;
            velocityUpdated = false;
            powerUpdated = true;
        }
    }
    void efficiencyListener(const std_msgs::Float32ConstPtr &msg)
    {
        efficiency = msg->data;
        efficiencyUpdated = true;
        /*
        if(powerUpdated && efficiencyUpdated)
        {
            mechanicalTorque = calculateMechanicalTorque();
            powerUpdated = false;
            efficiencyUpdated = false;
        }
         */
    }
    void angularVelocityListener(const std_msgs::Float32ConstPtr &msg)
    {
        angularVelocity = msg->data;
        velocityUpdated = true;
        if(powerUpdated && velocityUpdated)
        {
            electricalTorque = calculateElectricalTorque();
            powerUpdated = false;
            velocityUpdated = false;
            powerUpdated = true;
        }
    }
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tb_loading_motor_torque");
    ROS_INFO("Started tb/loading_motor/torque node");
    ros::NodeHandle nh("~");
    ros::Rate rate(60);

    TorqueCalculator torqueCalculator;
    /* Subscribers */
    ros::Subscriber powerReceiver = nh.subscribe<tb_digital_twin::Power>("/tb/loading_motor/motor_power/electrical_power", 100,
                                                                       &TorqueCalculator::powerListener, &torqueCalculator);
    ros::Subscriber efficiencyReceiver = nh.subscribe<std_msgs::Float32>("/tb/loading_motor/efficiency", 100,
                                                                    &TorqueCalculator::efficiencyListener, &torqueCalculator);
    ros::Subscriber angularVelocityReceiver = nh.subscribe<std_msgs::Float32>("/tb/loading_motor/actual_rpm", 100,
                                                                              &TorqueCalculator::angularVelocityListener, &torqueCalculator);

    /* Publishers */
    ros::Publisher electricalTorquePublisher = nh.advertise<std_msgs::Float32>("/tb/loading_motor/electrical_torque", 10);
    ros::Publisher mechanicalTorquePublisher = nh.advertise<std_msgs::Float32>("/tb/loading_motor/mechanical_torque", 10);


    std_msgs::Float32 elec_torque_msg;
    std_msgs::Float32 mech_torque_msg;
    elec_torque_msg.data = 0;
    mech_torque_msg.data = 0;

    while(ros::ok())
    {
        ros::spinOnce();
        ROS_DEBUG("I happen for sure");
        elec_torque_msg.data = torqueCalculator.getElectricalTorque();
        mech_torque_msg.data = torqueCalculator.getMechanicalTorque();
        electricalTorquePublisher.publish(elec_torque_msg);
        mechanicalTorquePublisher.publish(mech_torque_msg);
        rate.sleep();
    }


    return 0;
}