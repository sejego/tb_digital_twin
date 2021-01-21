/**
 * twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack
 *
 * This code is based on a similar node from the differential_drive package
 * http://wiki.ros.org/differential_drive
 * https://github.com/jfstepha/differential-drive
 *
 */

#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <iostream>
#include <fstream>
#include <ctime>
#include <thread>
#include "pid.h" // include PID class

#define testingrn

int ticksSinceTarget;
float encoderValue;
float desiredValue;
int rate;
int timeoutTicks;
float pid_consts[3];
int sgn(double v) {
    if (v < 0) return -1;
    if (v > 0) return 1;
    return 0;
}
void velocityReferenceReset(float &vel, float p_vel)
{
    if(sgn(vel)+sgn(p_vel)==0)
        vel=0;
    else vel=p_vel;
}

void encoderReceive(std_msgs::Float32ConstPtr msg)
{

    ticksSinceTarget = 0;
    encoderValue=msg->data;

}
void desiredReceive(std_msgs::Float32ConstPtr msg)
{

    ticksSinceTarget = 0;
    desiredValue=msg->data;

}

class MotorController {
public:
    //float max_torque = 60;
    MotorController(float cp, float ci, float cd, float max_torque, float min_value) {

        motorPID=new PID(0.018f,0.012,0.0000f,max_torque,0);
        motorPID->max_i=500;
        rate=60;
        timeoutTicks=2;
        ros::NodeHandle pnode("~");
        pnode.getParam("twist_to_motor_rate", rate);
        pnode.getParam("twist_to_motor_timeout_ticks", timeoutTicks);

        // initializing publishers/subscribers
        encoderReceiver = handler.subscribe<std_msgs::Float32>("tb/loading_motor/actual_rpm", 10, encoderReceive);
        desiredReceiver = handler.subscribe<std_msgs::Float32>("tb/loading_motor/desired_rpm", 10, desiredReceive);
        controlPublisher = handler.advertise<std_msgs::Float32>("tb/loading_motor/torque", 10);
    }
    void spin() {
        ros::Rate r(rate);
        ros::Rate idle(10);
        ros::Time then = ros::Time::now();
        ros::spinOnce();
        ROS_INFO("Initialized");
        // main control loop
        while (ros::ok())
        {
            spinOnce();
            ros::spinOnce();
            idle.sleep();
        }

        ROS_INFO("Quit");
    }
private:

    PID* motorPID;
    ros::NodeHandle handler;
    ros::Subscriber encoderReceiver;
    ros::Subscriber desiredReceiver;
    ros::Publisher controlPublisher;

    void spinOnce()
    {
        ros::spinOnce();

        motorPID->loop(&desiredValue,&encoderValue);
        std_msgs::Float32 value;
        value.data = motorPID->getOutput();
        controlPublisher.publish(value);
        ticksSinceTarget += 1;

        ROS_INFO("Looping");
    }
};
//TODO: cosine phi!!!!
int main(int argc, char **argv) {
    float cp=0, ci=0, cd=0,maxTorque=0,minValue=0;
    ros::init(argc, argv, "tb/loading_motor/controller");
    ROS_INFO("Started /tb/loading_motor/controller node");
    ros::param::get("tb/loading_motor/controller/P", cp);
    ros::param::get("/tb/loading_motor/controller/I", ci);
    ros::param::get("/tb/loading_motor/controller/D", cd);
    ros::param::get("/tb/loading_motor/controller/max_torque",maxTorque);
    ros::param::get("/tb/loading_motor/controller/min_value",minValue);

    try {
        MotorController baseController(cp,ci,cd,maxTorque,minValue);
        baseController.spin();
    }
    catch (const ros::Exception) {
        return (1);
    }
    return 0;
}

