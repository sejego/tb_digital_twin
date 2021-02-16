//
// Created by sejego on 10/19/20.
// Last modified by sejego on 02/01/21.
//
//

#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <chrono>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <tb_digital_twin/Current.h>
#include <tb_digital_twin/Voltage.h>

#include "ParseDewetron.h"

class InputCurrentVoltage
{
private:
    std::vector<std::vector<float>> arrayOfProcessedData;
    ParseDewetron *dewetron;
    ros::Publisher PublishInputCurrent;
    ros::Publisher PublishInputVoltage;
    tb_digital_twin::Current inputCurrentValuesMsg;
    tb_digital_twin::Voltage inputVoltageValuesMsg;
    int rate = 1000; // 1kHz
    //float timeFromStart;
    //float timeIncrement;
    int arrIndex = 0;
    bool runForever;

public:

    InputCurrentVoltage(std::string filename, int numberOfCols, int numberOfVals, bool runForever)
    {
        ros::NodeHandle handler;
        runForever = runForever;
        // initializing publishers/subscribers
        PublishInputCurrent = handler.advertise<tb_digital_twin::Current>("tb/loading_motor/input_current", 10);
        PublishInputVoltage = handler.advertise<tb_digital_twin::Voltage>("tb/loading_motor/input_voltage", 10);
        dewetron = new ParseDewetron(filename, numberOfCols); // get from params
        processValues();
    }
    void processValues()
    {
        arrayOfProcessedData = dewetron->getOnlyValues();
    }

    void wrapToMsgArray(int index)
    {
        /* According to indexes in the file */
        inputCurrentValuesMsg.current1 = arrayOfProcessedData[index][5];
        inputCurrentValuesMsg.current2 = arrayOfProcessedData[index][4];
        inputCurrentValuesMsg.current3 = arrayOfProcessedData[index][3];
        inputVoltageValuesMsg.voltage1 = arrayOfProcessedData[index][0];
        inputVoltageValuesMsg.voltage2 = arrayOfProcessedData[index][1];
        inputVoltageValuesMsg.voltage3 = arrayOfProcessedData[index][2];
    }
    void spin() {

        ros::Rate r(rate);
        //ros::Time = ros::Time::now();
        ros::spinOnce();
        ROS_INFO("Initialized");
        // main control loop
        while (ros::ok())
        {
            spinOnce();
            ros::spinOnce();
            r.sleep();
        }
        ROS_INFO("Quit");
    }

    void spinOnce()
    {
        ros::spinOnce();
        /* If runForever is activated, the index will be reset to 0*/
        if(runForever)
        {
            if (arrIndex >= dewetron->getNumberOfRows())
            {
                arrIndex = 0;
            }
        }

        wrapToMsgArray(arrIndex);
        PublishInputCurrent.publish(inputCurrentValuesMsg);
        PublishInputVoltage.publish(inputVoltageValuesMsg);
        arrIndex += 1;
    }

};
int main(int argc, char **argv)
{
    std::string csv_file;
    float frequency;
    int cols, vals;
    bool runForever;

    ros::init(argc, argv, "loading_motor_1");
    ROS_INFO("Started iseauto inputCurrentVoltage node");
    ros::param::get("loading_motor_1/csv_file", csv_file);
    ros::param::get("loading_motor_1/frequency", frequency);
    ros::param::get("loading_motor_1/number_of_columns", cols);
    ros::param::get("loading_motor_1/number_of_values", vals);
    ros::param::get("loading_motor_1/run_forever",runForever);

    try
    {
        InputCurrentVoltage inputIV(csv_file, cols, vals, runForever);
        //std::cout << "Instantiated\n";

        inputIV.spin();
    }
    catch (const ros::Exception &e)
    {
        ROS_ERROR("Error occured %s", e.what());
        return (1);
    }

    return 0;
}