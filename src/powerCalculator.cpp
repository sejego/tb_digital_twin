#include <ros/ros.h>
#include <stdlib.h>
#include <std_msgs/Float32.h>
#include <tb_digital_twin/Current.h>
#include <tb_digital_twin/Voltage.h>
#include <tb_digital_twin/Power.h>

class Power
{
private:
    float inputCurrent[3];
    float inputVoltage[3];
    float mean[2][3], square[2][3], k[2][3];
    std::vector<std::vector<float>> currents;
    std::vector<std::vector<float>> voltages;
    std::vector<std::vector<float>> currentBuffer;
    std::vector<std::vector<float>> voltageBuffer;
    const float cosPhi = 0.84;
    const int SIZE_A = 5000;
    int c, v = 0;

    void calculateRMS(std::vector<std::vector<float>> currentBuff, std::vector<std::vector<float>> voltageBuff)
    {
        for (int j = 0; j < SIZE_A; j++)
        {
            /*Currents*/
            square[0][0] += currentBuff[0][j] * currentBuff[0][j];
            square[0][1] += currentBuff[1][j] * currentBuff[1][j];
            square[0][2] += currentBuff[2][j] * currentBuff[2][j];

            /*Voltages*/
            square[1][0] += voltageBuff[0][j] * voltageBuff[0][j];
            square[1][1] += voltageBuff[1][j] * voltageBuff[1][j];
            square[1][2] += voltageBuff[2][j] * voltageBuff[2][j];
        }
        for (int j = 0; j < 3; j++)
        {
            mean[0][j] = (square[0][j] / SIZE_A);
            mean[1][j] = (square[1][j] / SIZE_A);
        }
        for (int j = 0; j < 3; j++)
        {
            rmsCurrents[j] = std::sqrt(mean[0][j]);
            rmsVoltages[j] = std::sqrt(mean[1][j]);
        }
    }

public:
    bool canCalculate = false;
    bool* const pCanCalculate = &canCalculate;
    float rmsCurrents[3] = {0.0};
    float rmsVoltages[3] = {0.0};
    float phasePowerElectrical[3];
    float totalPowerElectrical;
    float phasePowerReactive[3];
    float totalPowerReactive;

    Power()
    {
        currents.resize(3);
        voltages.resize(3);
    }

    void currentCallback(const tb_digital_twin::Current::ConstPtr& msg)
    {
        inputCurrent[0] = msg->current1;
        inputCurrent[1] = msg->current2;
        inputCurrent[2] = msg->current3;

    /* Reset if 5k is in the buffer */
        if(c >= SIZE_A )
        {
            //std::cout << "Listened\n";
            c = 0;
            currentBuffer = currents;
            canCalculate = true;
            currents.clear();
            currents.resize(3);

        }

        currents.at(0).push_back(msg->current1);
        //std::cout<< currents[0][i] << "\n\n";
        currents.at(1).push_back(msg->current2);
        currents.at(2).push_back(msg->current3);

        c++;
    }
    void voltageCallback(const tb_digital_twin::Voltage::ConstPtr& msg)
    {
        inputVoltage[0] = msg->voltage1;
        inputVoltage[1] = msg->voltage2;
        inputVoltage[2] = msg->voltage3;
        if(v >= SIZE_A )
        {
            //std::cout << "Listened\n";
            v = 0;
            voltageBuffer = voltages;
            canCalculate = true;
            voltages.clear();
            voltages.resize(3);
        }

        voltages.at(0).push_back(msg->voltage1);
        voltages.at(1).push_back(msg->voltage2);
        voltages.at(2).push_back(msg->voltage3);

        v++;
    }
    void calculatePowerReactive()
    {
        for(int i=0;i<3;i++)
        {
            phasePowerReactive[i] = (abs(inputCurrent[i]*inputVoltage[i]))/3;
        }
        totalPowerReactive = phasePowerReactive[0]+phasePowerReactive[1]+phasePowerReactive[2];
        //return totalPower;
    }
    void calculatePowerElectrical()
    {
        calculateRMS(currentBuffer, voltageBuffer);
        phasePowerElectrical[0] = rmsCurrents[0]*rmsVoltages[0]*cosPhi;
        phasePowerElectrical[1] = rmsCurrents[1]*rmsVoltages[1]*cosPhi;
        phasePowerElectrical[2] = rmsCurrents[2]*rmsVoltages[2]*cosPhi;
        totalPowerElectrical = phasePowerElectrical[0]+phasePowerElectrical[1]+phasePowerElectrical[2];
    }

};

int main(int argc, char *argv[])
{
    Power power;
    ros::init(argc, argv, "tb_loading_motor_power");
    ros::NodeHandle handler;
    ros::Publisher PowerReactivePublisher = handler.advertise<tb_digital_twin::Power>("tb/loading_motor/motor_power_reactive", 100);
    ros::Publisher PowerElectricalPublisher = handler.advertise<tb_digital_twin::Power>("tb/loading_motor/motor_power_electrical", 100);
    ros::Subscriber voltageSubscriber = handler.subscribe<tb_digital_twin::Voltage>("tb/loading_motor/input_voltage", 100, &Power::voltageCallback, &power);
    ros::Subscriber currentSubscriber = handler.subscribe<tb_digital_twin::Current>("tb/loading_motor/input_current", 100, &Power::currentCallback, &power);
    ros::Rate rate(60);

    tb_digital_twin::Power powerReactMsg;
    tb_digital_twin::Power powerElMsg;

    while(ros::ok())
    {
        ros::spinOnce();
        power.calculatePowerReactive();
        if(power.canCalculate)
        {
            power.calculatePowerElectrical();
            powerElMsg.phase1 = power.phasePowerElectrical[0];
            powerElMsg.phase2 = power.phasePowerElectrical[1];
            powerElMsg.phase3 = power.phasePowerElectrical[2];
            powerElMsg.total = power.totalPowerElectrical;
            PowerElectricalPublisher.publish(powerElMsg);
        }
        powerReactMsg.phase1 = power.phasePowerReactive[0];
        powerReactMsg.phase2 = power.phasePowerReactive[1];
        powerReactMsg.phase3 = power.phasePowerReactive[2];
        powerReactMsg.total = power.totalPowerReactive;
        PowerReactivePublisher.publish(powerReactMsg);
        rate.sleep();
    }
    return 0;
}
