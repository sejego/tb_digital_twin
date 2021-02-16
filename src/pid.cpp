#include <stdio.h>
#include <math.h>
#include "pid.h"

int sgn(float v) {
    if (v < 0) return -1;
    if (v > 0) return 1;
    return 0;
}

PID::PID(float cp, float ci, float cd, float max_out, float min_value)
{
    deadband[0]=max_out;
    deadband[1]=min_value;
    p=cp;
    i=ci;
    d=cd;
}

void PID::update(float cp, float ci, float cd)
{
    p=cp;
    i=ci;
    d=cd;
}

void PID::loop(float *desired, float *real)
{
    pError = *desired - *real;
    iError += pError;
    dError = (pError - old_pError);
    checkInput(desired);
    checkOverflow(iError,max_i);
    output=p * pError + i * iError + d * dError;
    checkOverflow(output,deadband[0]);
    checkDeadband(output,deadband[1]);
}

float PID::getOutput()
{
    return output;
}

void PID::checkOverflowIntegral(float &integral, float max_iError)
{
    if (fabs(integral)>max_iError)
        integral=sgn(integral)*max_iError;
}
void PID::checkOverflow(float &out, float max_out)
{
    if (fabs(out)>max_out)
        out=sgn(out)*max_out;
}
float PID::checkInput(float *des)
{
    if (*des==0)
        iError=0;
}
void PID::checkDeadband(float &out,float min_value)
{
    if (fabs(out)<min_value)
        out=0;
}
