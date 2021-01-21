#ifndef PID_H
#define PID_H

/*Universal PID controller class*/
class PID
{
public:
    float pError;
    float iError;
    float dError;
    float p;
    float i;
    float d;
    float max_i;
    float old_pError;
    float deadband[2];
    float output;

    PID(float cp, float ci, float cd, float max_out, float min_value);
    void update(float cp, float ci, float cd);
    void loop(float *desired, float *real);
    float getOutput();

private:
    void checkOverflowIntegral(float &integral, float max_iError);
    void checkOverflow(float &out, float max_out);
    float checkInput(float *des);
    void checkDeadband(float &out,float min_value);
};
#endif
