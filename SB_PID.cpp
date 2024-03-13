#include "SB_PID.h"

SB_PID::SB_PID()
{
}
SB_PID::SB_PID(float _p, float _i, float _d)
{
    kp = _p;
    ki = _i;
    kd = _d;
}

// PID_SB::PID_SB(float _p,float _i,float _d,Gearing_SB *input_,Gearing_SB *output_)
// {
//     kp = _p;
//     ki = _i;
//     kd = _d;

//     Gearing_input = input_;
//     Gearing_output = output_;
// }

// void PID_SB::set(Gearing_SB *input_,Gearing_SB *output_)
// {
//     Gearing_input = input_;
//     Gearing_output = output_;
// }

void SB_PID::set(float _p, float _i, float _d)
{
    kp = _p;
    ki = _i;
    kd = _d;
}

float SB_PID::get_kp()
{
    return kp;
}

float SB_PID::get_ki()
{
    return ki;
}

float SB_PID::get_kd()
{
    return kd;
}



float SB_PID::Output(float goal, float input)
{
    unsigned long t = micros();
    float dt = t > ex_time ? (float)(t - ex_time) : (float)(4294967295 - ex_time + t);

    dt = dt * 0.000001;

    float err = goal - input;
    i_err += err*dt;
    float i_err_limit = 255/ki;
    i_err = i_err > i_err_limit ? i_err_limit : i_err < -i_err_limit ? -i_err_limit : i_err;
    d_err = (err - prev_err)/dt;

    ex_time = t;
    output = kp * err + ki * i_err + kd * d_err;
    return output;
}


void SB_PID::Initiation()
{
    i_err = 0;
}
// void PID_SB::Aim(float aim)
// {
//     unsigned long t = micros();
//     float dt = t > ex_time ? (float)(t - ex_time)/1000000 : (float)(4294967295 - ex_time + t)/1000000;

//     float err = aim - (*Gearing_input).Gearing_float();
//     i_err += err*dt;
//     d_err = (err - prev_err)/dt;

//     ex_time = t;
//     output = (kp * err + ki * i_err + kd * d_err);
//     (*Gearing_output).Gearing(&output);
// }
