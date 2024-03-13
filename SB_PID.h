#ifndef SB_PID_H
#define SB_PID_H

#include <Arduino.h>

class SB_PID
{
    private :
    float kp = 0, ki = 0, kd = 0;
    // Gearing_SB *Gearing_input;
    // Gearing_SB *Gearing_output;
    float prev_err = 0;
    float i_err = 0;
    float d_err = 0;
    float output;
    unsigned long ex_time = 0;

    public :
    SB_PID();
    SB_PID(float,float,float);
    // PID_SB(float,float,float,Gearing_SB *,Gearing_SB *);
    // void set(Gearing_SB *,Gearing_SB *);
    void set(float, float, float);
    float get_kp();
    float get_ki();
    float get_kd();
    float Output(float, float);
    void Initiation();
    // void Aim(float);
};


#endif
