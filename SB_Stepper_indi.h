#ifndef SB_Stepper_indi_H
#define SB_Stepper_indi_H

class SB_Stepper_indi
{
    private :
    int TYPE;
    public : 

    int A,B,C,D;
    int Limit_Pin;
    bool Limit_Switch_Reverse = false;
    int EN, DIR, STEP;
    bool Step_Dir;
    
    bool stadby_forced = true;
    bool Pulse_enable = false;
    bool Enable_flag = false;
    bool Snchronize = false;

    long Pos_now;
    long Pos_goal;
    long Pos_gap;
    float Step_Per_Unit = 1;
    float Velocity_Rate;

    bool Direct;
    unsigned int prev_duty;
    unsigned int Result_Duty;
    float Result_Duty_f_gap;

    float Now_Sps;
    float Goal_Pulse_Sps;
    
    unsigned int Min_Sps = 10;  // Min sps(step per sec) for Activation
    unsigned int Max_Sps = 1000;  // Max sps for Activation
    unsigned int End_Sps = 10;  // Activation last sps
    unsigned int Acc_Sps = 100;  

    bool stop_flag;
    int Step_Level = 0;

    SB_Stepper_indi();
    SB_Stepper_indi(int limit_pin, int EN, int DIR, int STEP, bool Step_Dir); // EN, DIR, STEP, Spin_Dir(bool)
    SB_Stepper_indi(int limit_pin, int A, int B, int C, int D, bool Step_Dir); // A, B, C, D, Spin_Dir(bool)

    void DIR_SET(bool);

    void MoveTo(float goal, unsigned int sps);
    void Set_Step_Per_Unit(float);
    void Set_Sps(unsigned int min, unsigned int max);
    void Set_AccSps(unsigned int);
    void Set_Origin(float);
    void Set_Origin();
    void Set_Limit_Switch_Reverse(bool);
    bool Complete();
    long Pause();
    void Pulse();

    void Start();
    void Stop();
    float Get_Step();

    void digitalwrite_high(int pin);
    void digitalwrite_low(int pin);
    bool digitalread_Fast(int pin);
};

#endif
