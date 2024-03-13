#include "SB_Stepper_indi.h"
#include <Arduino.h>
#include "digitalWriteFast.h"
#include "Scheduler_SB_Manager.h"

#define Constrain(x, min, max) (x)<(min) ? (min) : (x)>(max) ? (max) : (x);
#define Constrain_Max(x, max) (x) > (max) ? (max) : (x);
#define Constrain_Min(x, min) (x) < (min) ? (min) : (x);

SB_Stepper_indi::SB_Stepper_indi()
{
    TYPE = -1;
}

SB_Stepper_indi::SB_Stepper_indi(int limit_pin, int EN_pin, int DIR_pin, int STEP_pin, bool dir)
{
    TYPE = 1;
    Limit_Pin = limit_pin;
    EN = EN_pin;
    DIR = DIR_pin;
    STEP = STEP_pin;
    Step_Dir = dir;

    pinMode(Limit_Pin, INPUT);

    pinMode(EN, OUTPUT);

    pinMode(DIR, OUTPUT);
    
    pinMode(STEP, OUTPUT);

    digitalwrite_low(EN);
}

SB_Stepper_indi::SB_Stepper_indi(int limit_pin, int A_pin, int B_pin, int C_pin, int D_pin, bool dir)
{

    TYPE = 0;
    Limit_Pin = limit_pin;
    A = A_pin;
    B = B_pin;
    C = C_pin;
    D = D_pin;
    Step_Dir = dir;
    pinMode(Limit_Pin, INPUT);

    pinMode(A,OUTPUT);
    pinMode(B,OUTPUT);
    pinMode(C,OUTPUT);
    pinMode(D,OUTPUT);
}

void SB_Stepper_indi::DIR_SET(bool dir)
{
    Step_Dir = dir;
}

void SB_Stepper_indi::MoveTo(float goal, unsigned int sps)
{
    Pos_goal = goal * Step_Per_Unit;
    
    if(Pos_now == Pos_goal)
        return;
    
    long dif_t = Pos_goal - Pos_now;

    Direct = dif_t > 0 ? true : false;

    Pulse_enable = true;

    Goal_Pulse_Sps = Constrain(sps, Min_Sps, Max_Sps);

    Pos_gap = abs(dif_t);
}

void SB_Stepper_indi::Set_Step_Per_Unit(float val)
{
    Step_Per_Unit = val;
}

void SB_Stepper_indi::Set_Sps(unsigned int min, unsigned int max)
{
    Max_Sps = max;
    Min_Sps = min;
}

void SB_Stepper_indi::Set_AccSps(unsigned int accsps)
{
    Acc_Sps = accsps;
}

void SB_Stepper_indi::Set_Origin(float pos)
{
    Pos_goal = Pos_now = pos * Step_Per_Unit;
    Pulse_enable = false;
}

void SB_Stepper_indi::Set_Origin()
{
    bool hook = false;
    MoveTo(190, 7500);
    for(;;)
    {
        Scheduler_SB_Manager::idle();
        if(Limit_Switch_Reverse ? digitalread_Fast(Limit_Pin) : !digitalread_Fast(Limit_Pin))
        {
            hook = true;
            Pause();
            break;
        }

        if(Complete())
        {
            break;
        }
    }
    Set_Origin(0);

    if(!hook)
    {
        MoveTo(-380, 7500);
        for (;;)
        {
            if(Limit_Switch_Reverse ? digitalread_Fast(Limit_Pin) : !digitalread_Fast(Limit_Pin))
            {
                hook = true;
                Pause();
                break;
            }

            if (Complete())
                break;
            
            Scheduler_SB_Manager::idle();
        }
        Set_Origin(0);

        MoveTo(-10, 7500);
        for (;;)
        {
            if(Complete())
            {
                break;
            }

            Scheduler_SB_Manager::idle();
        }
    }
    Set_Origin(0);

    if(hook)
    {
        MoveTo(-10, 7500);
        for (;;)
        {
            if(Complete())
                break;

            Scheduler_SB_Manager::idle();
        }
        Set_Origin(0);

        MoveTo(50, 1000);
        for (;;)
        {
            if(Limit_Switch_Reverse ? digitalread_Fast(Limit_Pin) : !digitalread_Fast(Limit_Pin))
            {
                Pause();
                Set_Origin(-6);
                break;
            }
            Scheduler_SB_Manager::idle();
        }
        MoveTo(0, 5000);
    }



}

void SB_Stepper_indi::Set_Limit_Switch_Reverse(bool sw)
{
    Limit_Switch_Reverse = sw;
}

bool SB_Stepper_indi::Complete()
{
    return Pos_goal == Pos_now;
}

long SB_Stepper_indi::Pause()
{
    Pos_goal = Pos_now;
    Result_Duty = 65535;
    Result_Duty_f_gap = 0;
    return Pos_now;
}

void SB_Stepper_indi::Pulse()
{
    if(Pos_now == Pos_goal)
        return;

    prev_duty++;

    if(prev_duty >= 60)
    {
        prev_duty = 0;
        
        Direct ? Pos_now++ : Pos_now--;
        
        if(TYPE == 0)
        {
            switch (Step_Level)
            {
            case 0:
                digitalwrite_high(A);
                digitalwrite_low(B);
                digitalwrite_low(C);
                digitalwrite_low(D);
                break;
            case 1:
                digitalwrite_high(A);
                digitalwrite_high(B);
                digitalwrite_low(C);
                digitalwrite_low(D);
                break;
            case 2:
                digitalwrite_low(A);
                digitalwrite_high(B);
                digitalwrite_low(C);
                digitalwrite_low(D);
                break;
            case 3:
                digitalwrite_low(A);
                digitalwrite_high(B);
                digitalwrite_high(C);
                digitalwrite_low(D);
                break;
            case 4:
                digitalwrite_low(A);
                digitalwrite_low(B);
                digitalwrite_high(C);
                digitalwrite_low(D);
                break;
            case 5:
                digitalwrite_low(A);
                digitalwrite_low(B);
                digitalwrite_high(C);
                digitalwrite_high(D);
                break;
            case 6:
                digitalwrite_low(A);
                digitalwrite_low(B);
                digitalwrite_low(C);
                digitalwrite_high(D);
                break;
            case 7:
                digitalwrite_high(A);
                digitalwrite_low(B);
                digitalwrite_low(C);
                digitalwrite_high(D);
                break;
            default:
                break;
            }

            (Step_Dir ? Direct : !Direct) ? Step_Level++ : Step_Level--;
            Step_Level = Step_Level > 7 ? 0 : Step_Level < 0 ? 7 : Step_Level;
        }
        else if(TYPE == 1)
        {
            Direct ? digitalwrite_high(DIR) : digitalwrite_low(DIR);
            // digitalWrite(STEP, HIGH);
            // digitalWrite(STEP, LOW);
            digitalwrite_high(STEP);
            digitalwrite_low(STEP);
        }
    }
}


void SB_Stepper_indi::Start()
{
    if(!stop_flag)
        return;
    stop_flag = false;

    if(TYPE == 0)
    {
        switch (Step_Level)
        {
            case 0:
                digitalwrite_high(A);
                digitalwrite_low(B);
                digitalwrite_low(C);
                digitalwrite_low(D);
                break;
            case 1:
                digitalwrite_high(A);
                digitalwrite_high(B);
                digitalwrite_low(C);
                digitalwrite_low(D);
                break;
            case 2:
                digitalwrite_low(A);
                digitalwrite_high(B);
                digitalwrite_low(C);
                digitalwrite_low(D);
                break;
            case 3:
                digitalwrite_low(A);
                digitalwrite_high(B);
                digitalwrite_high(C);
                digitalwrite_low(D);
                break;
            case 4:
                digitalwrite_low(A);
                digitalwrite_low(B);
                digitalwrite_high(C);
                digitalwrite_low(D);
                break;
            case 5:
                digitalwrite_low(A);
                digitalwrite_low(B);
                digitalwrite_high(C);
                digitalwrite_high(D);
                break;
            case 6:
                digitalwrite_low(A);
                digitalwrite_low(B);
                digitalwrite_low(C);
                digitalwrite_high(D);
                break;
            case 7:
                digitalwrite_high(A);
                digitalwrite_low(B);
                digitalwrite_low(C);
                digitalwrite_high(D);
                break;
            default:
                break;
            }
    }
    else if(TYPE == 1)
    {
        digitalwrite_low(EN);
    }

    Pulse_enable = true;
}

bool Stop_flag = false;
void SB_Stepper_indi::Stop()
{
    if(stop_flag)
        return;
    stop_flag = true;

    Pos_goal = Pos_now;
    Result_Duty = 65535;
    Result_Duty_f_gap = 0;

    if(TYPE == 0)
    {
        digitalwrite_low(A);
        digitalwrite_low(B);
        digitalwrite_low(C);
        digitalwrite_low(D);
    }
    else if(TYPE == 1)
    {
        digitalwrite_high(EN);
    }

    Pulse_enable = false;
}

float SB_Stepper_indi::Get_Step()
{
    return (float)(Pos_now / Step_Per_Unit);
}

void SB_Stepper_indi::digitalwrite_high(int pin)
{
    switch(pin)
    {
        case 2 : 
            digitalWriteFast(2,HIGH);
            break;
        case 3 : 
            digitalWriteFast(3,HIGH);
            break;
        case 4 : 
            digitalWriteFast(4,HIGH);
            break;
        case 5 : 
            digitalWriteFast(5,HIGH);
            break;
        case 6 : 
            digitalWriteFast(6,HIGH);
            break;
        case 7 : 
            digitalWriteFast(7,HIGH);
            break;
        case 8 : 
            digitalWriteFast(8,HIGH);
            break;
        case 9 : 
            digitalWriteFast(9,HIGH);
            break;
        case 10 : 
            digitalWriteFast(10,HIGH);
            break;
        case 11 : 
            digitalWriteFast(11,HIGH);
            break;
        case 12 : 
            digitalWriteFast(12,HIGH);
            break;
        case 13 : 
            digitalWriteFast(13,HIGH);
            break;
        case 14 : 
            digitalWriteFast(14,HIGH);
            break;
        case 15 : 
            digitalWriteFast(15,HIGH);
            break;
        case 16 : 
            digitalWriteFast(16,HIGH);
            break;
        case 17 : 
            digitalWriteFast(17,HIGH);
            break;
        case 18 : 
            digitalWriteFast(18,HIGH);
            break;
        case 19 : 
            digitalWriteFast(19,HIGH);
            break;
        case 20 : 
            digitalWriteFast(20,HIGH);
            break;
        case 21 : 
            digitalWriteFast(21,HIGH);
            break;

        default :
            break;
    }
}

void SB_Stepper_indi::digitalwrite_low(int pin)
{
    switch(pin)
    {
        case 2 : 
            digitalWriteFast(2,LOW);
            break;
        case 3 : 
            digitalWriteFast(3,LOW);
            break;
        case 4 : 
            digitalWriteFast(4,LOW);
            break;
        case 5 : 
            digitalWriteFast(5,LOW);
            break;
        case 6 : 
            digitalWriteFast(6,LOW);
            break;
        case 7 : 
            digitalWriteFast(7,LOW);
            break;
        case 8 : 
            digitalWriteFast(8,LOW);
            break;
        case 9 : 
            digitalWriteFast(9,LOW);
            break;
        case 10 : 
            digitalWriteFast(10,LOW);
            break;
        case 11 : 
            digitalWriteFast(11,LOW);
            break;
        case 12 : 
            digitalWriteFast(12,LOW);
            break;
        case 13 : 
            digitalWriteFast(13,LOW);
            break;
        case 14 : 
            digitalWriteFast(14,LOW);
            break;
        case 15 : 
            digitalWriteFast(15,LOW);
            break;
        case 16 : 
            digitalWriteFast(16,LOW);
            break;
        case 17 : 
            digitalWriteFast(17,LOW);
            break;
        case 18 : 
            digitalWriteFast(18,LOW);
            break;
        case 19 : 
            digitalWriteFast(19,LOW);
            break;
        case 20 : 
            digitalWriteFast(20,LOW);
            break;
        case 21 : 
            digitalWriteFast(21,LOW);
            break;

        default :
            break;
    }
}

bool SB_Stepper_indi::digitalread_Fast(int pin)
{
    switch(pin)
    {
        case 2 : 
            return digitalReadFast(2);
        case 3 : 
            return digitalReadFast(3);
        case 4 : 
            return digitalReadFast(4);
        case 5 : 
            return digitalReadFast(5);
        case 6 : 
            return digitalReadFast(6);
        case 7 : 
            return digitalReadFast(7);
        case 8 : 
            return digitalReadFast(8);
        case 9 : 
            return digitalReadFast(9);
        case 10 : 
            return digitalReadFast(10);
        case 11 : 
            return digitalReadFast(11);
        case 12 : 
            return digitalReadFast(12);
        case 13 : 
            return digitalReadFast(13);
        case 14 : 
            return digitalReadFast(14);
        case 15 : 
            return digitalReadFast(15);
        case 16 : 
            return digitalReadFast(16);
        case 17 : 
            return digitalReadFast(17);
        case 18 : 
            return digitalReadFast(18);
        case 19 : 
            return digitalReadFast(19);
        case 20 : 
            return digitalReadFast(20);
        case 21 : 
            return digitalReadFast(21);

        default :
            return false;
    }
}
