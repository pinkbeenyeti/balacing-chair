#ifndef SB_Stepper_Manager_H
#define SB_Stepper_Manager_H

#include <Arduino.h>
#include "SB_Stepper_indi.h"

namespace SB_Stepper_Manager
{
    extern unsigned int SB_Stepper_indi_Stack; 
    extern bool move_start;
    extern long dif_gap;
    extern bool deacc_flag;

    extern unsigned int tcnt2;

    void init();
    void stop();
    void add(SB_Stepper_indi *);
    void remove(SB_Stepper_indi *);

    void Manager();
}

#endif
