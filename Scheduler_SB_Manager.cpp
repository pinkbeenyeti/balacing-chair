#include "Scheduler_SB_Manager.h"

const unsigned char Scheduler_SB_Garage_Num = 10;
Scheduler_SB Scheduler_SB_Garage[Scheduler_SB_Garage_Num];
unsigned char Scheduler_SB_Manager::Garage_Stack = 0;
volatile bool Scheduler_SB_Manager::overflowing;
volatile unsigned int Scheduler_SB_Manager::tcnt2;

// void Scheduler_SB_Manager::init()
// {
//     float prescaler = 0.0;

// #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//     TIMSK2 &= ~(1 << TOIE2);
//     TCCR2A &= ~((1 << WGM21) | (1 << WGM20));
//     TCCR2B &= ~(1 << WGM22);
//     ASSR &= ~(1 << AS2);
//     TIMSK2 &= ~(1 << OCIE2A);

//     if ((F_CPU >= 1000000UL) && (F_CPU <= 16000000UL))
//     { // prescaler set to 64
//         // TCCR2B |= (1 << CS22);
//         // TCCR2B &= ~((1 << CS21) | (1 << CS20));
//         // prescaler = 64.0;
//         TCCR2B |= ((1 << CS20));
//         TCCR2B &= ~((1 << CS22) | (1 << CS21));
//         prescaler = 1.0;
//     }
//     else if (F_CPU < 1000000UL)
//     { // prescaler set to 8
//         TCCR2B |= (1 << CS21);
//         TCCR2B &= ~((1 << CS22) | (1 << CS20));
//         prescaler = 8.0;
//     }
//     else
//     { // F_CPU > 16Mhz, prescaler set to 128
//         TCCR2B |= ((1 << CS22) | (1 << CS20));
//         TCCR2B &= ~(1 << CS21);
//         prescaler = 128.0;
//     }

//     ////////////////////

//     // tcnt2 = 256 - (int)((float)F_CPU * 0.001 / prescaler);

//     // TCNT2 = tcnt2;
//     TIMSK2 |= (1 << TOIE2);
// #elif defined(__AVR_ATmega8__)
//     TIMSK &= ~(1 << TOIE2);
//     TCCR2 &= ~((1 << WGM21) | (1 << WGM20));
//     TIMSK &= ~(1 << OCIE2);
//     ASSR &= ~(1 << AS2);

//     if ((F_CPU >= 1000000UL) && (F_CPU <= 16000000UL))
//     { // prescaler set to 64
//         TCCR2 |= (1 << CS22);
//         TCCR2 &= ~((1 << CS21) | (1 << CS20));
//         prescaler = 64.0;
//     }
//     else if (F_CPU < 1000000UL)
//     { // prescaler set to 8
//         TCCR2 |= (1 << CS21);
//         TCCR2 &= ~((1 << CS22) | (1 << CS20));
//         prescaler = 8.0;
//     }
//     else
//     { // F_CPU > 16Mhz, prescaler set to 128
//         TCCR2 |= ((1 << CS22) && (1 << CS20));
//         TCCR2 &= ~(1 << CS21);
//         prescaler = 128.0;
//     }

//     tcnt2 = 256 - (int)((float)F_CPU * 0.001 / prescaler);

//     TCNT2 = tcnt2;
//     TIMSK |= (1 << TOIE2);
// #elif defined(__AVR_ATmega128__)
//     TIMSK &= ~(1 << TOIE2);
//     TCCR2 &= ~((1 << WGM21) | (1 << WGM20));
//     TIMSK &= ~(1 << OCIE2);

//     if ((F_CPU >= 1000000UL) && (F_CPU <= 16000000UL))
//     { // prescaler set to 64
//         TCCR2 |= ((1 << CS21) | (1 << CS20));
//         TCCR2 &= ~(1 << CS22);
//         prescaler = 64.0;
//     }
//     else if (F_CPU < 1000000UL)
//     { // prescaler set to 8
//         TCCR2 |= (1 << CS21);
//         TCCR2 &= ~((1 << CS22) | (1 << CS20));
//         prescaler = 8.0;
//     }
//     else
//     { // F_CPU > 16Mhz, prescaler set to 256
//         TCCR2 |= (1 << CS22);
//         TCCR2 &= ~((1 << CS21) | (1 << CS20));
//         prescaler = 256.0;
//     }

//     tcnt2 = 256 - (int)((float)F_CPU * 0.001 / prescaler);

//     TCNT2 = tcnt2;
//     TIMSK |= (1 << TOIE2);
// #endif
// }

// void Scheduler_SB_Manager::stop()
// {
// #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega328P__) || (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//     TIMSK2 &= ~(1 << TOIE2);
// #elif defined(__AVR_ATmega128__)
//     TIMSK &= ~(1 << TOIE2);
// #elif defined(__AVR_ATmega8__)
//     TIMSK &= ~(1 << TOIE2);
// #endif
// }

unsigned long micro_t = 0;
void Scheduler_SB_Manager::idle()
{
    micro_t = micros();
    for (unsigned int i = 0; i < Garage_Stack; i++)
    {
        (Scheduler_SB_Garage[i]).Execute(micro_t);
    }

    // for (unsigned int i = 0; i < Scheduler_SB_Stack-1; i++)
    // {
    //     for (unsigned int j = i+1; j < Scheduler_SB_Stack; j++)
    //     {
    //         if (((*Scheduler_SB_Garage[i]).get_priority() < (*Scheduler_SB_Garage[j]).get_priority()))
    //         {
    //             (*Scheduler_SB_Garage[Scheduler_SB_Stack + 1]) = (*Scheduler_SB_Garage[i]);
    //             (*Scheduler_SB_Garage[i]) = (*Scheduler_SB_Garage[j]);
    //             (*Scheduler_SB_Garage[j]) = (*Scheduler_SB_Garage[Scheduler_SB_Stack + 1]);
    //         }
    //     }
    //     // (*Scheduler_SB_Garage[i]).Execute();
    // }
    // for(unsigned int i = 0; i < Scheduler_SB_Stack; i++)
    // {
    //     if((*Scheduler_SB_Garage[i]).get_available())
    //     {
    //         (*Scheduler_SB_Garage[i]).Execute();
    //         return;

    //     }
    // }
}
void Scheduler_SB_Manager::add(Scheduler_SB SB)
{
    if(Garage_Stack >= Scheduler_SB_Garage_Num)
        return;

    Scheduler_SB_Garage[Garage_Stack++] = SB;

    return;
}

void Scheduler_SB_Manager::remove(Scheduler_SB *SB)
{

    for (unsigned int i = 0; i < Garage_Stack; i++)
    {
        if (&Scheduler_SB_Garage[i] == SB)
        {
            for (unsigned int j = i; j < Garage_Stack - 1; j++)
            {
                Scheduler_SB_Garage[j] = Scheduler_SB_Garage[j + 1];
            }
        }
    }
    Garage_Stack--;
    return;
}

// ISR(TIMER2_OVF_vect)
// {
// #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega328P__) || (__AVR_ATmega1280__)
//     TCNT2 = Scheduler_SB_Manager::tcnt2;
// #elif defined(__AVR_ATmega128__)
//     TCNT2 = MsTimer2::tcnt2;
// #elif defined(__AVR_ATmega8__)
//     TCNT2 = MsTimer2::tcnt2;
// #endif
//     if (!Scheduler_SB_Manager::overflowing)
//     {
//         TCNT2 = 51;
//         Scheduler_SB_Manager::overflowing = true;
//         //SB_Stepper::Stepper_Pulse_1();
//         Scheduler_SB_Manager::idle();
//         Scheduler_SB_Manager::overflowing = false;
//     }
// }
