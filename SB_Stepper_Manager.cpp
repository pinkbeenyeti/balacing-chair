#include "SB_Stepper_Manager.h"
#include "Scheduler_SB_Manager.h"



#define ISR_HZ 20000
#define Manager_HZ 100


SB_Stepper_indi **SB_Stepper_indi_Garage;
unsigned int SB_Stepper_Manager::SB_Stepper_indi_Stack = 0;
bool SB_Stepper_Manager::move_start = false;
long SB_Stepper_Manager::dif_gap = 0;
bool SB_Stepper_Manager::deacc_flag = false;
unsigned int SB_Stepper_Manager::tcnt2 = 0;

Scheduler_SB Manager1(SB_Stepper_Manager::Manager, 10, 1000000 / Manager_HZ);

void SB_Stepper_Manager::init()
{
    Scheduler_SB_Manager::add(Manager1);

    float prescaler = 0;

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    TIMSK2 &= ~(1 << TOIE2);
    TCCR2A &= ~((1 << WGM21) | (1 << WGM20));
    TCCR2B &= ~(1 << WGM22);
    ASSR &= ~(1 << AS2);
    TIMSK2 &= ~(1 << OCIE2A);

    if ((F_CPU >= 1000000UL) && (F_CPU <= 16000000UL))
    {
        TCCR2B |= ((1 << CS21));
        TCCR2B &= ~((1 << CS22) | (1 << CS20));
        prescaler = 8.0;
    }
    else if (F_CPU < 1000000UL)
    { // prescaler set to 1
        TCCR2B |= (1 << CS20);
        TCCR2B &= ~((1 << CS22) | (1 << CS21));
        prescaler = 1.0;
    }
    else
    { // F_CPU > 16Mhz, prescaler set to 32
        TCCR2B |= ((1 << CS22) | (1 << CS21));
        TCCR2B &= ~(1 << CS20);
        prescaler = 32.0;
    }

    ////////////////////

    tcnt2 = 256 - (int)((float)F_CPU / (prescaler * (ISR_HZ*1.05)));

    // TCNT2 = tcnt2;
    TIMSK2 |= (1 << TOIE2); // ON
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
#endif
    //Move(2,6400,20);
    // long a[5] = {6400,6400,6400,3200,1000};
    // time___ = micros();
    // SB_Stepper::Snchrozie_MoveTo__Sec(a,1);
}

void SB_Stepper_Manager::stop()
{
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega48__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega328P__) || (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    TIMSK2 &= ~(1 << TOIE2);
#elif defined(__AVR_ATmega128__)
    TIMSK &= ~(1 << TOIE2);
#elif defined(__AVR_ATmega8__)
    TIMSK &= ~(1 << TOIE2);
#endif
}


ISR(TIMER2_OVF_vect)
{
    TCNT2 = SB_Stepper_Manager::tcnt2;

    
    for(int i=0; i<SB_Stepper_Manager::SB_Stepper_indi_Stack; i++)
        (*SB_Stepper_indi_Garage[i]).Pulse();
    return;
}


void SB_Stepper_Manager::add(SB_Stepper_indi *child)
{
    if (SB_Stepper_Manager::SB_Stepper_indi_Stack == 0)
    {
        SB_Stepper_indi_Garage = (SB_Stepper_indi **)malloc(sizeof(SB_Stepper_indi *) * 1);
        SB_Stepper_indi_Garage[0] = (SB_Stepper_indi *)malloc(sizeof(SB_Stepper_indi));
    }
    for (unsigned int i = 0; i < SB_Stepper_Manager::SB_Stepper_indi_Stack; i++)
    {
        if (SB_Stepper_indi_Garage[i] == child)
            return;
    }
    SB_Stepper_Manager::SB_Stepper_indi_Stack++;
    SB_Stepper_indi_Garage = (SB_Stepper_indi **)realloc(SB_Stepper_indi_Garage, sizeof(SB_Stepper_indi) * (SB_Stepper_Manager::SB_Stepper_indi_Stack + 1));
    SB_Stepper_indi_Garage[SB_Stepper_Manager::SB_Stepper_indi_Stack - 1] = child;
}

void SB_Stepper_Manager::remove(SB_Stepper_indi *child)
{
    for (unsigned int i = 0; i < SB_Stepper_Manager::SB_Stepper_indi_Stack; i++)
    {
        if (SB_Stepper_indi_Garage[i] == child)
        {
            for (unsigned int j = i; j < SB_Stepper_Manager::SB_Stepper_indi_Stack - 1; j++)
            {
                SB_Stepper_indi_Garage[j] = SB_Stepper_indi_Garage[j + 1];
            }
        }
    }
    SB_Stepper_Manager::SB_Stepper_indi_Stack--;
    SB_Stepper_indi_Garage = (SB_Stepper_indi **)realloc(SB_Stepper_indi_Garage, sizeof(SB_Stepper_indi) * SB_Stepper_Manager::SB_Stepper_indi_Stack);
    return;
}


void SB_Stepper_Manager::Manager()
{
  //Serial.println("MANAGER");
    int cnt = 0;
    long max_gap = 0;
    int Snchronize_Center_Pin = 0;
    for(int i=0; i<SB_Stepper_indi_Stack; i++)
    {
        if ((*SB_Stepper_indi_Garage[i]).Pos_now == (*SB_Stepper_indi_Garage[i]).Pos_goal)
        {
            if(!(*SB_Stepper_indi_Garage[i]).stadby_forced)
                (*SB_Stepper_indi_Garage[i]).Stop();

            cnt++;
        }
        else
        {
            if(!(*SB_Stepper_indi_Garage[i]).stadby_forced)
                (*SB_Stepper_indi_Garage[i]).Start();

            if (max_gap < (*SB_Stepper_indi_Garage[i]).Pos_gap)
            {
                Snchronize_Center_Pin = i;
                max_gap = (*SB_Stepper_indi_Garage[i]).Pos_gap;
            }

            if((*SB_Stepper_indi_Garage[i]).Pulse_enable)
                move_start = true;
        }

    }

    if(cnt == SB_Stepper_indi_Stack)
    {
        move_start = true; ////// 다음 움직임을 가능하게 해줌
        return;
    }


    if (move_start)
    {
        dif_gap = abs((*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Pos_now
                    - (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Pos_goal);
        move_start = false;

        for (int i = 0; i < SB_Stepper_indi_Stack; i++)
        {
            (*SB_Stepper_indi_Garage[i]).Result_Duty_f_gap = 0;
            if ((*SB_Stepper_indi_Garage[i]).Pos_now == (*SB_Stepper_indi_Garage[i]).Pos_goal)
            {
                (*SB_Stepper_indi_Garage[i]).Velocity_Rate = 0;
                continue;
            }
            
            (*SB_Stepper_indi_Garage[i]).Velocity_Rate = (*SB_Stepper_indi_Garage[i]).Pos_gap/max_gap;

            if((*SB_Stepper_indi_Garage[i]).Goal_Pulse_Sps < (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Goal_Pulse_Sps * (*SB_Stepper_indi_Garage[i]).Velocity_Rate)
            {
                (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Goal_Pulse_Sps = (*SB_Stepper_indi_Garage[i]).Goal_Pulse_Sps / (*SB_Stepper_indi_Garage[i]).Velocity_Rate;
            }
        }
    }

    // Serial.print("Now_Sps ");
    // Serial.print(Now_Sps[Snchronize_Center_Pin]);
    // Serial.print(" Goal_Pulse_Sps ");
    // Serial.print( Goal_Pulse_Sps[Snchronize_Center_Pin] );

    // Serial.print(" 남은 시간 ");
    // Serial.print(float(abs(Pos_goal[Snchronize_Center_Pin] - Pos_now[Snchronize_Center_Pin])/Now_Sps[Snchronize_Center_Pin]));
    // Serial.print(" 접점 시간 ");
    // Serial.print( float((Now_Sps[Snchronize_Center_Pin] - Min_Sps_28byj_48)/Acc_Sps) );
    // Serial.print("\n");

    if ((*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Now_Sps < (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Goal_Pulse_Sps
        && 2 * abs((*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Pos_goal - (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Pos_now) / (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Now_Sps
         >= ((*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Now_Sps - (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Min_Sps) / (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Acc_Sps)
    {
        // Serial.println("증가");
        (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Now_Sps += (float)(*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Acc_Sps / Manager_HZ;
        (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Now_Sps = constrain((*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Now_Sps, 
                                                                            (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Min_Sps, 
                                                                            (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Max_Sps);
    }
    else if (2 * abs((*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Pos_goal - (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Pos_now) 
            / (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Now_Sps < ((*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Now_Sps - (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).End_Sps) / (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Acc_Sps || !deacc_flag)
    {
        // if(t_b)
        // {
        //     t_b = false;
        //     Serial.print("now ");
        //     Serial.print(Pos_now[Snchronize_Center_Pin]);
        //     Serial.print(" goal ");
        //     Serial.print(Pos_goal[Snchronize_Center_Pin]);
        //     Serial.print("  ");
        //     Serial.print(2*abs(Pos_goal[Snchronize_Center_Pin] - Pos_now[Snchronize_Center_Pin])/Now_Sps[Snchronize_Center_Pin]);
        //     Serial.print(" < ");
        //     Serial.print((Now_Sps[Snchronize_Center_Pin] - Min_Sps_28byj_48)/Acc_Sps);
        //     Serial.println("\n 감소");
        // }
        deacc_flag = false;
        (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Now_Sps -= (float)(*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Acc_Sps / Manager_HZ;
        (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Now_Sps = constrain((*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Now_Sps,
                                                                            (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Min_Sps, 
                                                                            (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Max_Sps);
    }


    for (int i = 0; i < SB_Stepper_indi_Stack; i++)
    {
        if (i != Snchronize_Center_Pin)
            (*SB_Stepper_indi_Garage[i]).Now_Sps = (float)((*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Now_Sps * (*SB_Stepper_indi_Garage[i]).Velocity_Rate);

        // Serial.print(Now_Sps[i]);
        // Serial.print("  ");

        float now_duty = ISR_HZ / (*SB_Stepper_indi_Garage[i]).Now_Sps;

        //(*SB_Stepper_indi_Garage[i]).Result_Duty_f_gap += now_duty - (unsigned int)(now_duty);

        //(*SB_Stepper_indi_Garage[i]).Result_Duty = constrain((unsigned int)(now_duty + (*SB_Stepper_indi_Garage[i]).Result_Duty_f_gap), ISR_HZ / (*SB_Stepper_indi_Garage[i]).Max_Sps, ISR_HZ / (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Min_Sps);

        //Serial.print("now sps : ");
        //Serial.print(now_duty);
        
        (*SB_Stepper_indi_Garage[i]).Result_Duty = constrain((unsigned int)now_duty , ISR_HZ / (*SB_Stepper_indi_Garage[i]).Max_Sps, ISR_HZ / (*SB_Stepper_indi_Garage[Snchronize_Center_Pin]).Min_Sps);

        //Serial.print("Result_Duty : ");
        //Serial.println((*SB_Stepper_indi_Garage[i]).Result_Duty);
        
        //(*SB_Stepper_indi_Garage[i]).Result_Duty_f_gap -= ((float)(*SB_Stepper_indi_Garage[i]).Result_Duty - now_duty);
    }
    // Serial.print("\n");

    return;
}
