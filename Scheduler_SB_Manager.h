#ifndef Scheduler_SB_Manager_H
#define Scheduler_SB_Manager_H

#include "Scheduler_SB.h"
#include <avr/interrupt.h>

namespace Scheduler_SB_Manager
{
	extern volatile bool overflowing;
	extern volatile unsigned int tcnt2;
	extern unsigned char Garage_Stack;
	// Scheduler_SB **Scheduler_SB_Garage;
	// void init();
	// void stop();
	void overflow();
	void idle();
	void add(Scheduler_SB);
	void remove(Scheduler_SB *);
	
} // namespace Scheduler_SB_Manager

#endif
