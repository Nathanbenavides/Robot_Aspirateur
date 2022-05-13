#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>

#include <line_research.h>
#include <motor_control.h>
#include <process_image.h>


static thread_t *findLineThd;
static uint8_t findLine_configured = 0;

static THD_WORKING_AREA(waFindLine, 256);
static THD_FUNCTION(FindLine, arg) {		//Thread called to search the black line

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time_search = chVTGetSystemTime();

    while(!chThdShouldTerminateX()){

    	right_motor_set_speed(LOW_SPEED);		//robot rotates at low speed so there is enough time to take and process the images
		left_motor_set_speed(-LOW_SPEED);

		if(return_line_detected()){				//line detected -> robot goeas to "PARK" state
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			send(PARK);
		}
		else if(chVTGetSystemTime() >= time_search + MS2ST(TIME_WAIT_SEARCHING_ROTA)){ //searching a line for TIME_WAIT_SEARCHING_ROTA if no line found
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			send(RESEARCH_MVNT);
		}

		chThdSleepMilliseconds(LINE_RESEARCH_WAIT_TIME);
    }
}

void find_line_start(void){						//starts the line search thread
	if(findLine_configured) return;

	findLineThd = chThdCreateStatic(waFindLine, sizeof(waFindLine), NORMALPRIO, FindLine, NULL);
	findLine_configured = 1;
}

void find_line_stop(void){						//stops the line search thread
	if(!findLine_configured) return;

	chThdTerminate(findLineThd);
	chThdWait(findLineThd);
	findLineThd = NULL;

	findLine_configured = 0;
}


