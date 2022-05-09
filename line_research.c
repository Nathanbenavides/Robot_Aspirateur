#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <motor_control.h>
#include <line_research.h>
#include <process_image.h>

#include <leds.h>

static thread_t *findLineThd;
static uint8_t findLine_configured = 0;

static THD_WORKING_AREA(waFindLine, 256);
static THD_FUNCTION(FindLine, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time_search = chVTGetSystemTime();

    while(!chThdShouldTerminateX()){

    	right_motor_set_speed(LOW_SPEED);
		left_motor_set_speed(-LOW_SPEED);

		if(return_line_detected()){
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			send(PARK);
		}
		else if(chVTGetSystemTime() >= time_search + MS2ST(TIME_WAIT_SEARCHING_ROTA)){ //searching a line for TIME_WAIT_SEARCHING_ROTA
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			send(RESEARCH_MVNT);
		}

		//chprintf((BaseSequentialStream *)&SDU1, "Find Line");

		chThdSleepMilliseconds(100);

    }
}

void find_line_start(void){
	if(findLine_configured) return;

	findLineThd = chThdCreateStatic(waFindLine, sizeof(waFindLine), NORMALPRIO, FindLine, NULL);
	findLine_configured = 1;
}

void find_line_stop(void){
	if(findLine_configured == 0) return;

	chThdTerminate(findLineThd);
	chThdWait(findLineThd);
	findLineThd = NULL;

	findLine_configured = 0;
}


