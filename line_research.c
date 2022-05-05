#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <line_research.h>
#include <process_image.h>


static THD_WORKING_AREA(waFindLine, 256);
static THD_FUNCTION(FindLine, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    systime_t time = chVTGetSystemTime();


    while(1){
    	systime_t time_search = chVTGetSystemTime();
    	while(1){

    		right_motor_set_speed(MOTOR_SPEED_LIMIT/3);
    		left_motor_set_speed(-MOTOR_SPEED_LIMIT/3);
    		if(return_line_detected()){
    			right_motor_set_speed(0);
				left_motor_set_speed(0);
				chThdSleepMilliseconds(100);
//    			Send(PARK);
    		}
    		chThdSleepMilliseconds(100);
    		if(time_search + 6000 < chVTGetSystemTime()) {
    			break;
    		}
    	}
    	right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
		left_motor_set_speed(MOTOR_SPEED_LIMIT/2);
//		Send(CLEAN);
		chThdSleepMilliseconds(100);
    }
}

void find_line_start(void){
	chThdCreateStatic(waFindLine, sizeof(waFindLine), NORMALPRIO+2, FindLine, NULL);
}

