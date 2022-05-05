#include "ch.h"
#include "hal.h"
#include <math.h>
#include <stdlib.h>
#include <usbcfg.h>
#include <chprintf.h>

<<<<<<< HEAD
#include <sensors/proximity.h>
=======
#include "sensors/proximity.h"
#include <process_image.h>
>>>>>>> Nathan
#include <main.h>
#include <detect_proximity.h>
#include <motor_control.h>
#include <motors.h>

static thread_t *tp;
static THD_WORKING_AREA(waMotorControl, 256);
static THD_FUNCTION(MotorControl, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    systime_t time;


    while(1){
    	time = chVTGetSystemTime();

    	if(!return_wall_detected() || (abs(return_wall_angle()) > 90)){
			right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
			left_motor_set_speed(MOTOR_SPEED_LIMIT/2);
		}
    	else if((return_wall_angle() >= -10) && (return_wall_angle() <= 10)){
    		int random_number = rand()%2;
    		if(random_number == 0){
    			right_motor_set_speed(-MOTOR_SPEED_LIMIT/2);
				left_motor_set_speed(MOTOR_SPEED_LIMIT/2);
				chThdSleepUntilWindowed(time, time + MS2ST(500));
    		}
    		else {
    			right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
				left_motor_set_speed(-MOTOR_SPEED_LIMIT/2);
				chThdSleepUntilWindowed(time, time + MS2ST(500));
    		}
    	}
    	else if(return_wall_angle() > 0 ){
    		if((return_wall_angle() == 90) && (prox_value_delta(5) > 150)){
    			right_motor_set_speed(-MOTOR_SPEED_LIMIT/2);
				left_motor_set_speed(MOTOR_SPEED_LIMIT/2);
    		}
    		right_motor_set_speed(MOTOR_SPEED_LIMIT/2 - MOTOR_SPEED_LIMIT/2 * (90 - return_wall_angle()));
    		left_motor_set_speed(MOTOR_SPEED_LIMIT/2 + MOTOR_SPEED_LIMIT/2 * (90 - return_wall_angle()));
    	}
    	else if(return_wall_angle() < 0){
    		if((return_wall_angle() == -90) && (prox_value_delta(2) > 150)){
				right_motor_set_speed(MOTOR_SPEED_LIMIT/2);
				left_motor_set_speed(-MOTOR_SPEED_LIMIT/2);
			}
    		right_motor_set_speed(MOTOR_SPEED_LIMIT/2 + MOTOR_SPEED_LIMIT/2 * (90 - return_wall_angle()));
			left_motor_set_speed(MOTOR_SPEED_LIMIT/2 - MOTOR_SPEED_LIMIT/2 * (90 - return_wall_angle()));
    	}


    	chThdSleepUntilWindowed(time, time + MS2ST(20));
    }
}



void motor_control_start(void){
	chThdCreateStatic(waMotorControl, sizeof(waMotorControl), NORMALPRIO+1, MotorControl, NULL);
}

void motor_control_stop(void){
	chThdTerminate(tp);
}
