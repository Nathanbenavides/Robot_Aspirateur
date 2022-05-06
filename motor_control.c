#include "ch.h"
#include "hal.h"
#include <math.h>
#include <stdlib.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include <sensors/proximity.h>

#include <detect_proximity.h>
#include <motor_control.h>
#include <motors.h>
#include <line_research.h>

static bool motor_control_active = 1;
static systime_t time_running = 10000;
static systime_t time_start = 0;
static thread_t *tp;
static THD_WORKING_AREA(waMotorControl, 256);
static THD_FUNCTION(MotorControl, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    systime_t time;
//    if(time_running == 0){
//    time_running = chVTGetSystemTime();
//    }
    while(1){
    	if(time_start == 0){
    		time_start = chVTGetSystemTime();
    	    }
    	if(motor_control_active){
			if((time_start + MS2ST(time_running)) < chVTGetSystemTime()){
				set_search_line(1);
			}
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
    	}

    	chThdSleepUntilWindowed(time, time + MS2ST(20));
    }
}



void motor_control_start(void){
	tp = chThdCreateStatic(waMotorControl, sizeof(waMotorControl), NORMALPRIO, MotorControl, NULL);
}

void motor_control_stop(void){
    chThdTerminate(tp);
    chThdWait(tp);
    tp = NULL;

}

void set_motor_control_active(bool set){
	time_start = 0;
	motor_control_active = set;
}

void set_time_running(systime_t set){
	time_running = set;
}
