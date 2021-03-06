#include "ch.h"
#include "hal.h"
#include <math.h>
#include <stdlib.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>

#include <detect_proximity.h>
#include <motor_control.h>


static thread_t *motContThd;
static uint8_t MotorControl_configured = 0;


static THD_WORKING_AREA(waMotorControl, 256);
static THD_FUNCTION(MotorControl, arg) {					//The thread sets the speed of the two motors depending on angle at which the wall is

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    systime_t time;

    while(!chThdShouldTerminateX()){

    	time = chVTGetSystemTime();

		if(!return_wall_detected() || (abs(return_wall_angle()) > ANGLE_BEHIND)){		//robot goes forward if there is no wall or if it is behind the robot
			right_motor_set_speed(MOTOR_SPEED);
			left_motor_set_speed(MOTOR_SPEED);
		}
		else if((return_wall_angle() >= -ANGLE_IN_FRONT) && (return_wall_angle() <= ANGLE_IN_FRONT)){		//when the robot arrives perpendicular to a wall, it turns randomly right or left
			int random_number = rand()%2;
			if(random_number == 0){
				right_motor_set_speed(-MOTOR_SPEED);
				left_motor_set_speed(MOTOR_SPEED);
				chThdSleepUntilWindowed(time, time + MS2ST(MOTOR_CONTROL_ROTA_WAIT_TIME));
			}
			else {
				right_motor_set_speed(MOTOR_SPEED);
				left_motor_set_speed(-MOTOR_SPEED);
				chThdSleepUntilWindowed(time, time + MS2ST(MOTOR_CONTROL_ROTA_WAIT_TIME));
			}
		}
		else if(return_wall_angle() > 0 ){			// wall on the left
			if((return_wall_angle() == 90) && (prox_value_delta(5) > PROX_VALUE_THRESHOLD)){	//correction of the trajectory	if wall too close
				right_motor_set_speed(-MOTOR_SPEED);
				left_motor_set_speed(MOTOR_SPEED);
			}
			right_motor_set_speed(MOTOR_SPEED - MOTOR_SPEED * (90 - return_wall_angle()));		//turning right with a speed depending on the wall angle
			left_motor_set_speed(MOTOR_SPEED + MOTOR_SPEED * (90 - return_wall_angle()));
		}
		else if(return_wall_angle() < 0){			//wall on the right
			if((return_wall_angle() == -90) && (prox_value_delta(2) > PROX_VALUE_THRESHOLD)){	//correction of the trajectory	if wall too close
				right_motor_set_speed(MOTOR_SPEED);
				left_motor_set_speed(-MOTOR_SPEED);
			}
			right_motor_set_speed(MOTOR_SPEED + MOTOR_SPEED * (90 - return_wall_angle()));		//turning left with a speed depending on the wall angle
			left_motor_set_speed(MOTOR_SPEED - MOTOR_SPEED * (90 - return_wall_angle()));
		}

    	chThdSleepUntilWindowed(time, time + MS2ST(MOTOR_CONTROL_WAIT_TIME));
    }
}



void motor_control_start(void){ 			//starts the motor control thread
	if(MotorControl_configured) return;

	motContThd = chThdCreateStatic(waMotorControl, sizeof(waMotorControl), NORMALPRIO, MotorControl, NULL);
	MotorControl_configured = 1;
}

void motor_control_stop(void){				//stops the motor control thread
    chThdTerminate(motContThd);
    chThdWait(motContThd);
    motContThd = NULL;

    MotorControl_configured = 0;

    right_motor_set_speed(0);
	left_motor_set_speed(0);
}

