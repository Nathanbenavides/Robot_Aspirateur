#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>

#include <pi_regulator.h>
#include <process_image.h>
#include <detect_proximity.h>

static thread_t *piThd;
static uint8_t PiRegulator_configured = 0;

//PI regulator implementation
int16_t PI_dist(float e, uint8_t init){
	static systime_t time = 0;
	static int16_t I = 0;

	if(init){
		time = chVTGetSystemTime();
		I = 0;
		return 0;
	}

	I = I + KI_dist*(chVTGetSystemTime()-time)*e;

	if(I > MAX_SUM_ERROR)	I = MAX_SUM_ERROR;
	else if(I < -MAX_SUM_ERROR) I = -MAX_SUM_ERROR;

	time = chVTGetSystemTime();

	int16_t y = I + KP_dist*e;

	return (abs(y) < MIN_SPEED_MOTOR)? 0 : y;
}

int16_t PI_pos(float e, uint8_t init){
	static systime_t time = 0;
	static int16_t I = 0;

	if(init){
		time = chVTGetSystemTime();
		I = 0;
		return 0;
	}

	I = I + KI_pos*(chVTGetSystemTime()-time)*e;

	if(I > MAX_SUM_ERROR)	I = MAX_SUM_ERROR;
	else if(I < -MAX_SUM_ERROR) I = -MAX_SUM_ERROR;

	time = chVTGetSystemTime();

	int16_t y = I + KP_pos*e;

	return (abs(y) < MIN_SPEED_MOTOR)? 0 : y;
}

static THD_WORKING_AREA(waPiRegulator, 512);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    uint16_t distance_mm = 0, line_position = 0;
    int16_t speed = 0, speed_correction = 0;

    float goal_dist = distance_value();
    PI_dist(0, 1);
    PI_pos(0, 1);

    PiRegulator_configured = 1;

    while(!chThdShouldTerminateX()){
    	time = chVTGetSystemTime();

    	distance_mm = distance_value();
    	line_position = get_line_position();

    	if((return_line_detected()==0 && goal_dist > APPROCHE_DISTANCE) || compare_front_prox()){
    		send(RESEARCH_MVNT);
    		break;
    	}

		//computes the speed to give to the motors
		speed = PI_dist(distance_mm - goal_dist, 0);
		//computes a correction factor to let the robot rotate to be in front of the line
		speed_correction = PI_pos(line_position - (IMAGE_BUFFER_SIZE/2), 0);

		if(distance_mm < APPROCHE_DISTANCE){
			speed_correction = 0;
		}

		//applies the speed from the PI regulator and the correction for the rotation
		right_motor_set_speed(speed - speed_correction);
		left_motor_set_speed(speed + speed_correction);

		if(goal_dist > MIN_DISTANCE){
			goal_dist-=DECREMENT_DIST;
		}
		else if(speed == 0 && abs(distance_mm - MIN_DISTANCE) < ERROR_MIN){
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			send(SLEEP);
		}

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(PI_REGULATOR_WAIT_TIME));
    }
}


void pi_regulator_start(void){

	if(PiRegulator_configured) return;

	piThd = chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}


void pi_regulator_stop(void){
    chThdTerminate(piThd);
    chThdWait(piThd);
    piThd = NULL;

	right_motor_set_speed(0);
	left_motor_set_speed(0);

	PiRegulator_configured = 0;
}
