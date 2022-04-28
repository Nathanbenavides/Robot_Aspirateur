#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>

#define KP_dist						200
#define KI_dist						0.4	//must not be zero
#define KP_pos						800.0f
#define KI_pos 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT)

//simple PI regulator implementation
int16_t PI_dist(float e){
	static systime_t time = 0;
	static int16_t I = 0;

	I = I + KI_dist*(chVTGetSystemTime()-time)*e;

	if(I > MAX_SUM_ERROR)	I = MAX_SUM_ERROR;
	else if(I < -MAX_SUM_ERROR) I = -MAX_SUM_ERROR;

	time = chVTGetSystemTime();

	int16_t y = I + KP_pos*e;

	return (y < 150 && -150 < y)? 0 : y;
}

int16_t PI_pos(float e){
	static systime_t time = 0;
	static int16_t I = 0;

	I = I + KI_pos*(chVTGetSystemTime()-time)*e;

	if(I > MAX_SUM_ERROR)	I = MAX_SUM_ERROR;
	else if(I < -MAX_SUM_ERROR) I = -MAX_SUM_ERROR;

	time = chVTGetSystemTime();

	int16_t y = I + KP_pos*e;

	return (y < 150 && -150 < y)? 0 : y;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();
        
        //computes the speed to give to the motors
        //distance_cm is modified by the image processing thread
        speed = PI_dist(get_distance_cm() - GOAL_DISTANCE);
        //computes a correction factor to let the robot rotate to be in front of the line
        speed_correction = PI_pos(get_line_position() - (IMAGE_BUFFER_SIZE/2));

        //if the line is nearly in front of the camera, don't rotate
        if(abs(speed_correction) < ROTATION_THRESHOLD){
        	speed_correction = 0;
        }

        //applies the speed from the PI regulator and the correction for the rotation
		right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
		left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
