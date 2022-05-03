#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <pi_regulator.h>
#include <process_image.h>

#include <leds.h>

#define KP_dist						30
#define KI_dist						0.1	//must not be zero

#define KP_pos						2
#define KI_pos 						0	//must not be zero

#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT)

#define ERROR_MIN				0.5f

#define MIN_DISTANCE			20.0f

static thread_t *piThd;
static uint8_t PiRegulator_configured = 0;

//simple PI regulator implementation
int16_t PI_dist(float e, uint8_t init){
	static systime_t time = 0;
	static int16_t I = 0;

	if(init == 1){
		time = chVTGetSystemTime();
		I = 0;
		return 0;
	}

	I = I + KI_dist*(chVTGetSystemTime()-time)*e;

	if(I > MAX_SUM_ERROR)	I = MAX_SUM_ERROR;
	else if(I < -MAX_SUM_ERROR) I = -MAX_SUM_ERROR;

	time = chVTGetSystemTime();

	int16_t y = I + KP_dist*e;

	return (y < 150 && -150 < y)? 0 : y;
}

int16_t PI_pos(float e, uint8_t init){
	static systime_t time = 0;
	static int16_t I = 0;

	if(init == 1){
		time = chVTGetSystemTime();
		I = 0;
		return 0;
	}

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

    float goal_dist = 100;

    PI_dist(0, 1);
    PI_pos(0, 1);

    while(1){
    	time = chVTGetSystemTime();

    	uint16_t distance_mm = VL53L0X_get_dist_mm() - TOF_OFFSET;
    	uint16_t line_position = get_line_position();

		//computes the speed to give to the motors
		//distance_cm is modified by the image processing thread
		speed = PI_dist(distance_mm - goal_dist, 0);
		//computes a correction factor to let the robot rotate to be in front of the line
		speed_correction = PI_pos(line_position - (IMAGE_BUFFER_SIZE/2), 0);

		if(-MIN_SPEED_MOTOR < speed && speed < MIN_SPEED_MOTOR) speed = 0;
		if(-MIN_SPEED_MOTOR < speed_correction && speed_correction < MIN_SPEED_MOTOR) speed_correction = 0;

//		chprintf((BaseSequentialStream *)&SDU1, "Pos = %d dist = %d goal = %.2f\r\n", line_position, distance_mm, goal_dist);

		//applies the speed from the PI regulator and the correction for the rotation
		right_motor_set_speed(speed - speed_correction);
		left_motor_set_speed(speed + speed_correction);

		if(fabs(distance_mm - goal_dist) < ERROR_MIN && goal_dist > MIN_DISTANCE && speed == 0) goal_dist-=5;

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}


void pi_regulator_start(void){

	if(PiRegulator_configured==1) return;

	piThd = chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
	VL53L0X_start();

	PiRegulator_configured = 1;
}


void pi_regulator_stop(void){
    chThdTerminate(piThd);
    chThdWait(piThd);
    piThd = NULL;

	VL53L0X_stop();

	PiRegulator_configured = 0;
}
