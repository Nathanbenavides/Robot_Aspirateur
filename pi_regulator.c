#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>

#define KP 200
#define KI 0.4
#define GOAL_DIST 10.0

int16_t PI_fonction(float e);

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;

    while(1){
        time = chVTGetSystemTime();

        speed = PI_dist(get_distance_cm() - GOAL_DIST);

        if(-150 < speed && speed < 150) speed = 0;
        
        //applies the speed from the PI regulator
		right_motor_set_speed(speed);
		left_motor_set_speed(speed);

//		chprintf((BaseSequentialStream *)&SDU1, "Speed = %d\r\n", speed);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(100));
    }
}

int16_t PI_dist(float e){
	static systime_t time = 0;
	static int16_t I = 0;

	I = I + KI*(chVTGetSystemTime()-time)*e;

	if(I > 1000)	I = 1000;
	else if(I < -1000) I = -1000;

	time = chVTGetSystemTime();

//	chprintf((BaseSequentialStream *)&SDU1, "I = %d\t", I);

	return I + KP*e;
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
