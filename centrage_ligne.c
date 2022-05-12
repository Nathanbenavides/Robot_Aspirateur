#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <motors.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <centrage_ligne.h>

static thread_t *centreLigneThd;
static uint8_t centreLigne_configured = 0;

static THD_WORKING_AREA(waCentreLigne, 256);
static THD_FUNCTION(CentreLigne, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    int8_t rotation_sense = 0; // 0 sens horaire et 1 sens antihoraire

    chThdSleepMilliseconds(500);

    int16_t min_dist = VL53L0X_get_dist_mm()/10, time_start = chVTGetSystemTime();

    while(!chThdShouldTerminateX() || rotation_sense == 0){
    	right_motor_set_speed(LOW_SPEED);
		left_motor_set_speed(-LOW_SPEED);

    }
    while(!chThdShouldTerminateX() || rotation_sense == 1){
        	right_motor_set_speed(-LOW_SPEED);
    		left_motor_set_speed(LOW_SPEED);

    }
}



void detect_proximity_start(void){
	if(centreLigne_configured) return;

	centreLigneThd = chThdCreateStatic(waDetectProximity, sizeof(waDetectProximity), NORMALPRIO+1, DetectProximity, NULL);
	centreLigne_configured = 1;

	VL53L0X_start();
}

void detect_proximity_stop(void){
    chThdTerminate(centreLigneThd);
    chThdWait(centreLigneThd);
    centreLigneThd = NULL;

	centreLigne_configured = 0;

	VL53L0X_stop();

}


