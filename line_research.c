#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <line_research.h>
#include <process_image.h>


static THD_WORKING_AREA(waLineResearch, 256);
static THD_FUNCTION(LineResearch, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;


    while(1){
    	time = chVTGetSystemTime();


        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

