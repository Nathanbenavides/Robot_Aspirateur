#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>

#include <pi_regulator.h>
#include <process_image.h>

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

void fct_sleep(){

}
void fct_exit(){

}
void fct_clean(){

}
void fct_research(){

}
void fct_park(){

}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	//inits the motors
	motors_init();

	//stars the threads for the pi regulator and the processing of the image
	pi_regulator_start();
	process_image_start();

    /* Infinite loop. */

	enum state current_state = SLEEP;
	enum state last_state = SLEEP;

    while (1) {

    	switch(current_state){
    	case SLEEP :
    		{
    		if(current_state != last_state) fct_sleep();
    		}
    	    	break;
    	case EXIT :
			{
			if(current_state != last_state) fct_exit();
			}
		break;
		case CLEAN :
			{
			if(current_state != last_state) fct_clean();
			}
		break;
    	case RESEARCH :
			{
			if(current_state != last_state) fct_research();
			}
		break;
    	case PARK :
			{
			if(current_state != last_state) fct_park();
			}
		break;
    	}

    	last_stat = current_state;

        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
