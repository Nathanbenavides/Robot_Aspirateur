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
#include "proximity.h"

#include <detect_proximity.h>
#include <motor_control.h>
#include <pi_regulator.h>
#include <process_image.h>

// initialisation mutex proximity sensor
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//variable pour fsm
static thread_t *fsm;
static enum state current_state = SLEEP;

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

void Send(enum state etat){
	 (void)chMsgSend(fsm, (msg_t)etat);
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

    //proximity_start();
    messagebus_init(&bus, &bus_lock, &bus_condvar);



	//stars the threads for the pi regulator and the processing of the image
	pi_regulator_start();
	process_image_start();

    /* Infinite loop. */

    while (1) {

    	switch(current_state){
    		case SLEEP : fct_sleep(); break;
    		case EXIT : fct_exit(); break;
    		case CLEAN : fct_clean(); break;
    		case RESEARCH : fct_research(); break;
    		case PARK : fct_park();	break;
    	}


        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
