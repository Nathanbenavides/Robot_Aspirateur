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
#include <sensors/proximity.h>
#include <selector.h>
#include <leds.h>

#include <detect_proximity.h>
#include <motor_control.h>
#include <pi_regulator.h>
#include <process_image.h>

#define GO_BACK_TIME 3000

// initialisation mutex proximity sensor
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//variable pour fsm
static thread_t *fsmThd;
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

void send(enum state etat){
	(void)chMsgSend(fsmThd, (msg_t)(etat+1));
}

void receive(){
    thread_t *tp = chMsgWait();
    msg_t msg = chMsgGet(tp);
    current_state = msg-1;
}

void fct_sleep(){
	uint8_t selector = get_selector();
	set_body_led(1);
	do{
		chThdSleepMilliseconds(500);
	}while(selector == get_selector());
	set_body_led(0);
	chThdSleepMilliseconds(1000);
	current_state = EXIT;
}
void fct_exit(){
	left_motor_set_speed(-LOW_SPEED);
	right_motor_set_speed(-LOW_SPEED);
	chThdSleepMilliseconds(GO_BACK_TIME);
	left_motor_set_speed(-LOW_SPEED);
	right_motor_set_speed(LOW_SPEED);
	chThdSleepMilliseconds(TIME_WAIT_360_DEG/2);
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	current_state = SLEEP;

}
void fct_clean(){

}
void fct_research(){

}
void fct_park(){

}


static THD_WORKING_AREA(waMainFSM, 256);
static THD_FUNCTION(MainFSM, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;



    while(1){
    	switch(current_state){
			case SLEEP : fct_sleep(); break;
			case EXIT : fct_exit(); break;
			case CLEAN : fct_clean(); break;
			case RESEARCH : fct_research(); break;
			case PARK : fct_park();	break;
    	}
    }
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
	//pi_regulator_start();
	//process_image_start();

    fsmThd = chThdCreateStatic(waMainFSM, sizeof(waMainFSM), NORMALPRIO, MainFSM, NULL);

    /* Infinite loop. */

    while (1) {
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
