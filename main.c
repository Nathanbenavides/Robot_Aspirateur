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
#include <line_research.h>

// initialisation mutex proximity sensor
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//variable pour fsm
static thread_t *fsmThd;
static enum state current_state = SLEEP;

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

//to send message from the thread to the FSM
void send(enum state etat){
	(void)chMsgSend(fsmThd, (msg_t)(etat+MESSAGE_FSM_SHIFT));
}

//wait on the FSM to have message from the threat
void receive(){
    thread_t *tp = chMsgWait();
    msg_t msg = chMsgGet(tp);
    current_state = msg-MESSAGE_FSM_SHIFT;
    chMsgRelease(tp, MSG_OK);
}

//sub function of the FSM
void fct_sleep(void){
	uint8_t selector = get_selector();

	set_body_led(1);

	do{
		chThdSleepMilliseconds(SLEEP_WAIT_TIME);
	}while(selector == get_selector());

	set_body_led(0);
	chThdSleepMilliseconds(START_WAIT_TIME);
	current_state = EXIT;
}

void fct_exit(void){
	left_motor_set_speed(-LOW_SPEED);
	right_motor_set_speed(-LOW_SPEED);

	chThdSleepMilliseconds(GO_BACK_TIME);

	left_motor_set_speed(-LOW_SPEED);
	right_motor_set_speed(LOW_SPEED);

	chThdSleepMilliseconds(TIME_WAIT_360_DEG/2);

	left_motor_set_speed(0);
	right_motor_set_speed(0);

	current_state = CLEAN;
}

void fct_clean(void){
	systime_t time = chVTGetSystemTime();

	motor_control_start();
	chThdSleepUntilWindowed(time, time + S2ST(TIME_WAIT_CLEANING));
	motor_control_stop();
	current_state = RESEARCH_ROTA;
}

void fct_research_mvnt(void){
	systime_t time = chVTGetSystemTime();

	motor_control_start();
	chThdSleepUntilWindowed(time, time + S2ST(TIME_WAIT_SEARCHING_MVNT));
	motor_control_stop();
	current_state = RESEARCH_ROTA;
}

void fct_research_rota(void){
	find_line_start();
	receive();
	find_line_stop();
}

void fct_park(void){
	pi_regulator_start();
	receive();
	pi_regulator_stop();
}


//threat of the FSM
static THD_WORKING_AREA(waMainFSM, 512);
static THD_FUNCTION(MainFSM, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
    	switch(current_state){
			case SLEEP : fct_sleep(); break;
			case EXIT : fct_exit(); break;
			case CLEAN : fct_clean(); break;
			case RESEARCH_MVNT : fct_research_mvnt(); break;
			case RESEARCH_ROTA : fct_research_rota(); break;
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

	//init. of the messagebus used by the proximiy thread
    messagebus_init(&bus, &bus_lock, &bus_condvar);

	//starts the threads for the processing of the image
	process_image_start();
	//starts the treads for getting prox. sensors values and analysing them
	proximity_start();
	detect_proximity_start();

	//start of the FSM
    fsmThd = chThdCreateStatic(waMainFSM, sizeof(waMainFSM), NORMALPRIO, MainFSM, NULL);

    /* Infinite loop. Where we do nothing*/
    while (1) {
        chThdSleepMilliseconds(MAIN_WAIT_TIME);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
