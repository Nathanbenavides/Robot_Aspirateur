#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <main.h>
#include <usbcfg.h>
#include "proximity.h"
#include "motors.h"
#include <detect_proximity.h>
#include <motor_control.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
//////////////////////////
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

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
///////////////////////////////
int main(void)
{

    halInit();
    chSysInit();
    mpu_init();
    ///////////////////
    serial_start();
    usb_start();

    //motors start
    motors_init();
    motor_control_start();
    //proximity_start();
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    proximity_start();
    detect_proximity_start();





//    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
//    proximity_msg_t prox_values;

    ///////////////////

    /* Infinite loop. */
    while (1) {


    	//waits 1 second
        chThdSleepMilliseconds(100);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
