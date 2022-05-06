#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <motor_control.h>
#include <line_research.h>
#include <process_image.h>

static thread_t *searchThd;
static bool search_line = 0;
static bool line_found = 0;
static THD_WORKING_AREA(waFindLine, 256);
static THD_FUNCTION(FindLine, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    systime_t time = chVTGetSystemTime();


    while(1){ //faire un bool modifié par motor_control pour dire quand il cherche la ligne
    	if (search_line){
    		set_motor_control_active(0);	//stops random movements

			systime_t time_search = chVTGetSystemTime();

//			motor_control_stop();
			while(!((time_search + MS2ST(5000)) < chVTGetSystemTime())){ //searching a line for 5000ms

				right_motor_set_speed(MOTOR_SPEED_LIMIT/4);
				left_motor_set_speed(-MOTOR_SPEED_LIMIT/4);
				if(return_line_detected()){
					palTogglePad(GPIOD, GPIOD_LED1);	//activates LED if it finds a line
					line_found = 1;
					break;
				}
				chThdSleepUntilWindowed(time, time + MS2ST(100));
			}
			if(line_found){					//ligne trouvée, à remplir pour lancer le PID
				search_line = 0;			//arrête de chercher la ligne
				left_motor_set_speed(0);
			}
			else{
				search_line = 0;				//stops searching line
				set_time_running(5000);  		//random movements for x mx
				set_motor_control_active(1);	//starts random movements
			}
    	}
    	else{
    	chThdSleepMilliseconds(200);
    	}
    }

}

void find_line_start(void){
	searchThd = chThdCreateStatic(waFindLine, sizeof(waFindLine), NORMALPRIO, FindLine, NULL);
}

void set_search_line(bool set_search_line){
	search_line = set_search_line;
	return;
}


