#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <main.h>
#include <sensors/proximity.h>
#include <detect_proximity.h>

static thread_t *tp;
static proximity_msg_t prox_values;
static int wall_angle = 0;
static bool wall_detected = 0;
bool return_wall_detected(void);
void approximate_wall_angle(void);


static THD_WORKING_AREA(waDetectProximity, 256);
static THD_FUNCTION(DetectProximity, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;


    while(!chThdShouldTerminateX()){

    	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

    	// Sensors info print: each line contains data related to a single sensor.
//    	for (uint8_t i = 0; i < sizeof(prox_values.ambient)/sizeof(prox_values.ambient[0]); i++) {
//    		chprintf((BaseSequentialStream *)&SDU1, "%4d,", prox_values.ambient[i]);
//    		chprintf((BaseSequentialStream *)&SDU1, "%4d,", prox_values.reflected[i]);
//    		chprintf((BaseSequentialStream *)&SDU1, "%4d", prox_values.delta[i]);
//    		chprintf((BaseSequentialStream *)&SDU1, "\r\n");
//    	}
//    	chprintf((BaseSequentialStream *)&SDU1, "\r\n");
    	chprintf((BaseSequentialStream *)&SDU1, "%4d", prox_values.delta[0]);
    	chprintf((BaseSequentialStream *)&SDU1, "%4d", prox_values.delta[7]);
    	chprintf((BaseSequentialStream *)&SDU1, "\r\n");
    	approximate_wall_angle();

//    	chprintf((BaseSequentialStream *)&SDU1, "%4d", wall_angle);
//    	palTogglePad(GPIOD, GPIOD_LED1);


    	chThdSleepMilliseconds(150);
    }
}



void detect_proximity_start(void){
	tp = chThdCreateStatic(waDetectProximity, sizeof(waDetectProximity), NORMALPRIO+1, DetectProximity, NULL);
}

void detect_proximity_stop(void){
	chThdTerminate(tp);
}

void approximate_wall_angle(void){
	bool sensor_see_wall[8];
	uint8_t closest_sensor = 0;
	uint16_t closest_sensor_delta = 0;

	for (uint8_t i = 0; i < 8 ; ++i){
		if(prox_values.delta[i] > 300){  //wall distance detection (big delta = close wall)
			sensor_see_wall[i] = 1;
		}
		else sensor_see_wall[i] = 0;
	}
	for (uint8_t i = 0; i < 8 ; ++i){
		if(sensor_see_wall[i] == 1){
			wall_detected = 1;
		}
	}
	if(!sensor_see_wall[0] && !sensor_see_wall[1] && !sensor_see_wall[2] &&
			!sensor_see_wall[3] && !sensor_see_wall[4] && !sensor_see_wall[5] &&
			!sensor_see_wall[6] && !sensor_see_wall[7]){
		wall_detected = 0;
	}

	for (uint8_t i = 0; i < 8 ; ++i){
		if(sensor_see_wall[i]){
			if(prox_values.delta[i] > closest_sensor_delta){
				closest_sensor_delta = prox_values.delta[i];
				closest_sensor = i;
			}
		}
	}

	if((prox_values.delta[0] < prox_values.delta[7] + 100)
			&& (prox_values.delta[0] > prox_values.delta[7] - 100)
			&& (prox_values.delta[0] > 200)){
		wall_angle = 0;
		return;
	}
	switch (closest_sensor) {
		case 0:
			wall_angle = -10;	//front right 1
			return;
		case 1:
			wall_angle = -45;	//front right 2
			return;
		case 2:
			wall_angle = -90;	//right
			return;
		case 3:
			wall_angle = -135;	//back right
			return;
		case 4:
			wall_angle = 135;	//back left
			return;
		case 5:
			wall_angle = 90;	//left
			return;
		case 6:
			wall_angle = 45;	//front left 2
			return;
		case 7:
			wall_angle = 10;	//front left 1
			return;

	}

	return;
}


int return_wall_angle(void){
	return wall_angle;
}

bool return_wall_detected(void){
	return wall_detected;
}

int prox_value_delta(uint8_t sensor){
return prox_values.delta[sensor];
}


