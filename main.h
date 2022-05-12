#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif
//////////////////

///////////////////
#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			60

#define PXTOCM					1545.0f //experimental value

#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define GOAL_DISTANCE 			10.0f
#define MIN_SPEED_MOTOR			150

//TOF Value
#define TOF_SLOP				0.8733f
#define TOF_OFFSET				(-4.0459f)
#define TOF_SMALL_DIST			35
#define TOF_CORRECTION(value)	(TOF_SLOP*value+TOF_OFFSET)

//Proximity Value
#define PROX_SLOP_BIG				(-0.0041f)
#define PROX_OFFSET_BIG				(14.487f)
#define PROX_SLOP_SMALL				(-0.067f)
#define PROX_OFFSET_SMALL			(40.54f)
#define PROX_THRESHOLD				420


#define PI                  	3.1415926536f
#define WHEEL_DISTANCE			5.35f    //cm
#define NSTEP_ONE_TURN			1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER			13 // [cm]

#define LOW_SPEED				200
#define TIME_WAIT_360_DEG		(PI*WHEEL_DISTANCE/WHEEL_PERIMETER*NSTEP_ONE_TURN/LOW_SPEED*1000)

#define TIME_WAIT_CLEANING			30	//sec
#define TIME_WAIT_SEARCHING_ROTA	(TIME_WAIT_360_DEG + 1000)
#define TIME_WAIT_SEARCHING_MVNT	3	//sec

#define GO_BACK_TIME 			3000



enum state {SLEEP, EXIT, CLEAN, RESEARCH_MVNT, RESEARCH_ROTA, PARK};

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);
void send(enum state etat);
void receive(void);

#ifdef __cplusplus
}
#endif

#endif
