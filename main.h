#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40

#define PXTOCM					1545.0f //experimental value

#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define GOAL_DISTANCE 			10.0f
#define MIN_SPEED_MOTOR			150

#define TOF_OFFSET				20

#define PI                  3.1415926536f
#define WHEEL_DISTANCE      5.35f    //cm
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     13 // [cm]

#define LOW_SPEED				200
#define TIME_WAIT_360_DEG		(PI*WHEEL_DISTANCE/WHEEL_PERIMETER*NSTEP_ONE_TURN/LOW_SPEED*1000)

#define CLEANING_TIME			(30*1000)	//30 sec
#define SEARCHING_ROTA_TIME		(TIME_WAIT_360_DEG + 1000)
#define SEARCHING_MVNT_TIME		(5*1000)	//5 sec

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
