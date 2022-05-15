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


//constants for the different parts of the project

/****					Constants detect_proximity							****/
#define DETECT_PROXIMITY_WAIT_TIME		150

#define PROXIMITY_COLISION_THRESHOLD	300
#define PROXIMITY_FRONT_THRESHOLD		200
#define PROXIMITY_FRONT_DELTA			100
#define PROXIMITY_ANGLE_1				-10	//deg
#define PROXIMITY_ANGLE_2				-45	//deg
#define PROXIMITY_ANGLE_3				-90	//deg
#define PROXIMITY_ANGLE_4				-135//deg
#define PROXIMITY_ANGLE_5				135	//deg
#define PROXIMITY_ANGLE_6				90	//deg
#define PROXIMITY_ANGLE_7				45	//deg
#define PROXIMITY_ANGLE_8				10	//deg
//Approximation of the distance with proximity on black surface
#define PROX_SLOP_BIG					(-0.0041f)
#define PROX_OFFSET_BIG					(14.487f)
#define PROX_SLOP_SMALL					(-0.067f)
#define PROX_OFFSET_SMALL				(40.54f)
#define PROX_THRESHOLD					420
//Approximation of the distance with the TOF sensor
#define TOF_SLOP						0.8733f
#define TOF_OFFSET						(-4.0459f)
#define TOF_LIMIT_DIST					35
#define TOF_CORRECTION(value)			(TOF_SLOP*value+TOF_OFFSET)


/****					Constants line_reasearch							****/
#define LINE_RESEARCH_WAIT_TIME			100		//ms
#define TIME_WAIT_SEARCHING_ROTA		(TIME_WAIT_360_DEG + 1000)	//ms


/****							Constants main								****/
#define MAIN_WAIT_TIME					1000	//ms
#define MESSAGE_FSM_SHIFT				1		//ms
//fsm state
enum state {SLEEP, EXIT, CLEAN, RESEARCH_MVNT, RESEARCH_ROTA, PARK};
//sleep
#define SLEEP_WAIT_TIME					500		//ms
#define START_WAIT_TIME					1000	//ms
//exit
#define GO_BACK_TIME 					3000	//ms
//clean
#define TIME_WAIT_CLEANING				30		//sec
//research_mvnt
#define TIME_WAIT_SEARCHING_MVNT		3		//sec


/****						Constants motor_control							****/
#define MOTOR_CONTROL_ROTA_WAIT_TIME	500
#define MOTOR_CONTROL_WAIT_TIME			20
#define MOTOR_SPEED						(MOTOR_SPEED_LIMIT/2)
#define ANGLE_BEHIND					90		//deg
#define ANGLE_IN_FRONT					10		//deg
#define PROX_VALUE_THRESHOLD			150

/****						Constants pi_regulator							****/
#define PI_REGULATOR_WAIT_TIME			10 //ms so 100Hz
//coef pi distance
#define KP_dist							15
#define KI_dist 						0.03f
//coef pi position de la ligne
#define KP_pos							2
#define KI_pos							0.005f
//coeff a tout le système
#define MAX_SUM_ERROR 					(MOTOR_SPEED_LIMIT*0.8)
#define ERROR_MIN						4		//mm
#define APPROCHE_DISTANCE				60.0f
#define MIN_DISTANCE					10.0f
#define DECREMENT_DIST					0.3


/****						Constants process_image							****/
//image capture
#define IMAGE_BUFFER_SIZE				640
#define LINE_NUMBER						200
#define NUMBER_OF_LINE					2	//min 2
//line detection
#define WIDTH_SLOPE						5
#define MIN_LINE_WIDTH					60
#define PXTOCM							1545.0f //experimental value
#define MAX_DISTANCE 					25.0f
#define BLACK_THRESHOLD					30


/****							Global Constants							****/
#define MIN_SPEED_MOTOR					150
#define LOW_SPEED						200

#define PI                  			3.1415926536f
#define WHEEL_DISTANCE					5.35f    //cm
#define NSTEP_ONE_TURN					1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER					13 // [cm]
#define TIME_WAIT_360_DEG				(PI*WHEEL_DISTANCE/WHEEL_PERIMETER*NSTEP_ONE_TURN/LOW_SPEED*1000)


/****							Robot wide IPC bus.							****/
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;


/****							FSM function								****/
void send(enum state etat);
void receive(void);

#ifdef __cplusplus
}
#endif

#endif
