#ifndef DETECT_PROXIMITY_H
#define DETECT_PROXIMITY_H

void detect_proximity_start(void);	//to start the thread
void detect_proximity_stop(void);	//to stop the thread
int return_wall_angle(void);		//return a approximation of the wall angle
bool return_wall_detected(void);
int prox_value_delta(uint8_t sensor);
float distance_value(void);
bool compare_front_prox(void);

#endif /* DETECT_PROXIMITY_H */
