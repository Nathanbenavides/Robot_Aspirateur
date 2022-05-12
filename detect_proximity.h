#ifndef DETECT_PROXIMITY_H
#define DETECT_PROXIMITY_H

void detect_proximity_start(void);
void detect_proximity_stop(void);
int return_wall_angle(void);
bool return_wall_detected(void);
int prox_value_delta(uint8_t sensor);
float distance_value(void);

#endif /* DETECT_PROXIMITY_H */
