#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

void motor_control_start(void);
void motor_control_stop(void);
void set_motor_control_active(bool set);
void set_time_running(systime_t set);

#endif /* MOTOR_CONTROL_H */
