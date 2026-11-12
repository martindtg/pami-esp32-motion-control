#ifndef MOTOR_H_
#define MOTOR_H_

#include "config.h"
#include "utils.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "robot_types.h"
#include "driver/gpio.h"
#include "math.h"


extern rmt_symbol_word_t pulseRight;

extern rmt_symbol_word_t pulseLeft;


#define STEP_GPIO_LEFT 11    
#define STEP_GPIO_RIGHT 7  
#define DIR_GPIO_LEFT 10  
#define DIR_GPIO_RIGHT 8  
#define UP_DURATION 50
#define  STEPPER_MICROSTEPS  1600
#define ANGLE_PER_STEP 2*PI*WHEEL_RADIUS/(STEPPER_MICROSTEPS*WHEELS_HALF_DISTANCE)
#define TURN_SPEED 0.05
#define DISTANCE_PER_STEP ((2*PI*WHEEL_RADIUS)/(STEPPER_MICROSTEPS))


extern float current_velocity_L;
extern float current_velocity_R;
extern float current_omega;

#define RMT_RES_HZ 1000000
void motor_init();
void start_step_left ();
void start_step_right ();


void motor_set_direction_L(dir direction);
void motor_set_direction_R(dir direction);
void motor_set_direction(dir direction);

void motor_set_speed_L(float v);
void motor_set_speed_R(float v);

void motor_update_speed(float v);

void motor_stop_L();
void motor_stop_R();
void motor_stops();
void motor_set_accel_L(float accel);

void set_rotation_motor_velocity(float w);

void motor_turn(float theta);
void traveling(float distance);
#endif