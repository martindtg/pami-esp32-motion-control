#ifndef SRC_ROBOT_TYPES_H_
#define SRC_ROBOT_TYPES_H_

#include <stdint.h>

#ifndef PI
#define PI 3.14159265f
#endif

#define WHEEL_RADIUS 0.025f  
#define WHEELS_HALF_DISTANCE 0.06f



typedef struct {int32_t right, left;} Wheel_ticks;
typedef struct {float right, left;} Wheel_speed; // m/s
typedef enum {ASSERV_IDLE, ASSERV_RUNNING, ASSERV_DONE} Asserv_status;

typedef struct {Wheel_speed cmd; Asserv_status status; } Asserv_t;
typedef struct{ float x, y, theta; } Pose; // m, m, rad (-pi, pi)

#endif 