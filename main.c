// le but c'est que dans le main on aient que des appels de fonctions pour décrire le match
/* exemple

    pami_init();
    wait_tirette();
    wait_85();
    GoTo(x1, y1, theta1);
    Goto(x2, y2, theta2); ..
    ...
*/
#include "motor.h"
#include "driver/gpio.h"
#include  "esp_log.h"


void app_main() {
    
    
    gpio_reset_pin(STEP_GPIO_LEFT);
    gpio_reset_pin(STEP_GPIO_RIGHT);
    gpio_reset_pin(DIR_GPIO_LEFT);
    gpio_reset_pin(DIR_GPIO_RIGHT);

    
    gpio_set_direction(STEP_GPIO_LEFT,GPIO_MODE_OUTPUT);
    gpio_set_direction(STEP_GPIO_RIGHT,GPIO_MODE_OUTPUT);
    gpio_set_direction(DIR_GPIO_LEFT,GPIO_MODE_OUTPUT);
    gpio_set_direction(DIR_GPIO_RIGHT,GPIO_MODE_OUTPUT);
    
    motor_init();
    gpio_set_level(DIR_GPIO_LEFT, 1);
    gpio_set_level(DIR_GPIO_RIGHT, 0);
    ESP_LOGI("main", " init fini");

    
    vTaskDelay(pdMS_TO_TICKS(3700));
    motor_update_speed(0.09);
    current_omega = PI/4;
    traveling(0.4);
    // motor_turn(PI);
    // traveling(0.4);
   

}
