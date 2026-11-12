// Contient tout le necessaire pour le déplacement du PAMI

// 1. déplacement en boucle ouverte
// 2. déplacement en boucle fermée (aservissement grace à asser.c)
// 3. devra contenir une fonction GoTo(float x, float y, float theta) pour le déplacement absolu
// et travel(float dist), rotate(float theta) pour le déplacement en relatif

#include "motor.h"



// ensuite il faut inclure les bibliothèques de gestion des impulsions pour le moteur


rmt_symbol_word_t pulseRight={
        .level0 = 1, .duration0=UP_DURATION,
        .level1 = 0, .duration1 = 500
    };

rmt_symbol_word_t pulseLeft={
        .level0 = 1, .duration0=UP_DURATION,
        .level1 = 0, .duration1 = 500
    };

static rmt_channel_handle_t tx_chan_left = NULL;
static rmt_encoder_handle_t copy_enc_left = NULL;

static rmt_channel_handle_t tx_chan_right = NULL;
static rmt_encoder_handle_t copy_enc_right = NULL;

float current_velocity_L = 0.0;
float current_velocity_R = 0.0;
float current_omega = 0.0;

float coef_rotation = 0.78;
float coef_linear = 1.10;


// initialisation des moteurs
void motor_init(){
    rmt_tx_channel_config_t chan_cfg_left ={
        .gpio_num = STEP_GPIO_LEFT ,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RES_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,

    };  
    rmt_tx_channel_config_t chan_cfg_right ={
        .gpio_num = STEP_GPIO_RIGHT ,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RES_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,

    };  

    rmt_new_tx_channel(&chan_cfg_left, &tx_chan_left);
    rmt_copy_encoder_config_t enc_cfg_left = {};
    rmt_new_copy_encoder(&enc_cfg_left, &copy_enc_left );
    rmt_enable(tx_chan_left);

    
    rmt_new_tx_channel(&chan_cfg_right, &tx_chan_right);
    rmt_copy_encoder_config_t enc_cfg_right = {};
    rmt_new_copy_encoder(&enc_cfg_right, &copy_enc_right);
    rmt_enable(tx_chan_right);
}

void start_step_left (){
    rmt_transmit_config_t tx_cfg_left = {
        .loop_count = -1,  // pour une boucle infinie
        .flags.eot_level = 0
    };
    rmt_transmit(tx_chan_left,copy_enc_left, &pulseLeft, sizeof(pulseLeft),&tx_cfg_left);
}

void start_step_right (){
    rmt_transmit_config_t tx_cfg_right = {
        .loop_count = -1,
        .flags.eot_level = 0
    };
    rmt_transmit(tx_chan_right,copy_enc_right, &pulseRight, sizeof(pulseRight),&tx_cfg_right);
}


float ms_to_pd(float v){
    float period = RMT_RES_HZ*(WHEEL_RADIUS*2*PI)/(STEPPER_MICROSTEPS*v) ;
    return period;
}



float pd_to_ms(float period){
    float v = WHEEL_RADIUS*2*PI/(STEPPER_MICROSTEPS*((period)/RMT_RES_HZ));
    return v;
}




//  Tout ce qui est gestion des directions de rotation des roues
// faire gaffe à gérer séparemment les roues gauches des roues droites
void motor_set_direction_L(dir direction){
    if (direction == Forward) {
        gpio_set_level(DIR_GPIO_LEFT, 0);
    }
    else{
        gpio_set_level(DIR_GPIO_LEFT, 1);
    }
    return;
}

void motor_set_direction_R(dir direction){
    if (direction == Forward) {
        gpio_set_level(DIR_GPIO_RIGHT, 1);
    }
    else{
        gpio_set_level(DIR_GPIO_RIGHT, 0);
    }
    return;
}

void motor_set_direction(dir direction){ // gérer la direction du PAMI global
    motor_set_direction_L(direction);
    motor_set_direction_R(direction);
    return;
}

//un pid se fait avec une commande en vitesse, donc dans la suite on va définir des fonction pour modifier les vitesses de rotation de chaque roue

void motor_set_speed_L(float v){
    if (current_velocity_L != v){
        motor_stop_L();
        pulseLeft.duration1 = ms_to_pd(v);
        start_step_left();
        current_velocity_L = v;
    }
    return;
}

void motor_set_speed_R(float v){
    if (current_velocity_R != v){
        motor_stop_R();
        pulseRight.duration1 = (int)(ms_to_pd(v));
        start_step_right();
        current_velocity_R = v;
    }
    return;
}

void motor_update_speed(float v){ // au cas où on a besoin d'update la vitesse sans stopper les moteurs
    if (current_velocity_L != v && current_velocity_R !=v){
        motor_set_speed_L(v);
        motor_set_speed_R(v);
        current_velocity_L = current_velocity_R = v;
    }

}

void motor_stop_L(){
    if (current_velocity_L != 0){
        rmt_disable(tx_chan_left);
        rmt_enable(tx_chan_left);
        current_velocity_L = 0;
    }
    return;
}

void motor_stop_R(){
    if (current_velocity_R !=0){
        rmt_disable(tx_chan_right);
        rmt_enable(tx_chan_right);
        current_velocity_R=0;
    }
    return;
}

void motor_stops(){
    motor_stop_R();
    motor_stop_L();
}


void set_rotation_motor_velocity(float omega){
     if (omega > 0){
        motor_set_direction_L(Backward);
        motor_set_direction_R(Forward);
    }
    else {
        motor_set_direction_L(Forward);
        motor_set_direction_R(Backward);
    }
    motor_set_speed_R(fabsf(omega)*coef_rotation*WHEELS_HALF_DISTANCE);
    motor_set_speed_L(fabsf(omega)*coef_rotation*WHEELS_HALF_DISTANCE);
    current_omega = omega;
}

void motor_turn(float theta){
    if (theta ==0){return;}
    if (theta <0){
        current_omega = -fabsf(current_omega);
    }
    else{
        current_omega = fabsf(current_omega);
    }
    set_rotation_motor_velocity(current_omega);
    vTaskDelay(pdMS_TO_TICKS(fabsf(1000*theta/(current_omega))));
    motor_stops();
}



void traveling(float distance){
    if (distance == 0) return;
    if (distance < 0){
        motor_set_direction_L(Forward);
        motor_set_direction_R(Forward);
    }
    else {
        motor_set_direction_L(Backward);
        motor_set_direction_R(Backward);
    }

    if (current_velocity_L >0 ){
        motor_update_speed(current_velocity_L);
        vTaskDelay(pdMS_TO_TICKS(1000*coef_linear*(distance)/current_velocity_L));
        motor_stops();
    }
    else{
        motor_update_speed(0.05);
        vTaskDelay(pdMS_TO_TICKS(1000*distance/(0.05)));
        motor_stops();
    }

}


// faudrait aussi pouvoir modifier les accelerations indépendamment
void motor_set_accel_L(float accel){
    return;
}

// ... à compléter, mais là c'est suffisant pour une boucle ouverte. 

