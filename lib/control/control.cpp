#include <stdint.h>

#define PWM1 10
#define PWM2 11

uint8_t PWM1_STATE = 0;
uint8_t PWM2_STATE = 0;
uint8_t PHASE_SHIFT = 0;

void setup_control(){

}

void start_pwm(){
}

void stop_pwm(){
    
}

void IRQ_PWM1(){
    PWM1_STATE = !PWM1_STATE;
    (PWM1,PWM1_STATE); 
}

void IRQ_PWM2(){
    PWM2_STATE = !PWM2_STATE;
    (PWM2,PWM2_STATE);
}

void set_phase_shift(){

}