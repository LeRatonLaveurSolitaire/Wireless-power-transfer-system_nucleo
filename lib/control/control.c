#include <stdint.h>
#include "stm32g4xx_hal.h"
#include <stdbool.h>


#define PWM1 10
#define PWM2 11

TIM_HandleTypeDef htim2;

uint8_t PHASE_SHIFT = 0;

void setup_control(){
    HAL_Init();
      GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void start_pwm(){
}

void stop_pwm(){

}

void IRQ_PWM1(){

}

void IRQ_PWM2(){

}

void set_phase_shift(){

}

