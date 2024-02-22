#ifndef CONTROL
#define CONTROL

void SystemClock_Config(void);
// static void MX_GPIO_Init(void);
// static void MX_TIM2_Init(void);

void setup_control();
void start_pwm();
void stop_pwm();
void set_phase_shift();

#endif