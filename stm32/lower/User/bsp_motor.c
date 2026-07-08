#include "bsp_motor.h"
#include "app_config.h"

#define LEFT_IN1_PIN    GPIO_Pin_13
#define LEFT_IN2_PIN    GPIO_Pin_12
#define RIGHT_IN1_PIN   GPIO_Pin_15
#define RIGHT_IN2_PIN   GPIO_Pin_14
#define MOTOR_DIR_PORT  GPIOB
#define MOTOR_STBY_PIN  GPIO_Pin_5
#define MOTOR_STBY_PORT GPIOB

static uint16_t abs_limit_pwm(int16_t pwm)
{
    int32_t v = pwm;
    if (v < 0) {
        v = -v;
    }
    if (v > MOTOR_PWM_PERIOD) {
        v = MOTOR_PWM_PERIOD;
    }
    return (uint16_t)v;
}

static uint16_t pwm_compare_for_cmd(int16_t pwm)
{
    if (pwm == 0) {
        /* TB6612: IN1=IN2=0 with PWM high is stop/high-Z, not short brake. */
        return MOTOR_PWM_PERIOD;
    }
    return abs_limit_pwm(pwm);
}

static void set_dir(GPIO_TypeDef *port, uint16_t in1, uint16_t in2, int16_t pwm)
{
    if (pwm > 0) {
        GPIO_SetBits(port, in1);
        GPIO_ResetBits(port, in2);
    } else if (pwm < 0) {
        GPIO_ResetBits(port, in1);
        GPIO_SetBits(port, in2);
    } else {
        GPIO_ResetBits(port, in1 | in2);
    }
}

static void set_coast(GPIO_TypeDef *port, uint16_t in1, uint16_t in2)
{
    GPIO_ResetBits(port, in1 | in2);
}

static void set_brake(GPIO_TypeDef *port, uint16_t in1, uint16_t in2)
{
    GPIO_SetBits(port, in1 | in2);
}

static void motor_enable_driver(void)
{
#if MOTOR_TB6612_STBY_GPIO
    GPIO_SetBits(MOTOR_STBY_PORT, MOTOR_STBY_PIN);
#endif
}

static void motor_disable_driver(void)
{
#if MOTOR_TB6612_STBY_GPIO
    GPIO_ResetBits(MOTOR_STBY_PORT, MOTOR_STBY_PIN);
#endif
}

void Motor_BspInit(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef oc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    GPIO_StructInit(&gpio);

#if MOTOR_TB6612_STBY_GPIO
    gpio.GPIO_Pin = MOTOR_STBY_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTOR_STBY_PORT, &gpio);
    GPIO_ResetBits(MOTOR_STBY_PORT, MOTOR_STBY_PIN);
#endif

    gpio.GPIO_Pin = LEFT_IN1_PIN | LEFT_IN2_PIN | RIGHT_IN1_PIN | RIGHT_IN2_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTOR_DIR_PORT, &gpio);
    GPIO_ResetBits(MOTOR_DIR_PORT, LEFT_IN1_PIN | LEFT_IN2_PIN | RIGHT_IN1_PIN | RIGHT_IN2_PIN);

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    GPIO_ResetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_7);

    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler = 0;
    tb.TIM_Period = MOTOR_PWM_PERIOD;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &tb);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &oc);
    TIM_OC2Init(TIM3, &oc);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    Motor_Coast();
    TIM_Cmd(TIM3, ENABLE);
    Motor_Stop();
}

void Motor_SetPwm(int16_t left_pwm, int16_t right_pwm)
{
    if (left_pwm == 0 && right_pwm == 0) {
        Motor_Coast();
        return;
    }

    motor_enable_driver();

    set_dir(MOTOR_DIR_PORT, LEFT_IN1_PIN, LEFT_IN2_PIN, left_pwm);
    set_dir(MOTOR_DIR_PORT, RIGHT_IN1_PIN, RIGHT_IN2_PIN, right_pwm);

    TIM_SetCompare1(TIM3, pwm_compare_for_cmd(left_pwm));
    TIM_SetCompare2(TIM3, pwm_compare_for_cmd(right_pwm));
}

void Motor_Coast(void)
{
    set_coast(MOTOR_DIR_PORT, LEFT_IN1_PIN, LEFT_IN2_PIN);
    set_coast(MOTOR_DIR_PORT, RIGHT_IN1_PIN, RIGHT_IN2_PIN);
    TIM_SetCompare1(TIM3, MOTOR_PWM_PERIOD);
    TIM_SetCompare2(TIM3, MOTOR_PWM_PERIOD);
    motor_disable_driver();
}

void Motor_Brake(void)
{
    motor_enable_driver();
    set_brake(MOTOR_DIR_PORT, LEFT_IN1_PIN, LEFT_IN2_PIN);
    set_brake(MOTOR_DIR_PORT, RIGHT_IN1_PIN, RIGHT_IN2_PIN);
    TIM_SetCompare1(TIM3, abs_limit_pwm((int16_t)MOTOR_BRAKE_PWM));
    TIM_SetCompare2(TIM3, abs_limit_pwm((int16_t)MOTOR_BRAKE_PWM));
}

void Motor_Stop(void)
{
#if MOTOR_STOP_BRAKE_ENABLE
    Motor_Brake();
#else
    Motor_Coast();
#endif
}
