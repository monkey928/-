#include "bsp_encoder.h"
#include "app_config.h"

static uint16_t g_left_last_count;
static uint16_t g_right_last_count;

static void encoder_timer_init(TIM_TypeDef *tim)
{
    TIM_TimeBaseInitTypeDef tb;
    TIM_ICInitTypeDef ic;

    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler = 0;
    tb.TIM_Period = 0xFFFF;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &tb);

    TIM_EncoderInterfaceConfig(tim, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_ICStructInit(&ic);
    ic.TIM_ICFilter = 6;
    ic.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(tim, &ic);
    ic.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(tim, &ic);

    TIM_SetCounter(tim, 0);
    TIM_Cmd(tim, ENABLE);
}

void Encoder_BspInit(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOB, &gpio);

    encoder_timer_init(TIM2);
    encoder_timer_init(TIM4);
    g_left_last_count = TIM_GetCounter(TIM2);
    g_right_last_count = TIM_GetCounter(TIM4);
}

static int32_t timer_delta(uint16_t now, uint16_t last)
{
    uint16_t diff = (uint16_t)(now - last);

    if (diff >= 0x8000U) {
        return (int32_t)diff - 0x10000L;
    }
    return (int32_t)diff;
}

int32_t Encoder_GetLeftDelta(void)
{
    uint16_t now = TIM_GetCounter(TIM2);
    int32_t delta = timer_delta(now, g_left_last_count);
    g_left_last_count = now;
    return delta * LEFT_ENCODER_SIGN;
}

int32_t Encoder_GetRightDelta(void)
{
    uint16_t now = TIM_GetCounter(TIM4);
    int32_t delta = timer_delta(now, g_right_last_count);
    g_right_last_count = now;
    return delta * RIGHT_ENCODER_SIGN;
}
