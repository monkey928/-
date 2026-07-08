#include "bsp_usart.h"

#define USART1_TX_BUF_SIZE          64U

static volatile uint8_t g_tx_buf[USART1_TX_BUF_SIZE];
static volatile uint16_t g_tx_head = 0U;
static volatile uint16_t g_tx_tail = 0U;
static volatile uint8_t g_tx_busy = 0U;

void USART1_BspInit(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    USART_StructInit(&usart);
    usart.USART_BaudRate = baudrate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usart);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(USART1, ENABLE);
}

void USART1_SendByte(uint8_t byte)
{
    (void)USART1_SendBuffer(&byte, 1U);
}

uint8_t USART1_SendBuffer(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    uint16_t used;
    uint16_t free_space;

    if (buf == 0 || len == 0U || len >= USART1_TX_BUF_SIZE) {
        return 0U;
    }

    __disable_irq();
    if (g_tx_tail >= g_tx_head) {
        used = (uint16_t)(g_tx_tail - g_tx_head);
    } else {
        used = (uint16_t)(USART1_TX_BUF_SIZE - g_tx_head + g_tx_tail);
    }
    free_space = (uint16_t)(USART1_TX_BUF_SIZE - 1U - used);
    if (len > free_space) {
        __enable_irq();
        return 0U;
    }

    for (i = 0; i < len; i++) {
        uint16_t next_tail = (uint16_t)((g_tx_tail + 1U) % USART1_TX_BUF_SIZE);

        g_tx_buf[g_tx_tail] = buf[i];
        g_tx_tail = next_tail;
    }

    if (g_tx_busy == 0U && g_tx_head != g_tx_tail) {
        g_tx_busy = 1U;
        USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    }
    __enable_irq();
    return 1U;
}

void USART1_TxIrqHandler(void)
{
    if (g_tx_head == g_tx_tail) {
        g_tx_busy = 0U;
        USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        return;
    }

    USART_SendData(USART1, g_tx_buf[g_tx_head]);
    g_tx_head = (uint16_t)((g_tx_head + 1U) % USART1_TX_BUF_SIZE);
}
