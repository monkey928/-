#include "bsp_systick.h"
#include "bsp_usart.h"
#include "protocol.h"
#include "stm32f10x.h"

static void fault_shutdown_motor(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_ResetBits(GPIOB, GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
}

void HardFault_Handler(void)
{
    __disable_irq();
    fault_shutdown_motor();
    NVIC_SystemReset();
    while (1) {
    }
}

void MemManage_Handler(void)
{
    __disable_irq();
    fault_shutdown_motor();
    NVIC_SystemReset();
    while (1) {
    }
}

void BusFault_Handler(void)
{
    __disable_irq();
    fault_shutdown_motor();
    NVIC_SystemReset();
    while (1) {
    }
}

void UsageFault_Handler(void)
{
    __disable_irq();
    fault_shutdown_motor();
    NVIC_SystemReset();
    while (1) {
    }
}

void SysTick_Handler(void)
{
    SysTick_IncTick();
}

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        /* Reading SR (in GetITStatus) then DR also clears ORE/FE/NE, so the
         * received byte is consumed before any error-clear sequence can
         * discard it. */
        Protocol_FeedByte((uint8_t)USART_ReceiveData(USART1));
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    } else if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET ||
               USART_GetFlagStatus(USART1, USART_FLAG_FE) != RESET ||
               USART_GetFlagStatus(USART1, USART_FLAG_NE) != RESET) {
        (void)USART1->SR;
        (void)USART1->DR;
    }

    if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {
        USART1_TxIrqHandler();
    }
}
