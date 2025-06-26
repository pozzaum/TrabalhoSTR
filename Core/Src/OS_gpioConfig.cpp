#include "main.h"
#include "miros.h"
#include "OS_gpioConfig.h"

extern volatile uint8_t button_pressed_flag;
extern volatile uint32_t last_button_tick;

// configuracao do GPIO
void MX_GPIO_Init(void){

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin = PushButton_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // interrupcao na borda de descida
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(PushButton_GPIO_Port, &GPIO_InitStruct);

    // prioridade e habilitacao da interrupcao EXTI para PC13
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1U, 1U);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}


//	redefinicao do handler
void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(PushButton_Pin);
}


// redefinicao do callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == PushButton_Pin) {
        uint32_t now = rtos::OS_tickCount;
        if ((now - last_button_tick > 10) && (button_pressed_flag == 0)) {	//	10 ticks debounce
            button_pressed_flag = 1;
            last_button_tick = now;
        }
    }
}

