#ifndef INC_OS_GPIOCONFIG_H_
#define INC_OS_GPIOCONFIG_H_

#ifdef INC_MIROS_H_

#define PushButton_Pin GPIO_PIN_13	//push_button integrado no stm
#define PushButton_GPIO_Port GPIOC

#define	SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB

#define SLC_Pin GPIO_PIN_15
#define SLC_GPIO_Port GPIOA

#define PWM_Pin GPIO_PIN_2
#define PWM_GPIO_Port GPIOC

extern void MX_GPIO_Init(void);

extern void EXTI15_10_IRQHandler(void);

extern void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif

#endif /* INC_OS_GPIOCONFIG_H_ */
