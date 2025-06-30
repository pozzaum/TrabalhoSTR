#include "main.h"
#include "miros.h"
#include "OS_gpioConfig.h"
#include "OS_pid.h"
#include "OS_scheduler.h"
#include "OS_aperiodicServer.h"
#include "OS_semaphore.h"
#include "OS_resourceManager.h"
#include "VL53L0X.h"

// Stacks e estruturas das threads
uint32_t stack_idleThread[40];
uint32_t stack_task_sensor[40];
uint32_t stack_task_pid[40];
uint32_t stack_task_pwm[40];
uint32_t stack_task_button[40];

uint32_t stack_aperiodicServer[40];

rtos::OSThread task_sensor, task_pid, task_pwm, task_button;
rtos::OSThread serverThread;

rtos::TaskControlBlock tcb_task_sensor, tcb_task_pid, tcb_task_pwm, tcb_task_button;
rtos::aperiodicServerTCB server;


// Recurso compartilhado (abstracao)
rtos::Resource recurso_compartilhado = {false, nullptr};
volatile int valor_compartilhado = 0;

// Requisicoes aperiodicas
volatile int setpoint;

volatile uint8_t button_pressed_flag = 0;
volatile uint32_t last_button_tick = 0;



//	inicializacao I2C

I2C_HandleTypeDef hi2c1;

void I2CError_Handler(void){
	while(true){
		//	nunca deve entrar aqui --> erro na inicializacao do i2c
	}
}

static void MX_I2C1_Init(void)
{
  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
	  I2CError_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
	  I2CError_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0x0) != HAL_OK)
  {
	  I2CError_Handler();
  }
}

//	inicializacao GPIO
static void MX_GPIO_Init(void){

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




//FUNCAO DAS REQUISICOES APERIODICAS
void aperiodic_requisition(void*){
	//alterar setpoint do pid
}



//FUNCOES DAS TAREFAS PERIODICAS

//	Leitura do sensor
statInfo_t_VL53L0X distanceStats;
volatile int current_distance;

void sensor_read() {
	while(true){
		//	funcao para leitura do sensor
		current_distance = (int)readRangeContinuousMillimeters(&distanceStats);

        rtos::mark_task_completed(&tcb_task_sensor);	//abstracao para marcar termino de uma tarefa
	}
}

void pid_adjust(){
	while(true){
		// funcao para regular o pid

        rtos::mark_task_completed(&tcb_task_pid);	//abstracao para marcar termino de uma tarefa

	}

}

void pwm_adjust(){
	while(true){
		//	funcao para regular o pwm

        rtos::mark_task_completed(&tcb_task_pwm);	//abstracao para marcar termino de uma tarefa
	}
}



//	leitura do botão

void button_func(){
    while (true) {
        if (button_pressed_flag) {
            rtos::aperiodic_server_add_request(&server, aperiodic_requisition, nullptr);
            button_pressed_flag = 0;
        }
        rtos::mark_task_completed(&tcb_task_button); // coopera com o escalonador
    }
}

int main(void) {
	MX_I2C1_Init();	// Inicaliz I2C
    MX_GPIO_Init(); // Inicializa o GPIO do botao

    // Inicializacao dos TCBs
    rtos::init_task_control_block(&tcb_task_sensor, 100, 10, 100);
    rtos::init_task_control_block(&tcb_task_pid, 100, 10, 100);
    rtos::init_task_control_block(&tcb_task_pwm, 100, 10, 100);
    rtos::init_task_control_block(&tcb_task_button, 50, 10, 50);

    rtos::aperiodic_server_init(&server, 50, 10);

    // Inicializacao do RTOS
    rtos::OS_init(stack_idleThread, sizeof(stack_idleThread));

    OSThread_start(&task_sensor, sensor_read, stack_task_sensor, sizeof(stack_task_sensor));
    OSThread_start(&task_pid, pid_adjust, stack_task_pid, sizeof(stack_task_pid));
    OSThread_start(&task_pwm, pwm_adjust, stack_task_pwm, sizeof(stack_task_pwm));
    OSThread_start(&task_button, button_func, stack_task_button, sizeof(stack_task_button));


    //	Inicializacao servidor aperiodico
    OSThread_start(&serverThread, rtos::aperiodic_server_func, stack_aperiodicServer, sizeof(stack_aperiodicServer));

    //	Atribuicao das tarefas
    rtos::add_thread_with_task(&task_sensor, &tcb_task_sensor);
    rtos::add_thread_with_task(&task_pid, &tcb_task_pid);
    rtos::add_thread_with_task(&task_pwm, &tcb_task_pwm);
    rtos::add_thread_with_task(&task_button, &tcb_task_button);

    rtos::associate_aperiodic_server_thread(&serverThread, &server);


    rtos::OS_run();

    while (true) {}
}
