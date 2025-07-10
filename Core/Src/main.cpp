#include "main.h"
#include "stm32g4xx_hal_tim.h"
#include "miros.h"
#include "OS_gpioConfig.h"
#include "OS_pid.h"
#include "OS_scheduler.h"
#include "OS_aperiodicServer.h"
#include "OS_semaphore.h"
#include "OS_resourceManager.h"
#include "VL53L0X.h"

// Instancia do gerenciador de memoria compartilhada
rtos::SharedMemoryManager smManager;

// IDs dos recursos compartilhados (serao preenchidos na criacao)
rtos::ResourceID resource_PIDSetpoint_ID = 0;
rtos::ResourceID resource_PIDInput_ID = 0;

#define PID_SCALE 1000000

// Ganhos desejados
#define KP_FIXED   (-0.0001  * PID_SCALE)  // -100
#define KI_FIXED   (-0.00001 * PID_SCALE)  // -10
#define KD_FIXED   ( 0.00001 * PID_SCALE)  // 10
#define PID_MIN	   ( 0.52 * PID_SCALE)
#define PID_MAX	   ( 0.58 * PID_SCALE)


uint32_t pwm_signal = 0;

// Stacks e estruturas das threads
uint32_t stack_idleThread[40];
uint32_t stack_task_sensor[40];
uint32_t stack_task_pid[40];
uint32_t stack_task_pwm[40];

uint32_t stack_aperiodicServer[40];

rtos::OSThread task_sensor, task_pid, task_pwm;
rtos::OSThread serverThread;

rtos::TaskControlBlock tcb_task_sensor, tcb_task_pid, tcb_task_pwm;;
rtos::aperiodicServerTCB server;


PIDController pid;

// Requisicoes aperiodicas
volatile uint32_t setpoint;

volatile uint8_t button_pressed_flag = 0;
volatile uint32_t last_button_tick = 0;

volatile uint8_t height_setpoint_mask;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim20;
DMA_HandleTypeDef hdma_tim20_ch2;

static void SystemClock_Config();
static void MX_I2C1_Init();
static void MX_TIM20_Init();
static void MX_DMA_Init(void);
void EXTI15_10_IRQHandler();
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
static void MX_GPIO_Init();
void Error_Handler();
void Resource_Managment_Error_Handler();


//FUNCAO DAS REQUISICOES APERIODICAS

uint32_t cont_aperiodic = 0;
void aperiodic_requisition(void*){
	uint32_t pid_setpoint = 55;
	bool dummy = false;
	height_setpoint_mask = (height_setpoint_mask + 1) % 3;

	switch(height_setpoint_mask){
	case 0:

		pid_setpoint = 53;
		break;

	case 1:

		pid_setpoint = 55;
		break;

	case 2:

		pid_setpoint = 57;
		break;

	default:
		Resource_Managment_Error_Handler();

	break;
	}
	do{
		dummy = smManager.manageResource(resource_PIDSetpoint_ID, "WRITE", sizeof(uint32_t), "I", &pid_setpoint);
	}while(!dummy);

	button_pressed_flag = 0;
	cont_aperiodic++;
}



//FUNCOES DAS TAREFAS PERIODICAS

//	Leitura do sensor
statInfo_t_VL53L0X distanceStats;
volatile int current_distance;

void sensor_read() {
	while(true){
		bool dummy = false;
		//	funcao para leitura do sensor
		uint32_t current_distance = (int32_t)readRangeContinuousMillimeters(&distanceStats);

		do{
			dummy = smManager.manageResource(resource_PIDInput_ID, "WRITE", sizeof(uint32_t), "I", &current_distance);
		}while(!dummy);

        rtos::mark_task_completed(&tcb_task_sensor);	//abstracao para marcar termino de uma tarefa
	}
}

void pid_adjust(){
	while(true){
		uint32_t pid_setpoint, pid_input;
		bool dummy_1, dummy_2;
		do{
			dummy_1 = smManager.manageResource(resource_PIDSetpoint_ID, "READ", sizeof(uint32_t), "", &pid_setpoint);
		}while(!dummy_1);

		do{
			dummy_2 = smManager.manageResource(resource_PIDInput_ID, "READ", sizeof(uint32_t), "", &pid_input);
		}while(!dummy_2);

		uint32_t error = pid_setpoint - pid_input;

		//LUCÃO --> Recurso Compartilhado
		uint32_t pid_pwm_value = PID_action(&pid, error);

		pwm_signal = pid_pwm_value;

        rtos::mark_task_completed(&tcb_task_pid);	//abstracao para marcar termino de uma tarefa

	}

}


void pwm_adjust(){
	while(true){
		//	funcao para regular o pwm

		// Use a variável global 'pwmVal' calculada pela tarefa do PID
		// O registrador CCR1 está dentro da estrutura htim2.Instance
		//htim2.Instance->CCR1 = (uint32_t)(pwmVal * htim2.Instance->ARR);

		//LUCÃO --> Recurso Compartilhado
		//htim20.Instance->CCR1 = (pwm_signal * htim20.Instance->ARR) / PID_SCALE;
		//htim20.Instance->CCR2 = 30;
		//htim20.Instance->CCR2 = 0;
		rtos::mark_task_completed(&tcb_task_pwm);	//abstracao para marcar termino de uma tarefa
	}
}


int main(void) {
	HAL_Init();
	SystemClock_Config();
	MX_DMA_Init();
	MX_GPIO_Init(); // Inicializa o GPIO
	MX_I2C1_Init();	// Inicializa I2C
    MX_TIM20_Init(); // Inicializa o Timer
    htim20.Instance->CCR2 = 80;
    HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_2);

    PID_setup(&pid, KP_FIXED, KI_FIXED, KD_FIXED, setpoint, PID_MAX, PID_MIN);

    // Inicializacao dos TCBs
    rtos::init_task_control_block(&tcb_task_sensor, 100, 10, 100);
    rtos::init_task_control_block(&tcb_task_pid, 100, 10, 100);
    rtos::init_task_control_block(&tcb_task_pwm, 100, 10, 100);

    rtos::aperiodic_server_init(&server, 50, 10);

    // Inicializacao do RTOS
    rtos::OS_init(stack_idleThread, sizeof(stack_idleThread));

    OSThread_start(&task_sensor, sensor_read, stack_task_sensor, sizeof(stack_task_sensor));
    OSThread_start(&task_pid, pid_adjust, stack_task_pid, sizeof(stack_task_pid));
    OSThread_start(&task_pwm, pwm_adjust, stack_task_pwm, sizeof(stack_task_pwm));


    //	Inicializacao servidor aperiodico
    OSThread_start(&serverThread, rtos::aperiodic_server_func, stack_aperiodicServer, sizeof(stack_aperiodicServer));

    //	Atribuicao das tarefas
    rtos::add_thread_with_task(&task_sensor, &tcb_task_sensor);
    rtos::add_thread_with_task(&task_pid, &tcb_task_pid);
    rtos::add_thread_with_task(&task_pwm, &tcb_task_pwm);

    rtos::associate_aperiodic_server_thread(&serverThread, &server);

    // tamanho maximo de 32 bits
    if (!smManager.manageResource(resource_PIDSetpoint_ID, "CREATE", sizeof(uint32_t), "I")) {
        while(1); // Trava o sistema se nao conseguir criar recurso critico
    }

    // tamanho maximo de 32 bits
    if (!smManager.manageResource(resource_PIDInput_ID, "CREATE", sizeof(uint32_t), "I")) {
        while(1); // Trava o sistema
    }

    rtos::OS_run();

    while (true) {}
}




void Error_Handler(){
	__disable_irq();
	while(true){

	}
}

void Resource_Managment_Error_Handler(){
	__disable_irq();
	while(true){

	}
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x40B285C2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM20 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM20_Init(void)
{

  /* USER CODE BEGIN TIM20_Init 0 */

  /* USER CODE END TIM20_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM20_Init 1 */

  /* USER CODE END TIM20_Init 1 */
  htim20.Instance = TIM20;
  htim20.Init.Prescaler = 0;
  htim20.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim20.Init.Period = 99;
  htim20.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim20.Init.RepetitionCounter = 0;
  htim20.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim20) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim20, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim20) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim20, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim20, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim20, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM20_Init 2 */

  /* USER CODE END TIM20_Init 2 */
  HAL_TIM_MspPostInit(&htim20);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void){
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOF_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = PushButton_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // interrupcao na borda de descida
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(PushButton_GPIO_Port, &GPIO_InitStruct);

    // prioridade e habilitacao da interrupcao EXTI para PC13
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1U, 1U);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

