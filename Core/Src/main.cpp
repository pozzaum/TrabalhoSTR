#include "main.h"
#include "miros.h"
#include "OS_scheduler.h"
#include "OS_aperiodicServer.h"
#include "OS_semaphore.h"
#include "OS_resourceManager.h"

#define PushButton_Pin GPIO_PIN_13	//push_button integrado no stm
#define PushButton_GPIO_Port GPIOC

// Stacks e estruturas das threads
uint32_t stack_idleThread[40];
uint32_t stack_task1[40];
uint32_t stack_task2[40];
uint32_t stack_aperiodicServer[40];
uint32_t stack_button[40];

rtos::OSThread task1, task2;
rtos::OSThread serverThread;
rtos::OSThread buttonThread;

rtos::TaskControlBlock tcb_task1, tcb_task2, tcb_button;
rtos::aperiodicServerTCB server;


// Recurso compartilhado (abstracao)
rtos::Resource recurso_compartilhado = {false, nullptr};
volatile int valor_compartilhado = 0;


// Requisicoes aperiodicas
volatile int interrupt1 = 0, interrupt2 = 0;
uint32_t requisicao1 = 0, requisicao2 = 0;

volatile uint8_t button_pressed_flag = 0;
volatile uint32_t last_button_tick = 0;

//	Funcao da requisicao aperiodica
void aperiodic_requisition1(void*){
	requisicao1++;
}

// Funcoes das tarefas
void tarefa1_func() {
	while(true){
        rtos::NPP_lock(&recurso_compartilhado);
        valor_compartilhado += 1;
        rtos::NPP_unlock(&recurso_compartilhado);

        // Simula trabalho
        for (int i = 0; i < 10; ++i){};

        rtos::mark_task_completed(&tcb_task1);	//abstracao para marcar termino de uma tarefa
	}
}

void tarefa2_func() {
	while(true){

        rtos::NPP_lock(&recurso_compartilhado);
        valor_compartilhado += 2;
        rtos::NPP_unlock(&recurso_compartilhado);

        // Simula trabalho
        for (int i = 0; i < 15; ++i){};

        rtos::mark_task_completed(&tcb_task2);	//abstracao para marcar termino de uma tarefa
	}
}

// verifica o pressionamento do botao e entao adiciona requisicao aperiodica ao servidor
void button_func(){
    while (true) {
        if (button_pressed_flag) {
            rtos::aperiodic_server_add_request(&server, aperiodic_requisition1, nullptr);
            button_pressed_flag = 0;
        }
        rtos::mark_task_completed(&tcb_button); // coopera com o escalonador
    }
}

// configuracao do GPIO
static void MX_GPIO_Init(void){
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

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


int main(void) {
    MX_GPIO_Init(); // Inicializa o GPIO do botao

    // Inicializacao dos TCBs
    rtos::init_task_control_block(&tcb_task1, 100, 10, 100);
    rtos::init_task_control_block(&tcb_task2, 150, 10, 150);
    rtos::init_task_control_block(&tcb_button, 50, 10, 50);
    rtos::aperiodic_server_init(&server, 50, 10);


    // Inicializacao do RTOS
    rtos::OS_init(stack_idleThread, sizeof(stack_idleThread));

    OSThread_start(&task1, tarefa1_func, stack_task1, sizeof(stack_task1));
    OSThread_start(&task2, tarefa2_func, stack_task2, sizeof(stack_task2));
    OSThread_start(&buttonThread, button_func, stack_button, sizeof(stack_button));

    //	Inicializacao servidor aperiodico
    OSThread_start(&serverThread, rtos::aperiodic_server_func, stack_aperiodicServer, sizeof(stack_aperiodicServer));

    //	Atribuicao das tarefas
    rtos::add_thread_with_task(&task1, &tcb_task1);
    rtos::add_thread_with_task(&task2, &tcb_task2);
    rtos::add_thread_with_task(&buttonThread, &tcb_button);
    rtos::associate_aperiodic_server_thread(&serverThread, &server);


    rtos::OS_run();

    while (true) {}
}
