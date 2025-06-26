#include "main.h"
#include "miros.h"
#include "OS_gpioConfig.h"
#include "OS_pid.h"
#include "OS_scheduler.h"
#include "OS_aperiodicServer.h"
#include "OS_semaphore.h"
#include "OS_resourceManager.h"

// Stacks e estruturas das threads
uint32_t stack_idleThread[40];
uint32_t stack_task1[40];
uint32_t stack_aperiodicServer[40];
uint32_t stack_button[40];

rtos::OSThread task1;
rtos::OSThread serverThread;
rtos::OSThread buttonThread;

rtos::TaskControlBlock tcb_task1, tcb_button;
rtos::aperiodicServerTCB server;


// Recurso compartilhado (abstracao)
rtos::Resource recurso_compartilhado = {false, nullptr};
volatile int valor_compartilhado = 0;


// Requisicoes aperiodicas
volatile int interrupt1 = 0, interrupt2 = 0;
uint32_t requisicao1 = 0, requisicao2 = 0;

//	Funcao da requisicao aperiodica
void aperiodic_requisition1(void*){
	//alterar setpoint do pid
}

//baixa --> media --> alta --> media --> baixa --> media --> alta

// Funcoes das tarefas
void tarefa1_func() {
	while(true){

		//ajustar o PID para manter o nivel da bolinha

        rtos::mark_task_completed(&tcb_task1);	//abstracao para marcar termino de uma tarefa
	}
}


// verifica o pressionamento do botao e entao adiciona requisicao aperiodica ao servidor
void button_func(){
	//chamar requisicao aperiodica
}


int main(void) {
    MX_GPIO_Init(); // Inicializa o GPIO do botao

    // Inicializacao dos TCBs
    rtos::init_task_control_block(&tcb_task1, 100, 10, 100);
    rtos::init_task_control_block(&tcb_button, 50, 10, 50);
    rtos::aperiodic_server_init(&server, 50, 10);

    // Inicializacao do RTOS
    rtos::OS_init(stack_idleThread, sizeof(stack_idleThread));

    OSThread_start(&task1, tarefa1_func, stack_task1, sizeof(stack_task1));
    OSThread_start(&buttonThread, button_func, stack_button, sizeof(stack_button));

    //	Inicializacao servidor aperiodico
    OSThread_start(&serverThread, rtos::aperiodic_server_func, stack_aperiodicServer, sizeof(stack_aperiodicServer));

    //	Atribuicao das tarefas
    rtos::add_thread_with_task(&task1, &tcb_task1);
    rtos::add_thread_with_task(&buttonThread, &tcb_button);
    rtos::associate_aperiodic_server_thread(&serverThread, &server);


    rtos::OS_run();

    while (true) {}
}
