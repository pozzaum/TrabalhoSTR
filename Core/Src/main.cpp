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


//	Funcao da requisicao aperiodica
void aperiodic_requisition(void*){
	//alterar setpoint do pid
}

// Funcoes das tarefas
void sensor_read() {
	while(true){
		//	funcao para leitura do sensor

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
