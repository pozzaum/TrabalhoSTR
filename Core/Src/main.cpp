#include "main.h"
#include <cstdint>
#include "miros.h"
#include "semaforo.h"

Produtor sensor1;
Consumidor processador1, processador2;

uint32_t stack_idleThread[40];

uint32_t stack_produtor[40];
rtos::OSThread produtor;
void main_produtor() {
    while (1) {
    	sensor1.produzir();
    }
}

uint32_t stack_consumidor1[40];
rtos::OSThread consumidor1;
void main_consumidor1() {
    while (1) {
    	rtos::OS_delay(rtos::TICKS_PER_SEC);
        processador1.consumir();
    }
}

uint32_t stack_consumidor2[40];
rtos::OSThread consumidor2;
void main_consumidor2() {
    while (1) {
    	rtos::OS_delay(rtos::TICKS_PER_SEC);
        processador2.consumir();
    }
}


int main(void)
{
	  srand(HAL_GetTick());
	  rtos::OS_init(stack_idleThread, sizeof(stack_idleThread));

	  //start blinky1 thread
	  rtos::OSThread_start(&produtor,
	                       &main_produtor,
	                       stack_produtor, sizeof(stack_produtor));

	  rtos::OSThread_start(&consumidor1,
	                       &main_consumidor1,
	                       stack_consumidor1, sizeof(stack_consumidor1));

	  rtos::OSThread_start(&consumidor2,
	                       &main_consumidor2,
	                       stack_consumidor2, sizeof(stack_consumidor2));



	  //transfer control to the RTOS to run the threads
	  rtos::OS_run();
}
