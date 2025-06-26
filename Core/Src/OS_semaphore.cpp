#include "main.h"
#include "miros.h"
#include "OS_semaphore.h"

void wait(uint32_t &s) {
    __disable_irq();
    while (s == 0);  // espera ocupada
    s--;
    __enable_irq();
}

void signal(uint32_t &s) {
	__disable_irq();
	while (s == BUFFER_SIZE);
    s++;
    __enable_irq();
}

Producer::Producer(double* buf, uint32_t& emp, uint32_t& ful, semaphore& mut, ProducerFunc func)
    : buffer(buf), emptys(emp), fulls(ful), mutex(mut), in(0), produce_func(func) {}

void Producer::produce(){
	while(true){
		wait(emptys);
		wait(mutex);
        //double data;
        //produce_func(data); // Produz o item (callback)
		signal(mutex);
		signal(fulls);
		in = (in + 1) % BUFFER_SIZE;
	}
}

Consumer::Consumer(double* buf, uint32_t& emp, uint32_t& ful, semaphore& mut, ConsumerFunc func)
    : buffer(buf), emptys(emp), fulls(ful), mutex(mut), out(0), consume_func(func) {}


void Consumer::consume(){
	while(true){
		wait(fulls);
		wait(mutex);
		//double data = buffer[out];
		buffer[out] = -1.0; // Marca como consumido (opcional)
		signal(mutex);
		signal(emptys);
		out = (out + 1) % BUFFER_SIZE;
	}

}






