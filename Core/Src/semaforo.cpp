#include "semaforo.h"
#include "miros.h"


uint32_t vazios = N;
uint32_t cheias = 0;
semaforo mutex = 1;
double buffer[N];


void wait(uint32_t &s) {
    while (s == 0);  // espera ocupada
    s--;
}

void signal(uint32_t &s) {
	while (s == N);
    s++;
}

Produtor::Produtor(){
	this->in = 0;
	this->min_delay = rtos::TICKS_PER_SEC * 2U / 10U;;
	this->max_delay = rtos::TICKS_PER_SEC * 8U / 10U;
}


void Produtor::sinal(){
	dado = (rand() % 20000) / 1000.0;
	this->rand_sensor_delay = min_delay + (rand() % (max_delay - min_delay + 1));
	rtos::OS_delay(rand_sensor_delay);
	//Simulando a aquisição de um sinal de corrente entre 0 e 20mA

}

void Produtor::produzir(){
	while(true){
		wait(vazios);
		wait(mutex);
		sinal();
		buffer[in] = dado;
		signal(mutex);
		signal(cheias);
		in = (in + 1) % N;
	}
}

Consumidor::Consumidor(){
	this->out = 0;
	this->min_delay = rtos::TICKS_PER_SEC * 4U / 10U;;
	this->max_delay = rtos::TICKS_PER_SEC * 10U / 10U;
}

void Consumidor::processar(){
	this->rand_processor_delay = min_delay + (rand() % (max_delay - min_delay + 1));
	rtos::OS_delay(rand_processor_delay);
	//Simulando o processamento do dado
}

void Consumidor::consumir(){
	while(true){
		wait(cheias);
		wait(mutex);
		processar();
		buffer[out] = -1.0;
		signal(mutex);
		signal(vazios);
		out = (out + 1) % N;
	}

}






