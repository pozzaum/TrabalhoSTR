#ifndef INC_SEMAFORO_H_
#define INC_SEMAFORO_H_

#include <cstdint>
#include <cstdlib>
#include <ctime>

#define N 5

extern uint32_t vazio;
extern uint32_t cheias;
typedef uint32_t semaforo;
extern semaforo mutex;
extern double buffer[N];

class Produtor{
private:
	uint32_t in;
	uint32_t rand_sensor_delay;
	uint32_t min_delay;
	uint32_t max_delay;
	double dado;

public:
	Produtor();
	void sinal();		//simula o recebimento de um sinal
	void produzir();

};

class Consumidor{
private:
	uint32_t out;
	uint32_t rand_processor_delay;
	uint32_t min_delay;
	uint32_t max_delay;
public:
	Consumidor();
	void processar();	//simula o tempo de processamento e envio de um sinal
	void consumir();

};

#endif



