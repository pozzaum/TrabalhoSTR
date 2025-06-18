#ifndef INC_OS_APERIODICSERVER_H_
#define INC_OS_APERIODICSERVER_H_

#ifdef INC_OS_SCHEDULER_H_

#define APERIODIC_QUEUE_SIZE 8

typedef struct {
    void (*func)(void*); // Ponteiro para função da requisição
    void* arg;           // Argumento da função
} AperiodicRequest;

namespace rtos{

typedef struct{
	TaskControlBlock tcb;
	uint32_t serverPeriod;
	uint32_t budget;
	uint32_t execTime;
	uint32_t serverActivations;

	AperiodicRequest queue[APERIODIC_QUEUE_SIZE];
	uint8_t head;
	uint8_t tail;

	bool isAperiodicServer;

}aperiodicServerTCB;

extern void associate_aperiodic_server_thread(OSThread* thread, aperiodicServerTCB* server);

extern void recharge_Budget(OSThread* thread);

extern void aperiodic_server_init(aperiodicServerTCB* server, uint32_t serverPeriod, uint32_t budget);

extern void aperiodic_server_add_request(aperiodicServerTCB* server, void (*func)(void*), void* arg);

extern AperiodicRequest* aperiodic_server_get_next_request(aperiodicServerTCB* server);

extern bool fila_vazia(aperiodicServerTCB* server);

extern void aperiodic_server_func();

}

#endif

#endif /* INC_OS_SCHEDULER_H_ */
