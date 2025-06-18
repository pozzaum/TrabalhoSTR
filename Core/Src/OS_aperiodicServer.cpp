#include "main.h"
#include "miros.h"
#include "OS_scheduler.h"
#include "OS_aperiodicServer.h"


namespace rtos{

//associa o servidor a um tcb, como qualquer outra thread
void associate_aperiodic_server_thread(OSThread* thread, aperiodicServerTCB* server) {
    add_thread_with_task(thread, &server->tcb);
}

//inicializacao de um servidor aperiodico
void aperiodic_server_init(aperiodicServerTCB* server, uint32_t serverPeriod, uint32_t budget){
	init_task_control_block(&server->tcb, serverPeriod, budget, serverPeriod);
	server->serverPeriod = serverPeriod;
	server->budget = budget;
	server->execTime = budget;
	server->serverActivations = 0;
	server->isAperiodicServer = true;
}


//recarga de budget a cada periodo
void recharge_Budget(OSThread* thread) {
    // tenta fazer cast para aperiodicServerTCB
    aperiodicServerTCB* server = reinterpret_cast<aperiodicServerTCB*>(thread->tcb);

    // verifica se e um servidor
    if (server->isAperiodicServer) {
        if ((OS_tickCount / server->serverPeriod) >= server->serverActivations) {
            server->execTime = server->budget;
            server->serverActivations += 1;
        }
    }
}

//adiciona uma requisicao aperiodica para ser executada
void aperiodic_server_add_request(aperiodicServerTCB* server, void (*func)(void*), void* arg) {
    uint8_t next_tail = (server->tail + 1) % APERIODIC_QUEUE_SIZE;
    if (next_tail != server->head) { // fila nao cheia: insere normalmente
        server->queue[server->tail].func = func;
        server->queue[server->tail].arg = arg;
        server->tail = next_tail;
    } else {
        // fila cheia: sobrescreve a requisicao mais antiga
        server->head = (server->head + 1) % APERIODIC_QUEUE_SIZE; // descarta a mais antiga
        server->queue[server->tail].func = func;
        server->queue[server->tail].arg = arg;
        server->tail = next_tail;
    }
}

//direciona a proxima requisicao do servidor
AperiodicRequest* aperiodic_server_get_next_request(aperiodicServerTCB* server) {
    if (server->head != server->tail) { // fila nao vazia
        AperiodicRequest* req = &server->queue[server->head];
        server->head = (server->head + 1) % APERIODIC_QUEUE_SIZE;
        return req;
    }
    return nullptr; // fila vazia
}

bool fila_vazia(aperiodicServerTCB* server) {
    return (server->head == server->tail);
}


void aperiodic_server_func() {
	// obtem o TCB associado a thread corrente
    aperiodicServerTCB* server = (aperiodicServerTCB*)(OS_curr->tcb);

    while (1) {
        while (server->execTime > 0) {
            AperiodicRequest* req = aperiodic_server_get_next_request(server);
            if (req != nullptr) {
                req->func(req->arg);
                server->execTime--;
            } else {
                break;
            }
        }
        if (server->execTime == 0) {
        	mark_task_completed(&server->tcb);
            while (!server->tcb.ready) {
                yield();
            }
        } else {
            yield();
        }
    }
}

}

