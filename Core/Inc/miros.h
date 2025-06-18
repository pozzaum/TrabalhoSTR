#ifndef INC_MIROS_H_
#define INC_MIROS_H_

namespace rtos {

extern volatile uint32_t OS_tickCount;


typedef struct {
    void (*task_func)(void);   	// função da tarefa
    uint32_t period;			// periodo (em ticks)
    uint32_t wcet;				// tempo de execução (em ticks)
    uint32_t exec_time;
    uint32_t deadline_rel;		// deadline relativa (em ticks)
    uint32_t deadline_abs;		// próxima deadline absoluta (em ticks)
    uint32_t activations;		// ativacões
    uint32_t conclusions;		// conclusões
    uint32_t last_release;
    uint32_t overruns;
    uint32_t wake_tick;
    bool ready;             	// flag se está pronta
    bool on_wait;
} TaskControlBlock;

typedef struct {
	void *sp;						  // stack pointer
	void (*thread_func)(void);       // função da thread
	TaskControlBlock* tcb;          // ponteiro para a task associada
} OSThread;


extern OSThread * volatile OS_curr;   // Thread atual
extern OSThread * volatile OS_next;   // Próxima thread
extern OSThread *OS_thread[32 + 1];   // Array de threads
extern uint32_t OS_readySet;          // Bitmask de threads prontas
extern uint8_t OS_threadNum;          // Número de threads
extern uint8_t OS_currIdx;
extern OSThread idleThread;

const uint16_t TICKS_PER_SEC = 100U;

typedef void (*OSThreadHandler)();

void OS_init(void *stkSto, uint32_t stkSize);

/* callback to handle the idle condition */
void OS_onIdle(void);

/* this function must be called with interrupts DISABLED */
void OS_sched(void);

/* transfer control to the RTOS to run the threads */
void OS_run(void);

/* blocking delay */
void OS_delay(uint32_t ticks);

/* process all timeouts */
void OS_tick(void);

/* callback to configure and start interrupts */
void OS_onStartup(void);

void OSThread_start(
    OSThread *me,
    OSThreadHandler threadHandler,
    void *stkSto, uint32_t stkSize);
}

#endif /* INC_MIROS_H_ */
