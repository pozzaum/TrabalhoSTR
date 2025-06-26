#ifndef INC_OS_SEMAPHORE_H_
#define INC_OS_SEMAPHORE_H_

#ifdef INC_MIROS_H_

#define BUFFER_SIZE 5

typedef uint32_t semaphore;

// Funções de callback para produção e consumo
typedef void (*ProducerFunc)(double&); // Produz um item
typedef void (*ConsumerFunc)(double&); // Consome um item

class Producer {
private:
    double* buffer;
    uint32_t& emptys;
    uint32_t& fulls;
    semaphore& mutex;
    uint32_t in;
    ProducerFunc produce_func;

public:
    Producer(double* buf, uint32_t& emp, uint32_t& ful, semaphore& mut, ProducerFunc func);
    void produce();
};

class Consumer {
private:
    double* buffer;
    uint32_t& emptys;
    uint32_t& fulls;
    semaphore& mutex;
    uint32_t out;
    ConsumerFunc consume_func;

public:
    Consumer(double* buf, uint32_t& emp, uint32_t& ful, semaphore& mut, ConsumerFunc func);
    void consume();
};

void wait(uint32_t &s);
void signal(uint32_t &s);


#endif

#endif



