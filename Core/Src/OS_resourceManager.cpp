#include "main.h"
#include "miros.h"
#include "OS_scheduler.h"
#include "OS_semaphore.h"
#include "OS_resourceManager.h"

namespace rtos {


void NPP_lock(Resource* res) {
    while (true) {
        __disable_irq();
        if (!res->locked) {
            res->locked = true;
            res->owner = OS_curr;
            OS_preempt_enabled = false;
            __enable_irq();
            break;
        }
        __enable_irq();
        yield(); // cede o processador para evitar busy-wait agressivo
    }
}

void NPP_unlock(Resource* res) {
    __disable_irq();
    if (res->owner == OS_curr) {
        res->locked = false;
        res->owner = nullptr;
        OS_preempt_enabled = true;
    }
    __enable_irq();
}

}


