#ifndef INC_OS_RESOURCEMANAGER_H_
#define INC_OS_RESOURCEMANAGER_H_

#ifdef INC_OS_SEMAPHORE_H_

#include <stdint.h>

namespace rtos {

typedef struct{
    bool locked;
    OSThread* owner;
}Resource;

void NPP_lock(Resource* res);

void NPP_unlock(Resource* res);
}


#endif	/* INC_OS_SEMAPHORE_H_ */

#endif /* INC_OS_RESOURCEMANAGER_H_ */
