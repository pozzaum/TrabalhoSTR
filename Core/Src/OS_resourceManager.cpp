#include "OS_resourceManager.h"
#include <cstring> // Para memcpy, strcmp, strncpy
#include <new>     // Para (std::nothrow)


namespace rtos {

// Construtor
SharedMemoryManager::SharedMemoryManager() : currentResourceCount(0), nextResourceID(1) {
    // Inicializa todos os slots do pool como nao usados
    for (uint8_t i = 0; i < MAX_SHARED_RESOURCES; ++i) {
        resourcePool[i].inUse = false;
        resourcePool[i].data = nullptr; // Garante que os ponteiros de dados sao nulos
    }
}

// Destrutor
SharedMemoryManager::~SharedMemoryManager() {
    enter_critical_section(); // Protege o acesso as estruturas de controle
    for (uint8_t i = 0; i < MAX_SHARED_RESOURCES; ++i) {
        if (resourcePool[i].inUse) {
            // Se o recurso estiver em uso, pode ser um problema tentar destrui-lo.
            if (resourcePool[i].resourceSemaphore.available == 0) {
                continue;
            }
            delete[] resourcePool[i].data; // Libera os dados do recurso
            resourcePool[i].data = nullptr;
            resourcePool[i].inUse = false;
            currentResourceCount--;
        }
    }
    exit_critical_section();
}

////////
// brief: Funcao principal para direcionar diferentes operacoes de gerenciamento
//        de recursos compartilhados (criar, destruir, ler, escrever).
//        Esta funcao atua como um dispatcher para as funcoes privadas de operacao.
//
// input:
// id (ResourceID&): O ID do recurso a ser operado. Para "CREATE", este sera preenchido.
// operation (const char*): A operacao desejada ("CREATE", "DESTROY", "READ", "WRITE").
// size (size_t): O tamanho do recurso (obrigatorio para "CREATE", para "READ"/"WRITE" e o tamanho do buffer/dados).
// type (const char*): O tipo do recurso (obrigatorio para "CREATE").
// dataBuffer (uint8_t*): Ponteiro para o buffer de dados a ser lido ou escrito (nullptr para "CREATE"/"DESTROY").
//
// return: bool: true se a operacao foi bem-sucedida, false caso contrario.
////////
bool SharedMemoryManager::manageResource(ResourceID& id, const char* operation, size_t size,
                                        const char* type, uint32_t* dataBuffer) {
    bool returnVal = false;

    if (strcmp(operation, "CREATE") == 0) {
        returnVal = createResource(id, size, type);
    } else if (strcmp(operation, "DESTROY") == 0) {
        returnVal = destroyResource(id);
    } else if (strcmp(operation, "READ") == 0) {
        returnVal = readResource(id, dataBuffer, size);
    } else if (strcmp(operation, "WRITE") == 0) {
        returnVal = writeResource(id, dataBuffer, size);
    }

    return returnVal;
}

////////
// brief: Encontra um slot vazio no resourcePool para alocar um novo recurso.
//
// input: (nenhum)
//
// return: int: O indice do slot vazio encontrado, ou -1 se nao houver slots disponiveis.
////////
int SharedMemoryManager::findEmptyResourceSlot() {
    if (currentResourceCount >= MAX_SHARED_RESOURCES) {
        return -1; // Pool cheio
    }
    for (uint8_t i = 0; i < MAX_SHARED_RESOURCES; ++i) {
        if (!resourcePool[i].inUse) {
            return i; // Slot vazio encontrado
        }
    }
    return -1; // Nao deveria acontecer se currentResourceCount < MAX_SHARED_RESOURCES, mas por seguranca
}

////////
// brief: Encontra o indice de um recurso no resourcePool dado seu ResourceID.
//
// input:
// id (ResourceID): O ID "falso" do recurso a ser procurado.
//
// return: int: O indice do recurso no resourcePool, ou -1 se o ID nao for encontrado.
////////
int SharedMemoryManager::findResourceIndexByID(ResourceID id) const {
    for (uint8_t i = 0; i < MAX_SHARED_RESOURCES; ++i) {
        if (resourcePool[i].inUse && resourcePool[i].id == id) {
            return i; // Recurso encontrado
        }
    }
    return -1; // Recurso nao encontrado
}

////////
// brief: Cria um novo recurso de memoria compartilhada.
//        Aloca memoria para a estrutura do recurso e seus dados,
//        inicializa o semaforo do recurso e o adiciona ao sistema.
//        Protege a manipulacao das estruturas internas com secoes criticas.
//
// input:
// newID (ResourceID&): Referencia para onde o ID "falso" do novo recurso sera armazenado.
// size (size_t): O tamanho em bytes do recurso a ser alocado.
// type (const char*): A string C que descreve o tipo do recurso.
//
// return: bool: true se o recurso foi criado com sucesso, false caso contrario.
////////
bool SharedMemoryManager::createResource(ResourceID& newID, size_t size, const char* type) {
    bool returnVal = false;
    enter_critical_section();

    int index = findEmptyResourceSlot();
    if (index == -1) {
        returnVal = false;
    } else if (type == nullptr || strlen(type) >= MAX_RESOURCE_TYPE_LEN) { // Validacao de tipo
        returnVal = false;
    } else {
        // Atribui o novo ID e incrementa para o proximo
        newID = nextResourceID++;
        resourcePool[index].id = newID;

        // Aloca memoria para os dados do recurso
        resourcePool[index].data = (uint8_t*) new (std::nothrow) uint8_t[size];
        if (resourcePool[index].data == nullptr) {
            resourcePool[index].inUse = false; // Garante que o slot nao e marcado como em uso
            returnVal = false;
        } else {
            resourcePool[index].size = size;
            strncpy(resourcePool[index].type, type, MAX_RESOURCE_TYPE_LEN - 1); // Copia o tipo
            resourcePool[index].type[MAX_RESOURCE_TYPE_LEN - 1] = '\0'; // Garante terminador nulo
            resourcePool[index].resourceSemaphore.init(); // Inicializa o semaforo
            resourcePool[index].inUse = true; // Marca o slot como em uso
            currentResourceCount++;

            returnVal = true;
        }
    }

    exit_critical_section();
    return returnVal;
}

////////
// brief: Destroi um recurso de memoria compartilhada existente.
//        Libera a memoria alocada para o recurso e o remove do sistema.
//        Verifica se o recurso esta em uso antes de destruir.
//        Protege a manipulacao das estruturas internas com secoes criticas.
//
// input:
// id (ResourceID): O ID "falso" do recurso a ser destruido.
//
// return: bool: true se o recurso foi destruido com sucesso, false caso contrario.
////////
bool SharedMemoryManager::destroyResource(ResourceID id) {
    bool returnVal = false;

    int index = findResourceIndexByID(id);
    if (index == -1) {
        returnVal = false;
    } else {
        SharedResource* resToDestroy = &resourcePool[index];

        if (!resToDestroy->resourceSemaphore.acquire()) { // acquire() tem sua propria secao critica
            returnVal = false;
        } else {
        	enter_critical_section();
            delete[] resToDestroy->data; // Libera os dados
            resToDestroy->data = nullptr; // Zera o ponteiro
            resToDestroy->inUse = false; // Marca o slot como nao usado
            currentResourceCount--;
            exit_critical_section();
            returnVal = true;
        }
    }

    return returnVal;
}

////////
// brief: Le dados de um recurso compartilhado para um buffer fornecido.
//        Adquire o semaforo do recurso para garantir exclusao mutua durante a leitura.
//        A validacao do ID e a obtencao do ponteiro para o recurso sao protegidas
//        por uma secao critica global para atomicidade.
//
// input:
// id (ResourceID): O ID "falso" do recurso a ser lido.
// buffer (uint8_t*): Ponteiro para o buffer de destino onde os dados serao copiados.
// bufferSize (size_t): O tamanho do buffer de destino. Deve ser >= ao tamanho do recurso.
//
// return: bool: true se a leitura foi bem-sucedida, false caso contrario.
////////
bool SharedMemoryManager::readResource(ResourceID id, uint32_t* buffer, size_t bufferSize) {
    bool returnVal = false;
    SharedResource* res = nullptr;
    int index = -1;

    index = findResourceIndexByID(id);
    if (index == -1) {
        returnVal = false;
    } else {
        res = &resourcePool[index]; // Obtem o ponteiro para o recurso
        bool acquired = res->resourceSemaphore.acquire(); // Tenta adquirir o semaforo
        												  // detalhe que o acquire ja usa
        												  // a sessao critica

        if (!acquired) {
            returnVal = false;
        } else {
            if (bufferSize < res->size) {
                res->resourceSemaphore.release();
                returnVal = false;
            } else {
                memcpy(buffer, res->data, res->size);
                res->resourceSemaphore.release();
                returnVal = true;
            }
        }
    }

    // Se o 'index' foi -1, exit_critical_section ja foi chamado.
    // Se o 'index' foi valido mas 'acquired' foi false, exit_critical_section ja foi chamado.
    // Se o 'index' foi valido e 'acquired' foi true, exit_critical_section ja foi chamado.
    return returnVal;
}

////////
// brief: Escreve dados de um buffer fornecido para um recurso compartilhado.
//        Adquire o semaforo do recurso para garantir exclusao mutua durante a escrita.
//        A validacao do ID e a obtencao do ponteiro para o recurso sao protegidas
//        por uma secao critica global para atomicidade.
//
// input:
// id (ResourceID): O ID "falso" do recurso a ser escrito.
// newData (const uint8_t*): Ponteiro para os dados a serem escritos no recurso.
// dataSize (size_t): O tamanho dos dados a serem escritos. Nao deve exceder o tamanho do recurso.
//
// return: bool: true se a escrita foi bem-sucedida, false caso contrario.
////////
bool SharedMemoryManager::writeResource(ResourceID id, const uint32_t* newData, size_t dataSize) {
    bool returnVal = false;
    SharedResource* res = nullptr;
    int index = -1;

    index = findResourceIndexByID(id);
    if (index == -1) {
        returnVal = false;
    } else {
        res = &resourcePool[index]; // Obtem o ponteiro para o recurso
        bool acquired = res->resourceSemaphore.acquire(); // mesmo caso do anterior

        if (!acquired) {
            returnVal = false;
        } else {
            if (dataSize > res->size) {
                res->resourceSemaphore.release();
                returnVal = false;
            } else {
                memcpy(res->data, newData, dataSize);
                res->resourceSemaphore.release();
                returnVal = true;
            }
        }
    }

    return returnVal;
}

} // namespace rtos
