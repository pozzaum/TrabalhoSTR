#ifndef OS_RESOURCEMANAGER_H
#define OS_RESOURCEMANAGER_H


#include <cstdint> // Para uint8_t, uint32_t, size_t
#include <stdbool.h> // Para bool
#include <cstddef>

extern "C"{
	#include "cmsis_compiler.h"
}

namespace rtos {

// Definicao do numero maximo de recursos compartilhados
// Ajuste este valor conforme a necessidade da sua aplicacao.
const uint8_t MAX_SHARED_RESOURCES = 10;

// Definicao do tamanho maximo da string de tipo de recurso
const uint8_t MAX_RESOURCE_TYPE_LEN = 16;

////////
// brief: Tipo para o identificador "falso" de um recurso.
//        Este ID e usado por tarefas externas para referenciar um recurso
//        compartilhado sem conhecer seu endereco de memoria real.
////////
using ResourceID = uint32_t;

////////
// brief: Entra em uma secao critica, desabilitando as interrupcoes globais.
//        Usada para proteger operacoes atomicas que nao podem ser interrompidas.
//
// input: (nenhum)
//
// return: (nenhum)
////////
inline void enter_critical_section() {
    __disable_irq(); // Desativa todas as interrupcoes
}

////////
// Sai de uma secao critica, reabilitando as interrupcoes globais.
//        Deve ser chamada apos uma enter_critical_section().
//
// input: (nenhum)
//
// return: (nenhum)
////////
inline void exit_critical_section() {
    __enable_irq();  // Reativa todas as interrupcoes
}

////////
// brief: Implementa um semaforo binario simples usando uma flag e secoes criticas.
//        Garante exclusao mutua para um recurso especifico.
////////
struct SimpleSemaphore {
    volatile uint8_t available; // 1 = disponivel, 0 = ocupado. 'volatile' e crucial!

    ////////
    // brief: Inicializa o semaforo, definindo-o como disponivel.
    //
    // input: (nenhum)
    //
    // return: (nenhum)
    ////////
    void init() {
        available = 1; // Inicialmente disponivel
    }

    ////////
    // brief: Tenta adquirir o semaforo.
    //        Se o semaforo estiver disponivel, ele e adquirido e a funcao retorna true.
    //        Caso contrario, retorna false imediatamente (nao bloqueia a tarefa).
    //        A atomicidade e garantida por desativacao/reativacao de interrupcoes.
    //
    // input: (nenhum)
    //
    // return: bool: true se o semaforo foi adquirido com sucesso, false caso contrario.
    ////////
    bool acquire() {
        enter_critical_section();
        bool returnVal = false;
        if (available == 1) {
            available = 0; // Ocupa o recurso
            returnVal = true;
        }
        exit_critical_section();
        return returnVal;
    }

    ////////
    // brief: Libera o semaforo, tornando o recurso disponivel novamente.
    //        A atomicidade e garantida por desativacao/reativacao de interrupcoes.
    //
    // input: (nenhum)
    //
    // return: (nenhum)
    ////////
    void release() {
        enter_critical_section();
        available = 1; // Libera o recurso
        exit_critical_section();
    }
};

////////
// brief: Estrutura para representar um recurso compartilhado.
//        Contem informacoes sobre o tamanho, tipo e os dados do recurso,
//        alem de um semaforo para controle de acesso.
////////
struct SharedResource {
    ResourceID id;       // O ID "falso" do recurso (para busca linear)
    size_t size;         // Tamanho do recurso em bytes
    char type[MAX_RESOURCE_TYPE_LEN]; // Tipo do recurso (e.g., "BUFFER", "VAR_INT")
    uint8_t* data;       // Ponteiro para os dados reais do recurso na memoria
    SimpleSemaphore resourceSemaphore; // Semaforo para proteger o acesso a este recurso especifico
    bool inUse;          // Flag para indicar se a entrada no array de recursos esta em uso (para alocacao)
};

////////
// brief: Classe para gerenciar a criacao, destruicao, leitura e alteracao
//        de recursos de memoria compartilhada entre diferentes tarefas.
//        Utiliza IDs "falsos" para abstrair enderecos de memoria reais e
//        semaforos para garantir a seguranca e a exclusao mutua no acesso.
//        Implementacao sem STL para uso em sistemas embarcados restritos.
////////
class SharedMemoryManager {
public:
    ////////
    // brief: Construtor da classe SharedMemoryManager.
    //        Inicializa o pool de recursos e o contador de IDs.
    //
    // input: (nenhum)
    //
    // return: (nenhum)
    ////////
    SharedMemoryManager();

    ////////
    // brief: Destrutor da classe SharedMemoryManager.
    //        Responsavel por liberar toda a memoria alocada para os recursos
    //        compartilhados ativos. Garante que recursos em uso nao sejam
    //        destruidos prematuramente, se possivel.
    //
    // input: (nenhum)
    //
    // return: (nenhum)
    ////////
    ~SharedMemoryManager();

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
    bool manageResource(ResourceID& id, const char* operation, size_t size = 0,
                        const char* type = nullptr, uint32_t* dataBuffer = nullptr);

private:
    SharedResource resourcePool[MAX_SHARED_RESOURCES]; // Array estatico para armazenar os recursos
    uint8_t currentResourceCount; // Contador de recursos atualmente em uso no pool
    ResourceID nextResourceID; // Proximo ID de recurso disponivel globalmente


    ////////
    // brief: Encontra um slot vazio no resourcePool para alocar um novo recurso.
    //
    // input: (nenhum)
    //
    // return: int: O indice do slot vazio encontrado, ou -1 se nao houver slots disponiveis.
    ////////
    int findEmptyResourceSlot();

    ////////
    // brief: Encontra o indice de um recurso no resourcePool dado seu ResourceID.
    //
    // input:
    // id (ResourceID): O ID "falso" do recurso a ser procurado.
    //
    // return: int: O indice do recurso no resourcePool, ou -1 se o ID nao for encontrado.
    ////////
    int findResourceIndexByID(ResourceID id) const;


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
    bool createResource(ResourceID& newID, size_t size, const char* type);

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
    bool destroyResource(ResourceID id);

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
    bool readResource(ResourceID id, uint32_t* buffer, size_t bufferSize);

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
    bool writeResource(ResourceID id, const uint32_t* newData, size_t dataSize);

    // As funcoes isValidResourceID e getRealResourceIndex serao substituídas por findResourceIndexByID

};


} // namespace rtos

#endif // SHARED_MEMORY_MANAGER_H
