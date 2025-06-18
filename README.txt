Para essa entrega foram adicionadas mais duas funcionalidades, que estão implementadas nos arquivos OS_aperiodicServer e OS_resourceManager.

O servidor aperiódico escolhido para a implementação foi o Deferrable Server e o protocolo de gerenciamento de recursos compartilhados foi o Non-Preemptive-Protocol.

A função main simula uma abstração de duas tarefas periódicas que compartilham entre sí uma variável, onde o gerenciamento de recurso se faz necessário para evitar a manipulação da variável dentro da região crítica.

Foi necessário alterar o escalonador entregue na atividade anterior para uma política de Rate Monotonic, mas a estrutura usada para a implementação do EDF foi preservada por ser uma implementação bastante genérica e que possibilitou poucas alterações para a troca de política.

A abordagem utilizada para disparar uma requisição de tarefa aperiódica foi usando interrupção gerada pelo botão integrado no microcontrolador. A configuração do botão bem como a redefinição do handler e do call-back associado ao GPIO está implementada na main.cpp.

Novamente, entendo que o uso de variáveis e funções globais é desnecessário e redundante, mas considerei manter uma implementação organizada para facilitar meu compreendimento do conteúdo e tornar a correção um pouco mais simples.

É possível acompanhar a política de escalonamento e o funcionamento do servidor aperiódico adicionando breakpoints no incremento da variável compartilhada e da variável requisição1 mediante pressionamento do botão.

O código pode ser gerado e compilado normalmente através da IDE.

Obrigado pela atenção! 