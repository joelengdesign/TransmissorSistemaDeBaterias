Código feito no Visual Studio Code por Joel Alison Ribeiro Carvalho - Doutorando em Engenharia Elétrica da Universidade Federal do Pará.

Última data de atualização do código: 23 de Março de 2023.

# Transmissão de dados da aplicação Sistema de Baterias através dos protocolos MODBUS e LoRa

## Aspectos Gerais do Projeto

O Sistema de Baterias é a aplicação do Smart Campus da UFPA que armazena energia para abastacimento do prédio CEAMAZON em possíveis blackouts. O objetivo deste código é extrair informações desta aplicação que estão armazenadas na UACT CC. Esta extração de dados é feita mediante o protocolo MODBUS. Uma vez que as informações são extraídas, estas devem ser enviadas criptografadas por LoRa.

----

## Serialização de dados

A serialização de dados consiste em juntar todas as variáveis em um único array de bytes de forma que os 5 primeiros bytes do vetor é utilizado para armazenar o identificador da aplicação (posição 0 do vetor), e o timestamp (posições 1, 2, 3, 4 do vetor). Da posição 5 em diante vem as mensagens úteis da aplicação. Esta informação que deve ser enviada por LoRa até que cada variável seja monitorada no servidor Dojot.

## Extração de dados MODBUS

A extração de dados da UACT CC por MODBUS é realizada acessando os registradores de 2 bytes, onde estão armazenadas as variáveis da aplicação Sistema de Baterias. O protocolo consiste em enviar um vetor de bytes (Requisição) para a UACT CC, que obedece alguns requisitos. Este vetor de 8 bytes contém as seguintes informações:

- posição 0: Identificador do dispositivo UACT CC que é 0x02;
- posição 1: Código da função que neste caso é 0x04 (para acessar os registradores do tipo input);
- posição 2 e 3: Endereço do registrador inicial;
- posição 4 e 5: Quantidade de registradores a partir do registrador inicial;
- posição 6 e 7: ErrorCheck que identifica a integridade da mensagem de requisição.

A mensagem (vetor) de resposta que a UACT CC envia ao Arduino contém as informações:

- posição 0: Identificador do dispositivo UACT CC que é 0x02;
- posição 1: Código da função que neste caso é 0x04 (para acessar os registradores do tipo input);
- posição 2: Quantidade de bytes da mensagem útil;
- posição 3 até n-3: Mensagem útil, de forma que as variáveis floats estão armazenadas a cada 4 bytes.
- posição n-1 e n: ErrorCheck que identifica a integridade resposta que o Arduino recebe.

em que n indica a última posição do vetor de resposta.

----

## Big Endian

Como cada variável float possui 4 bytes e cada registrador comporta 2 bytes, é notório afirmar que cada variável float está armazenada em 2 registradores. A forma que a UACT CC envia os dados de mensagem útil é no formato Big Endian, de forma que as informações contidas nos registradores são lidos da direita para a esquerda. A posição dos bytes de cada variável float armazenada nos registradores está na forma 1,2,3,4. Porém quando cada variável float é enviada para o Arduino, os bytes são transmitidos na forma 3,4,1,2. Portanto no código será possível verificar o tratamento de dados que é realizado para corrigir a posição dos bytes.

## Criptografia LoRa

Os dados na arquitetura LoRaWAN são enviados através da criptografia AES128 que contém as chaves NwsKey e AppSKey. Estas chaves devem ser as mesmas no transmissor e no componente da rede que se deseja recuperar a informação.

----

## Dados da aplicação

A aplicação do Sistema de Baterias possui ao todo 9 variáveis float com acréscimo do timestamp e o identificador da aplicação que deve ser acrescentado no início do pacote. O código que deve ser inserido no início do pacote que identifica a aplicação é o 0x04. O timestamp está armazenado nos registradores 0x04 e 0x05; enquanto que as variáveis da aplicação estão armazenadas nos registradores entre 0x0C e 0x1D. Porém as 3 últimas variáveis que constam na lista da aplicação do Sistema de Baterias, a UACT CC não transmite.

## Biblioteca MODBUS

Uma biblioteca MODBUS foi criada para comunicação MODBUS do Arduino com a UACT CC. Esta biblioteca possui os seguintes parâmetros:

- MODBUS::MODBUS(int RS485Comunicacao, int led): este método consiste em inicializar a biblioteca com o pino de comunicação RS485 para controle de leitura e escrita entre Arduino e UACT CCC. Em que a porta que controla este pino deve ser escolhida pelo projetista. O led é utilizado para teste visual de leitura e escrita.
- void MODBUS::EnviarPacote(byte EnderecoDoDispositivo, byte CodigoDaFuncao, uint16_t EnderecoInicial, uint16_t QuantidadeDeRegistradores): Método público envia a requisição para a UACT CC com os parâmetros essenciais para comunicação MODBUS;
- uint16_t MODBUS::ErrorCheck(byte mensagem[], uint8_t tamanho): Método privado que retorna o ErrorCheck (de 2 bytes) da mensagem, seja ela de requisição (que o Arduino envia) ou de resposta (que o Arduino recebe);
- bool MODBUS::validacaoPacote(byte pacote[]): Método que retorna um valor booleano de acordo com o ErrorCheck calculado, relacionado a mensagem que o Arduino recebe da UACT CC.
