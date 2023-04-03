// Código de transmissão de dados da aplicação SISTEMA DE BATERIAS relacionada ao Smart Campus da UFPA
// código editado em 03 de abril de 2023

#include <Arduino.h>
#include <lmic.h> // biblioteca lmic para transmissão LoRa
#include <hal/hal.h>
#include <MODBUS.h> // biblioteca MODBUS criada para extração de dados da UACT CC
#include <TimeLib.h> // biblioteca para converter timestamp no formato unix para hora e data
#include <softwareReset.hpp> // biblioteca para resetar o microcontrolador

MODBUS modbus(23,LED_BUILTIN);

// chaves de acesso AES128 da criptografia LoRaWAN
static const PROGMEM u1_t NWKSKEY[16] = {/* Informação confidencial */}; //projeto

static const u1_t PROGMEM APPSKEY[16] = {/* Informação confidencial */}; //projeto

static const u4_t DEVADDR = 0x260138A2 ; // projeto <-- Change this address for every node!

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

const unsigned TX_INTERVAL = 4;
// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
};

const byte aplicacao = 0x04; // código do sistema de baterias

// parâmetros do protocolo MODBUS
const byte DeviceAdress = 0x02; // endereço do dispositivo - UACT CC
const byte TypeRegisters = 0x04; // código de acesso dos registradores do tipo input
const uint16_t InitAdress = 0x0004; // endereço do registrador inicial
const uint16_t QuantRegisters = 0x001A; // quantidade de registradores

float TensaoBarramento;
float Corrente1;
float Corrente2;
float Corrente3;
float Corrente4;
float Corrente5;
float Corrente6;
float Corrente7;
float Corrente8;
float PotenciaCC;
float EnergiaFornecida;
float EnergiaConsumida;


// parâmetros dos dados recebidos da UAC CC
struct UACTStruct
{
  byte packetReceived[57];
  byte receivedByteIndex = 0;
  byte PkgTotal[53];
};
UACTStruct uactComponent;

// função que é chamada sempre que existem dados na serial do arduino para serem lidos
void serialEvent1(){
  byte incomingByte;
  // Serial.println(stillWaitNextBit);
  if (modbus.stillWaitNextBit)
  {
    while (Serial1.available()) {
      incomingByte = Serial1.read();
      // Serial.print(incomingByte, HEX);
      // Serial.print(' ');
      uactComponent.packetReceived[uactComponent.receivedByteIndex] = incomingByte;
      uactComponent.receivedByteIndex += 1;
    }
  }
}

// função para controlar o tempo entre requisição e resposta à UACT CC
void espera(unsigned long tempo) {
  unsigned long inicio = millis();
  while (millis() - inicio < tempo) {
    serialEvent1(); // Execute a função serialEvent1 durante o tempo de espera
  }
}

// função que limpa o payload de resposta da UACT CC
void clearReceivedPackage(){
  for (byte i = 0; i < sizeof(uactComponent.packetReceived); i++) uactComponent.packetReceived[i] = 0x0;
  uactComponent.receivedByteIndex = 0;
}

// função que limpa o payload de transmissão com todos os dados tratados
void clearReceivedPackageTotal(){
  for (byte i = 0; i < sizeof(uactComponent.PkgTotal); i++) uactComponent.PkgTotal[i] = 0x0;
}

// função que organiza os dados vindos da UACT CC e serializa com o timestamp e código da aplicação
void OrganizePkg(){
  modbus.stillWaitNextBit = false;
  bool valid = modbus.validacaoPacote(uactComponent.packetReceived);

  if(valid){ // só entra neste bloco se o CRC do pacote de resposta da UACT CC estiver correto
    uactComponent.PkgTotal[0] = aplicacao;

    // timestamp
    uactComponent.PkgTotal[1] = uactComponent.packetReceived[5];
    uactComponent.PkgTotal[2] = uactComponent.packetReceived[6];
    uactComponent.PkgTotal[3] = uactComponent.packetReceived[3];
    uactComponent.PkgTotal[4] = uactComponent.packetReceived[4];
    // variaveis de interesse
    for (byte i = 5; i<41; i=i+4){
      // i=40
      uactComponent.PkgTotal[i] = uactComponent.packetReceived[i+16]; //   21-5
      uactComponent.PkgTotal[i+1] = uactComponent.packetReceived[i+17]; // 22-5
      uactComponent.PkgTotal[i+2] = uactComponent.packetReceived[i+14]; // 19-5
      uactComponent.PkgTotal[i+3] = uactComponent.packetReceived[i+15]; // 20-5
    }

    // preenchendo com zero as variáveis com zeros
    for (byte i=41; i<53; i++){
      uactComponent.PkgTotal[i] = 0x00;
    }
  }
  else{ // se o pacote não estiver correto reseta o arduino
    Serial.println("Pacote inválido");
    softwareReset::standard();
  }
  clearReceivedPackage();
}

void espera_min(unsigned long time_min) {
  unsigned long time_ms = time_min * 60000UL; // converte minutos para milissegundos
  unsigned long start_time = millis();
  unsigned long end_time = start_time + time_ms;
  
  while (millis() < end_time) {
    // aguarda até que o tempo final seja alcançado
  }
}

void printarVars(){

    // timestamp
    unsigned long timestamp = ((unsigned long)uactComponent.PkgTotal[1] << 24) |  // Desloca o primeiro byte 24 bits para a esquerda
                          ((unsigned long)uactComponent.PkgTotal[2] << 16) |  // Desloca o segundo byte 16 bits para a esquerda
                          ((unsigned long)uactComponent.PkgTotal[3] << 8)  |  // Desloca o terceiro byte 8 bits para a esquerda
                          ((unsigned long)uactComponent.PkgTotal[4]);        // Não precisa deslocar

    setTime(timestamp);
    
    int dia = day();
    int mes = month();
    int ano = year();
    int hora = hour();
    int minuto = minute();

    // variaveis
    TensaoBarramento = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,5,6,7,8);
    Corrente1 = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,9,10,11,12);
    Corrente2 = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,13,14,15,16);
    Corrente3 = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,17,18,19,20);
    Corrente4 = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,21,22,23,24);
    Corrente5 = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,25,26,27,28);
    Corrente6 = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,29,30,31,32);
    Corrente7 = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,33,34,35,36);
    Corrente8 = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,37,38,39,40);
    PotenciaCC = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,41,42,43,44);
    EnergiaFornecida = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,45,46,47,48);
    EnergiaConsumida = modbus.IEEE754_HexToFloat(uactComponent.PkgTotal,49,50,51,52);

    // Imprime a data e hora obtidas
    if(dia<10){
      Serial.print("0");
      Serial.print(dia);
    }
    else{
      Serial.print(dia);
    }
    Serial.print("/");
    if(mes<10){
      Serial.print("0");
      Serial.print(mes);
    }
    else{
      Serial.print(mes);
    }
    Serial.print("/");
    Serial.println(ano);
    if(hora<10){
      Serial.print("0");
      Serial.print(hora);
    }
    else{
      Serial.print(hora);
    }
    Serial.print("h");
    if(hora<10){
      Serial.print("0");
      Serial.print(minuto);
    }
    else{
      Serial.print(minuto);
    }
    Serial.println("min");

    Serial.print("Tensão:\t\t\t");
    Serial.println(TensaoBarramento);

    Serial.print("Corrente 1:\t\t");
    Serial.println(Corrente1);

    Serial.print("Corrente 2:\t\t");
    Serial.println(Corrente2);

    Serial.print("Corrente 3:\t\t");
    Serial.println(Corrente3);

    Serial.print("Corrente 4:\t\t");
    Serial.println(Corrente4);

    Serial.print("Corrente 5:\t\t");
    Serial.println(Corrente5);

    Serial.print("Corrente 6:\t\t");
    Serial.println(Corrente6);

    Serial.print("Corrente 7:\t\t");
    Serial.println(Corrente7);

    Serial.print("Corrente 8:\t\t");
    Serial.println(Corrente8);

    Serial.print("Potencia:\t\t");
    Serial.println(PotenciaCC);

    Serial.print("Energia Fornecida:\t");
    Serial.println(EnergiaFornecida);

    Serial.print("Energia Consumida:\t");
    Serial.println(EnergiaConsumida);
}

// Função de envio do pacote por LoRa.
void do_send(osjob_t* j) {

  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println("OP_TXRXPEND, not sending");
  }
  else {
    // acessando o timestamp nos registradores holding
    clearReceivedPackage();
    modbus.EnviarPacote(DeviceAdress,TypeRegisters,InitAdress,QuantRegisters); // envia a requisição
    espera(300); // aguarda 300 milissegundos
    // Serial.print("Pacote Recebido: ");
    // modbus.printar(uactComponent.packetReceived,sizeof(uactComponent.packetReceived)); // printa o pacote recebido da UACT CC
    OrganizePkg(); // organiza o pacote para transmissão LoRa
    Serial.println("");
    Serial.print("Pacote para envio: ");
    modbus.printar(uactComponent.PkgTotal,sizeof(uactComponent.PkgTotal)); // printa o pacote organizado

    printarVars();

    LMIC_setTxData2(1, uactComponent.PkgTotal, sizeof(uactComponent.PkgTotal), 0); // envia o pacote organizado por LoRa
    Serial.println("Pacote enviado");
    clearReceivedPackageTotal(); // limpa o pacote de envio para não transmitir a informação anterior, quando houver erros no pacote
    espera_min(4);
  }
}
void onEvent (ev_t ev) {

  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT")); break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND")); break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED")); break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED")); break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING")); break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED")); break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1")); break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED")); break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED")); break;
    case EV_TXCOMPLETE:

      Serial.print(getSf(LMIC.rps));
      if (LMIC.dataLen) {
        // data received in rx slot after tx
        Serial.print("Data Received: ");
        Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
        Serial.println();
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}


void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);

  while (!Serial);

  delay(100);
  Serial.println("Sistema de Baterias. Enviando Dados...");
  Serial.println("---------------------------------------------");

#ifdef VCC_ENABLE
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif
  
  os_init();

  LMIC_reset();

#ifdef PROGMEM

  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else

  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif
  LMIC_selectSubBand(1);

  LMIC_setLinkCheckMode(0);

  LMIC_setAdrMode(0);

  /* esta função define a configuração LoRa de transmissão com o Spreading Factor (SF) e a potência de transmissão
  É importante destacar que esta função procura a melhor configuração de envio, de acordo com o tamanho do pacote
  Ela irá enviar no maior SF possível que suporte o  tamanho do pacote enviado, já que quanto maior o SF, maior alcance
  e qualidade na transmissão terá */
  LMIC_setDrTxpow(DR_SF10, 20);

  Serial.println(LMIC.datarate);
  do_send(&sendjob); // chama a função que executará o envio do pacote por LoRa
}

void loop() {
  os_runloop_once();
}