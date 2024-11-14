
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Arquivo:   IO NET DASHBOARD
//Empresa:   Brave Lion Automação e IoT
//Autor:     Leonardo Borges
//Versão:    V2.0
//Data:      AGOSTO / 2024
//Placa:     ESP32 DEVKIT V1
//#############################################################################################################

// =============================================================================================================
/*
  Equivalencia das saidas Digitais entre NodeMCU e ESP32 (na IDE do Arduino)
  
  D4  = 24; SENSOR TEMPERATURA 1
  D2  = 22; SENSOR TEMPERATURA 2
  D34 = 10; BOTÃO ACIONAMENTO MANUAL
  D25 = 14; SAIDA 1 BOMBA
  D26 = 15; SAIDA 2 RESISTENCIA ELETRICA
  D27 = 16; SAIDA 3 ANEL CIRCULAÇÃO
 
 
*/
//#############################################################################################################

// =============================================================================================================
//                                       BIBLIOTECAS
// =============================================================================================================
#include <Arduino.h>           // Biblioteca para programa arduino rodar no VSCODE
#include <FS.h>                // Esta precisa ser a primeira referência
#include <DNSServer.h>         // biblioteca servidor
#include <WiFi.h>              //inclui biblioteca WiFi
#include <WiFiManager.h>       // Biblioteca WIFIMANAGER
#include <PubSubClient.h>      // Biblioteca MQTT
#include <EEPROM.h>            // Biblioteca para usar memoria EEPROM ESP8266
#include <Wire.h>              // Biblioteca pino serial
#include <OneWire.h>           // BIBLIOTECA PARA SENSOR DE TEMPERATURA DS18B20
#include <DallasTemperature.h> // BIBLIOTECA PARA SENSOR DE TEMPERATURA DS18B20
#include <TimeLib.h>           // BIBLIOTECA TIME
#include <WiFiUdp.h>           // BIBLIOTECA TIME INTERNET
#include <Time.h>              // BIBLIOTECA TIME 

//#############################################################################################################

// =============================================================================================================
//                              DEFINIÇÃO DE PARAMETROS LOGIN SERVIDOR MQTT
// =============================================================================================================

#define servidor_mqtt         "remarkable-architect.cloudmqtt.com" //URL do servidor MQTT      
#define servidor_mqtt_porta   "1883"                               //Porta do servidor (a mesma deve ser informada na variável abaixo)
#define servidor_mqtt_usuario "lkwlizre"                           //Usuário
#define servidor_mqtt_senha   "Qc2kKeVpwHjr"                       //Senha
#define servidor_mqtt_id      "TESTEH"                             //ID USUARIO

//#############################################################################################################



// =============================================================================================================
//                                   DEFINIÇÃO DE VARIAVEIS GLOBAIS
// =============================================================================================================

#define mqtt_topico_sub         "TESTE/control"        //Tópico para subscrever
#define mqtt_topico_status      "TESTE/status"         //Tópico para subscrever
#define mqtt_topico_info        "TESTE/info"           //Tópico para subscrever
#define mqtt_topico_temp        "TESTE/temp"           //Tópico para subscrever
#define mqtt_topico_settemp     "TESTE/settemp"        //Tópico para subscrever
#define mqtt_topico_setrange    "TESTE/setrange"       //Tópico para subscrever
#define mqtt_topico_settime     "TESTE/settime"        //Tópico para subscrever
#define mqtt_topico_settempmax  "TESTE/settempmax"     //Tópico para subscrever
#define mqtt_topico_lasttempmin "TESTE/lastTempMin"    //Tópico para subscrever
#define mqtt_topico_lasttempmax "TESTE/lastTempMax"    //Tópico para subscrever
#define mqtt_topico_totalaction "TESTE/totalAction"    //Tópico para subscrever
#define mqtt_topico_ev1         "TESTE/ev1"            //Tópico para subscrever
#define mqtt_topico_ev2         "TESTE/ev2"            //Tópico para subscrever
#define mqtt_topico_ev3         "TESTE/ev3"            //Tópico para subscrever
#define mqtt_topico_ev4         "TESTE/ev4"            //Tópico para subscrever
#define mqtt_topico_timestart1  "TESTE/timestart1"     //Tópico para subscrever
#define mqtt_topico_timestart2  "TESTE/timestart2"     //Tópico para subscrever
#define mqtt_topico_timestart3  "TESTE/timestart3"     //Tópico para subscrever
#define mqtt_topico_timestart4  "TESTE/timestart4"     //Tópico para subscrever
#define mqtt_topico_timeend1    "TESTE/timeend1"       //Tópico para subscrever
#define mqtt_topico_timeend2    "TESTE/timeend2"       //Tópico para subscrever
#define mqtt_topico_timeend3    "TESTE/timeend3"       //Tópico para subscrever
#define mqtt_topico_timeend4    "TESTE/timeend4"       //Tópico para subscrever
#define memoria_alocada 128                            //Define o quanto sera alocado na EEPROM 
#define bomb 25                                        //SAIDA DIGITAL ACIONAMENTO BOMBA
#define res 26                                         //SAIDA DIGITAL ACIONAMENTO RESISTENCIA
#define circ 27                                        //SAIDA DIGITAL ACIONAMENTO ANEL CIRCULAÇÃO
#define bot_manu 14                                    //ENTRADA DIGITAL BOTAO COMANDO MANUAL
#define DEBUG                                          //HABILITA FUNÇÃO SERIAL PRINT
//#define wificonected 2


//#############################################################################################################



// =============================================================================================================
//                                   LOGIN E SENHA ACESSO WIFI LOCAL
// =============================================================================================================

const char *ssid     = "thermiccontroller";  //NOME WIFI A SER CONECTADO
const char *password = "thermiccontroller";  //SENHA WIFI A SER CONECTADO

//#############################################################################################################




// =============================================================================================================
//                                       PARAMETROS DAS BIBLIOTECAS
// =============================================================================================================
WiFiClient espClient;                                //Instância do WiFiClient
PubSubClient client(espClient);                      //Passando a instância do WiFiClient para a instância do PubSubClient
OneWire barramento_1(4);                            // DEFINIÇÃO PINO LEITURA SENSOR 1 DE TEMPERATURA
OneWire barramento_2(2);                            // DEFINIÇÃO PINO LEITURA SENSOR 2 DE TEMPERATURA
DallasTemperature sensor_1(&barramento_1);           // DEFINIÇÃO VARIAVEL PARA BIBLIOTECA SENSOR DE TEMPERATURA
DallasTemperature sensor_2(&barramento_2);           // DEFINIÇÃO VARIAVEL PARA BIBLIOTECA SENSOR DE TEMPERATURA
static const char ntpServerName[] = "pool.ntp.br";   // SERVIDOR TIME HORA
const int timeZone = -3;                             // PARAMETRO HORA BRASIL
WiFiUDP Udp;                                         // FUNÇÃO UDP
unsigned int localPort = 8888;                       // local port to listen for UDP packets
WiFiServer server(80);                               //define a porta que o servidor irá utilizar
                                          
//#############################################################################################################


// =============================================================================================================
//                                       VARIAVEIS GLOBAIS
// =============================================================================================================

String mxpt ;
String men_status = "0";
String hora = "00:00";
bool precisaSalvar = false;       // Flag para salvar os dados
bool Leitura_set_ok = false;      // Apos desligamento executa leitura d memoria eeprom
bool en_start = false;            // START
bool en_aque = false;             // ENABLE AQUECEDOR
bool en_res = false;              // ENABLE RESEISTENCIA
bool en_set = false;              // ENABLE SETUP VIA APP
bool bomb_ant = false;            // STATUS ANTERIOR BOMBA
bool res_ant = false;             // STATUS ANTERIOR RESISTENCIA
bool publicar = false;            // PEDIDO DE PUBLICAÇÃO
bool x_publicar = false;          // auxiliar PEDIDO DE PUBLICAÇÃO
bool x_pub_1 = false;             // auxiliar PEDIDO DE PUBLICAÇÃO
bool x_pub_2 = false;             // auxiliar PEDIDO DE PUBLICAÇÃO
bool x_pub_3 = false;             // auxiliar PEDIDO DE PUBLICAÇÃO
bool x_pub_4 = false;             // auxiliar PEDIDO DE PUBLICAÇÃO
bool x_pub_5 = false;             // auxiliar PEDIDO DE PUBLICAÇÃO
bool x_pub_6 = false;             // auxiliar PEDIDO DE PUBLICAÇÃO
bool x_pub_7 = false;             // auxiliar PEDIDO DE PUBLICAÇÃO
bool x_pub_8 = false;             // auxiliar PEDIDO DE PUBLICAÇÃO
bool x_pub_9 = false;             // auxiliar PEDIDO DE PUBLICAÇÃO
bool x_pub_10 = false;            // auxiliar PEDIDO DE PUBLICAÇÃO
bool x_pub_11 = false;            // auxiliar PEDIDO DE PUBLICAÇÃO
bool x_pub_12 = false;            // auxiliar PEDIDO DE PUBLICAÇÃO
bool x_pub_13 = false;            // auxiliar PEDIDO DE PUBLICAÇÃO
bool x_pub_14 = false;            // auxiliar PEDIDO DE PUBLICAÇÃO
bool x_pub_15 = false;            // auxiliar PEDIDO DE PUBLICAÇÃO
bool x_pub_16 = false;            // auxiliar PEDIDO DE PUBLICAÇÃO
bool x_pub_status = false;        // auxiliar PEDIDO DE PUBLICAÇÃO
bool gravatemp = false;           // PEDIDO GRAVACAO SETPOINT TEMP NA EEPROM
bool gravarange = false;          // PEDIDO GRAVACAO SETPOINT RANGE NA EEPROM
bool gravarsettempmax = false;    // PEDIDO GRAVAÇÃO MEMORIA NA EEPROM
bool gravarRtempmin = false;      // PEDIDO GRAVAÇÃO MEMORIA NA EEPROM
bool gravarRtempmax = false;      // PEDIDO GRAVAÇÃO MEMORIA NA EEPROM
bool gravarRaction = false;       // PEDIDO GRAVAÇÃO MEMORIA NA EEPROM
bool gravarAgenda = false;        // PEDIDO GRAVAÇÃO MEMORIA NA EEPROM
bool gravarbits = false;          // PEDIDO GRAVAÇÃO MEMORIA NA EEPROM
bool gravartime = false;          // PEDIDO GRAVAÇÃO MEMORIA NA EEPROM
bool aux_T = false;               // auxiliar time
bool com = false;                 // comunicação
bool call_reconect = false;       // chamada apra reconecção
bool xev1 = false;                // x evento 1
bool xev2 = false;                // x evento 2
bool xev3 = false;                // x evento 3
bool xev4 = false;                // x evento 4
bool xmbomb = false;              // x habilita manual aquecedor
bool xmres = false;               // x habilita manual apoio eletrico
bool xmstart = false;             // x Manual start
bool xt = false;                  // MEMORIA AUXILIAR
bool refresh = false;             // MEMORIA AUXILIAR
bool x1start = false;             // Auxiliar start evento 1
bool x2start = false;             // Auxiliar start evento 2
bool x3start = false;             // Auxiliar start evento 3
bool x4start = false;             // Auxiliar start evento 4
bool actionon = false;            // Auxiliar
bool xenter   = false;            // Auxiliar
bool  okev1 = false, okev2 = false, okev3 = false, okev4 = false;   // Auxiliar
uint16_t status_settemp = 0;      // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint16_t status_setrange = 0;     // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint16_t status_tempmax = 0;      // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint16_t status_Rtempmin = 0;     // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint16_t status_Rtempmax = 0;     // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint16_t status_time = 0;         // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint16_t status_Raction = 0;      // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint16_t status_x1start = 0;      // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint16_t status_x2start = 0;      // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint16_t status_x3start = 0;      // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint16_t status_x4start = 0;      // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint16_t status_x1end = 0;        // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint16_t status_x2end = 0;        // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint16_t status_x3end = 0;        // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint16_t status_x4end = 0;        // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint8_t status_aq = false;        // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint8_t status_res = false;       // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint8_t status_xmbomb = false;    // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint8_t status_xmres = false;     // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint16_t status_ = 0;             // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint8_t status_ev1 = 0;           // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint8_t status_ev2 = 0;           // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint8_t status_ev3 = 0;           // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
uint8_t status_ev4 = 0;           // Variável que armazenará o status da variavel  que foi gravado anteriormente na EEPROM
int menu_num = 1;                 // Menu de seleção
int sub_menu = 1;                 // SubMenu de Seleção
int settemp = 0;                  // Setpoint temperatura minima
int setrange = 0;                 // Setpoint temperatura maxima
int aux_settemp = 0;              // Auxiliar para setpoint temperatura minima
int aux_setrange = 1;             // Auxiliar para setpoint temperatura minima
int temp = 0;                     // Recebe valor sensor de temperatura
int tempS2 = 0;                   // Recebe valor sensor 2 de temperatura
int temp_ant = 0;                 // STATUS ANTERIOR SENSOR TEMPERATURA
int menu = 1;                     // Valor para seleção do Menu
int xsettempmax = 60;             // Valor maximo Temperatura
int xRtempmin = 0;                // Valor Resgistro minimo Temperatura
int xRtempmax = 0;                // Valor Registro maximo Temperatura
int xRaction = 0;                 // Valor Atuações
int timedelay = 3600000;          // Valor Delay comando manual
int startev1;                     // Start Evento 1
int startev2;                     // Start Evento 2
int startev3;                     // Start Evento 3
int startev4;                     // Start Evento 4
int endev1;                       // End Evento 1
int endev2;                       // End Evento 2
int endev3;                       // End Evento 3
int endev4;                       // End Evento 4
int dayofweek = 0 ;               // Dia da semana
int temp_atual = 0;               // AUZILIAR TEMPERATURA ATUAL
long int horanow;                 // Hora Atual
unsigned long tempo = 0;          // Variável de controle do tempo
unsigned long tempomax = 5400000; // Tempo em ms do intervalo a ser executado
unsigned long xmdelay;            //Tdelay modo manual
unsigned long loginover = 0;      // Auxiliar
unsigned long timereconect = 0;   // Auxiliar
unsigned long time_update_status = 0;    // Auxiliar
bool blockreconect = false ;      // Auxiliar
bool time_b127 = false;           // Auxiliar
time_t prevDisplay = 0;           // when the digital clock was displayed

//const char* ntpServer ="pool.ntp.org";
//const long gmtoffset_sec = -5400;
//const int daylighoffset_sec = -5400;




//#############################################################################################################

// =============================================================================================================
//                                       FUNCOES FUNCIONAMENTO CICLO
// =============================================================================================================

void ciclo();                                 // CICLO FUNCIONAMENTO
void comandos_web();                            // FUNÇÃO ACESSO PELO BROSER
time_t getNtpTime();                          // FUNÇÃO PEGA HORA
void digitalClockDisplay();                   // FUNÇÃO MOSTRA DATA E HORA
void printDigits(int digits);                 // FUNÇÃO IMPRIME SERIAL
void sendNTPpacket(IPAddress &address);       // FUNÇÃO AUXILIAR DATA E HORA
//#############################################################################################################

void digitalClockDisplay()
{
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(".");
  Serial.print(month());
  Serial.print(".");
  Serial.print(year());
  Serial.println();
}

void printDigits(int digits)
{
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48;     // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0)
    ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500)
  {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE)
    {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE); // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 = (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011; // LI, Version, Mode
  packetBuffer[1] = 0;          // Stratum, or type of clock
  packetBuffer[2] = 6;          // Polling Interval
  packetBuffer[3] = 0xEC;       // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket("pool.ntp.br", 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}




// =============================================================================================================
//                                           FUNCAO CICLO PRINCIPAL
// =============================================================================================================
void ciclo()
{

  bool bomb_atual = digitalRead(bomb);
  bool res_atual = digitalRead(res);
  
  int temp_plus = settemp + setrange;
  int temp_less = settemp - setrange;

  if (temp < temp_less)
  {

    if (((en_aque) && (en_start)) || ((xmstart) && (xmbomb)))
    {
      digitalWrite(bomb, HIGH);
      digitalWrite(circ, HIGH);
    }
    if (((en_res) && (en_start)) || ((xmstart) && (xmres)))
    {
      digitalWrite(res, HIGH);
    }
  }
  else if (temp > temp_plus)
  {
    digitalWrite(bomb, LOW);
    digitalWrite(res, LOW);
    digitalWrite(circ, LOW);
  }

  if ((((!en_aque) || (!en_start)) && (!xmstart)) || (((!xmbomb) || (!xmstart)) && (!en_start)))
  {
    digitalWrite(bomb, LOW);
    digitalWrite(circ, LOW);
  }
  if ((((!en_res) || (!en_start)) && (!xmstart)) || (((!xmres) || (!xmstart)) && (!en_start)))
  {
    digitalWrite(res, LOW);
  }

  if (bomb_atual != bomb_ant)
  {

    x_pub_1 = true;
    x_pub_12 = true;
    x_publicar = true;
    bomb_ant = bomb_atual;
  }

  if (res_atual != res_ant)
  {

    x_pub_2 = true;
    x_pub_12 = true;
    x_publicar = true;
    res_ant = res_atual;
  }

  if ((temp_atual > (temp_ant + 1)) || (temp_atual < (temp_ant - 1)))
  {
  
        x_pub_1 = true;
        x_pub_2 = true;
        x_pub_12 = true;
        x_pub_3 = true;
        x_publicar = true;
        temp_ant = temp_atual;         


    
  }

  if (x_publicar) 
  {

    publicar = true;
    x_publicar = false;
  }
}
//#############################################################################################################

// =============================================================================================================
//                                       FUNCAO IMPRIMIR NA SERIAL
// =============================================================================================================
void imprimirSerial(bool linha, String mensagem)
{
#ifdef DEBUG
  if (linha)
  {
    Serial.println(mensagem);
  }
  else
  {
    Serial.print(mensagem);
  }
#endif
}
//#############################################################################################################

// =============================================================================================================
//                                       FUNCAO SALVAR
// =============================================================================================================
void precisaSalvarCallback()
{
  imprimirSerial(true, "As configuracoes tem que ser salvas.");
  precisaSalvar = true;
}
//#############################################################################################################

// =============================================================================================================
//                                       FUNCAO RECONECTAR SERVIDOR MQTT
// =============================================================================================================
void reconectar()
{
  //Repete até conectar
  if (!client.connected())
  {
    imprimirSerial(false, "Tentando conectar ao servidor MQTT...");

    //digitalWrite(wificonected, HIGH);
    //delay(100);
    //digitalWrite(wificonected, LOW);
    //delay(100);
    //imprimirSerial(true, "SERVIDOR DESCONECTADO");

    //Tentativa de conectar. Se o MQTT precisa de autenticação, será chamada a função com autenticação, caso contrário, chama a sem autenticação.
    bool conectado = strlen(servidor_mqtt_usuario) > 0 ? client.connect(servidor_mqtt_id, servidor_mqtt_usuario, servidor_mqtt_senha) : client.connect(servidor_mqtt_id); //"ESP8266Client"

    if (conectado)
    {

      //Subscreve para monitorar os comandos recebidos
      client.subscribe(mqtt_topico_sub, 1);        //QoS 1
      client.subscribe(mqtt_topico_settemp, 1);    //QoS 1
      client.subscribe(mqtt_topico_setrange, 1);   //QoS 1
      client.subscribe(mqtt_topico_settime, 1);    //QoS 1
      client.subscribe(mqtt_topico_settempmax, 1); //QoS 1      
      client.subscribe(mqtt_topico_ev1, 1);        //QoS 1
      client.subscribe(mqtt_topico_ev2, 1);        //QoS 1
      client.subscribe(mqtt_topico_ev3, 1);        //QoS 1
      client.subscribe(mqtt_topico_ev4, 1);        //QoS 1
      client.subscribe(mqtt_topico_timestart1, 1); //QoS 1
      client.subscribe(mqtt_topico_timestart2, 1); //QoS 1
      client.subscribe(mqtt_topico_timestart3, 1); //QoS 1
      client.subscribe(mqtt_topico_timestart4, 1); //QoS 1
      client.subscribe(mqtt_topico_timeend1, 1);   //QoS 1
      client.subscribe(mqtt_topico_timeend2, 1);   //QoS 1
      client.subscribe(mqtt_topico_timeend3, 1);   //QoS 1
      client.subscribe(mqtt_topico_timeend4, 1);   //QoS 1
      imprimirSerial(true, "Conectado ao Servidor MQTT!");
      delay(500);
      
    }
    else
    {
      //digitalWrite(wificonected, HIGH);
      //delay(100);
      //digitalWrite(wificonected, LOW);
      //delay(100);
      imprimirSerial(false, "Falhou ao tentar conectar. Codigo: ");
      imprimirSerial(false, String(client.state()).c_str());
      imprimirSerial(true, " tentando novamente em 3 segundos");
      //Aguarda 3 segundos para tentar novamente
      delay(3000);
      //digitalWrite(wificonected, HIGH);
      //delay(100);
      //digitalWrite(wificonected, LOW);
      delay(100);
    }
  }
}
//#############################################################################################################

// =============================================================================================================
//                                       FUNCAO DESCONECTAR SERVIDOR MQTT
// =============================================================================================================
void desconectar()
{
  imprimirSerial(true, "Fechando a conexao com o servidor MQTT...");
  client.disconnect();
}
//#############################################################################################################



// =============================================================================================================
//                                       FUNCAO PUBLICAR NO SERVIDOR MQTT
// =============================================================================================================
void publicaComando()
{
  if (!client.connected())
  {
    imprimirSerial(true, "MQTT desconectado! Tentando reconectar...");
    reconectar();
  }
  client.loop();
  //DADOS A SEREM PUBLICADOS NO SERVIDOR MQTT
  imprimirSerial(true, "Fazendo a publicacao...");

  if (x_pub_1)
  {
    client.publish(mqtt_topico_info, digitalRead(bomb) == HIGH ? "b1" : "b0", false);
    x_pub_1 = false;
    delay(80);
  }
  if (x_pub_2)
  {
    client.publish(mqtt_topico_info, digitalRead(res) == HIGH ? "r1" : "r0", false);
    x_pub_2 = false;
    delay(80);
  }
  if (x_pub_3)
  {
    client.publish(mqtt_topico_temp, String(temp).c_str());
    x_pub_3 = false;
    delay(80);
  }
  if (x_pub_4)
  {
    client.publish(mqtt_topico_sub, en_aque == HIGH ? "aqon" : "aqoff", false);
    x_pub_4 = false;
    delay(80);
  }
  if (x_pub_5)
  {
    client.publish(mqtt_topico_sub, en_res == HIGH ? "reson" : "resoff", false);
    x_pub_5 = false;
    delay(80);
  }
  if (x_pub_6)
  {
    client.publish(mqtt_topico_settempmax, String(xsettempmax).c_str());
    x_pub_6 = false;
    delay(80);
  }
  if (x_pub_7)
  {
    client.publish(mqtt_topico_settemp, String(settemp).c_str());
    x_pub_7 = false;
    delay(80);
  }
  if (x_pub_8)
  {
    client.publish(mqtt_topico_setrange, String(setrange).c_str());
    x_pub_8 = false;
    delay(80);
  }
  if (x_pub_9)
  {
    client.publish(mqtt_topico_lasttempmin, String(xRtempmin).c_str());
    x_pub_9 = false;
    delay(80);
  }
  if (x_pub_10)
  {
    client.publish(mqtt_topico_lasttempmax, String(xRtempmax).c_str());
    x_pub_10 = false;
    delay(80);
  }
  if (x_pub_11)
  {
    client.publish(mqtt_topico_totalaction, String(xRaction).c_str());
    x_pub_11 = false;
    delay(80);
  }
  if (x_pub_12)
  {
    client.publish(mqtt_topico_sub, xmbomb == HIGH ? "mb1" : "mb0", false);
    client.publish(mqtt_topico_sub, xmres == HIGH ? "mr_1" : "mr_0", false);
    x_pub_12 = false;
    delay(80);
  }
  if (x_pub_13)
  {
    client.publish(mqtt_topico_settime, String(timedelay).c_str());
    x_pub_13 = false;
    delay(80);
  }


  if (x_pub_14)
  {
    // ---------------------------------------------------------------------
    if ((startev1 < 1000) && (startev1 > 100))
    {
      mxpt = "0" + String(startev1);
      client.publish(mqtt_topico_timestart1, String(mxpt).c_str());
      delay(80);
    }else if ((startev1 < 100) && (startev1 > 10))
    {
      mxpt = "00" + String(startev1);
      client.publish(mqtt_topico_timestart1, String(mxpt).c_str());
      delay(80);
    }else if (startev1 < 10)
    {
      mxpt = "000" + String(startev1);
      client.publish(mqtt_topico_timestart1, String(mxpt).c_str());
      delay(80);
    }    
    else{
      client.publish(mqtt_topico_timestart1, String(startev1).c_str());
      delay(80);
    }
    // ---------------------------------------------------------------------

    // ---------------------------------------------------------------------
    if ((startev2 < 1000) && (startev2 > 100))
    {
      mxpt = "0" + String(startev2);
      client.publish(mqtt_topico_timestart2, String(mxpt).c_str());
      delay(80);
    }else if ((startev2 < 100) && (startev2 > 10))
    {
      mxpt = "00" + String(startev2);
      client.publish(mqtt_topico_timestart2, String(mxpt).c_str());
      delay(80);
    }else if (startev2 < 10)
    {
      mxpt = "000" + String(startev2);
      client.publish(mqtt_topico_timestart2, String(mxpt).c_str());
      delay(80);
    }    
    else{
      client.publish(mqtt_topico_timestart2, String(startev2).c_str());
      delay(80);
    }
    // ---------------------------------------------------------------------

        // ---------------------------------------------------------------------
    if ((startev3 < 1000) && (startev3 > 100))
    {
      mxpt = "0" + String(startev3);
      client.publish(mqtt_topico_timestart3, String(mxpt).c_str());
      delay(80);
    }else if ((startev3 < 100) && (startev3 > 10))
    {
      mxpt = "00" + String(startev3);
      client.publish(mqtt_topico_timestart3, String(mxpt).c_str());
      delay(80);
    }else if (startev3 < 10)
    {
      mxpt = "000" + String(startev3);
      client.publish(mqtt_topico_timestart3, String(mxpt).c_str());
      delay(80);
    }    
    else{
      client.publish(mqtt_topico_timestart3, String(startev3).c_str());
      delay(80);
    }
    // ---------------------------------------------------------------------

        // ---------------------------------------------------------------------
    if ((startev4 < 1000) && (startev4 > 100))
    {
      mxpt = "0" + String(startev4);
      client.publish(mqtt_topico_timestart4, String(mxpt).c_str());
      delay(80);
    }else if ((startev4 < 100) && (startev4 > 10))
    {
      mxpt = "00" + String(startev4);
      client.publish(mqtt_topico_timestart4, String(mxpt).c_str());
      delay(80);
    }else if (startev4 < 10)
    {
      mxpt = "000" + String(startev4);
      client.publish(mqtt_topico_timestart4, String(mxpt).c_str());
      delay(80);
    }    
    else{
      client.publish(mqtt_topico_timestart4, String(startev4).c_str());
      delay(80);
    }
    // ---------------------------------------------------------------------
    // Time end 1
    if ((endev1 < 1000) && (endev1 > 100))
    {
      mxpt = "0" + String(endev1);
      client.publish(mqtt_topico_timeend1, String(mxpt).c_str());
      delay(80);
    }else if ((endev1 < 100) && (endev1 > 10))
    {
      mxpt = "00" + String(endev1);
      client.publish(mqtt_topico_timeend1, String(mxpt).c_str());
      delay(80);
    }else if (endev1 < 10)
    {
      mxpt = "000" + String(endev1);
      client.publish(mqtt_topico_timeend1, String(mxpt).c_str());
      delay(80);
    }    
    else{
      client.publish(mqtt_topico_timeend1, String(endev1).c_str());
      delay(80);
    }
    // ---------------------------------------------------------------------

    // ---------------------------------------------------------------------
    // Time end 2
    if ((endev2 < 1000) && (endev2 > 100))
    {
      mxpt = "0" + String(endev2);
      client.publish(mqtt_topico_timeend2, String(mxpt).c_str());
      delay(80);
    }else if ((endev2 < 100) && (endev2 > 10))
    {
      mxpt = "00" + String(endev2);
      client.publish(mqtt_topico_timeend2, String(mxpt).c_str());
      delay(80);
    }else if (endev2 < 10)
    {
      mxpt = "000" + String(endev2);
      client.publish(mqtt_topico_timeend2, String(mxpt).c_str());
      delay(80);
    }    
    else{
      client.publish(mqtt_topico_timeend2, String(endev2).c_str());
      delay(80);
    }
    // ---------------------------------------------------------------------

        // ---------------------------------------------------------------------
    // Time end 3
    if ((endev3 < 1000) && (endev3 > 100))
    {
      mxpt = "0" + String(endev3);
      client.publish(mqtt_topico_timeend3, String(mxpt).c_str());
      delay(80);
    }else if ((endev3 < 100) && (endev3 > 10))
    {
      mxpt = "00" + String(endev3);
      client.publish(mqtt_topico_timeend3, String(mxpt).c_str());
      delay(80);
    }else if (endev3 < 10)
    {
      mxpt = "000" + String(endev3);
      client.publish(mqtt_topico_timeend3, String(mxpt).c_str());
      delay(80);
    }    
    else{
      client.publish(mqtt_topico_timeend3, String(endev3).c_str());
      delay(80);
    }
    // ---------------------------------------------------------------------

        // ---------------------------------------------------------------------
    // Time end 4
    if ((endev4 < 1000) && (endev4 > 100))
    {
      mxpt = "0" + String(endev4);
      client.publish(mqtt_topico_timeend4, String(mxpt).c_str());
      delay(80);
    }else if ((endev4 < 100) && (endev4 > 10))
    {
      mxpt = "00" + String(endev4);   
      client.publish(mqtt_topico_timeend4, String(mxpt).c_str());
      delay(80);
    }else if (endev4 < 10)
    {
      mxpt = "000" + String(endev4);
      client.publish(mqtt_topico_timeend4, String(mxpt).c_str());
      delay(80);
    }    
    else{
      client.publish(mqtt_topico_timeend4, String(endev4).c_str());
      delay(80);
    }
    // ---------------------------------------------------------------------
   
    
    
    client.publish(mqtt_topico_ev1, xev1 == true ? "true" : "false", false);
    delay(80);
    client.publish(mqtt_topico_ev2, xev2 == true ? "true" : "false", false);
    delay(80);
    client.publish(mqtt_topico_ev3, xev3 == true ? "true" : "false", false);
    delay(80);
    client.publish(mqtt_topico_ev4, xev4 == true ? "true" : "false", false);
    delay(80);    
    x_pub_14 = false;
    delay(80);
  }

  if (x_pub_status)
  {
    men_status = "Temp="+ String(temp)+"; "+"SetTemp="+ String(settemp)+"; "+"Bomba="+ String(digitalRead(bomb))+"; "+"Apoio="+ String(digitalRead(res))+"; "+"Data="+ "/"+"/"+ "; "+"Hora="+String(hora);
    client.publish(mqtt_topico_status, String(men_status).c_str(),true);
    delay(80);
    x_pub_status = false;
  }
  

  if (!x_pub_1 && !x_pub_2 && !x_pub_3 && !x_pub_4 && !x_pub_5 && !x_pub_6 && !x_pub_7 && !x_pub_8 && !x_pub_9 && !x_pub_10 && !x_pub_11 && !x_pub_12 && !x_pub_13 && !x_pub_status)
  {
    publicar = false;
    refresh = false;
  }

  imprimirSerial(true, "Publicacao Feita com Sucesso !!!");
  
}
//#############################################################################################################

// =============================================================================================================
//                               FUNCAO CONSTROI INT COM 2 BYTES
// =============================================================================================================
int _constBits(byte h, byte m)
{
  int status = 0;

  for (size_t i = 0; i <= 16; i++)
  {
    if (i <= 7)
    {
      if (bitRead(h, i))
      {
        bitSet(status, i);
      }
      else
      {
        bitClear(status, i);
      }
    }
    else if (i > 7)
    {
      if (bitRead(m, i - 8))
      {
        bitSet(status, i);
      }
      else
      {
        bitClear(status, i);
      }
    }
  }
  return status;
}
//#############################################################################################################





// =============================================================================================================
//                               FUNCAO RECUPERA DADOS SETPOINT TEMP NA MEMORIA EEPROM
// =============================================================================================================
void lerStatussetpoint()
{
  byte start1h = 0, start1m = 0;
  byte start2h = 0, start2m = 0;
  byte start3h = 0, start3m = 0;
  byte start4h = 0, start4m = 0;

  byte end1h = 0, end1m = 0;
  byte end2h = 0, end2m = 0;
  byte end3h = 0, end3m = 0;
  byte end4h = 0, end4m = 0;

  EEPROM.begin(memoria_alocada);     //Aloca o espaco definido na memoria
  status_settemp = EEPROM.read(0);   //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  status_setrange = EEPROM.read(2);  //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  status_tempmax = EEPROM.read(4);  //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  status_Rtempmin = EEPROM.read(6); //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  status_Rtempmax = EEPROM.read(8); //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  status_Raction = EEPROM.read(10);  //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  status_time = EEPROM.read(12);     //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  status_aq = EEPROM.read(14);     //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  status_res = EEPROM.read(16);    //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  status_ev1 = EEPROM.read(18);    //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  status_ev2 = EEPROM.read(20);    //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  status_ev3 = EEPROM.read(22);    //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  status_ev4 = EEPROM.read(24);    //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  start1h = EEPROM.read(32);         //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  start1m = EEPROM.read(34);         //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  start2h = EEPROM.read(36);         //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  start2m = EEPROM.read(38);         //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  start3h = EEPROM.read(40);         //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  start3m = EEPROM.read(42);         //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  start4h = EEPROM.read(44);         //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  start4m = EEPROM.read(46);         //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  end1h = EEPROM.read(48);           //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  end1m = EEPROM.read(50);           //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  end2h = EEPROM.read(52);           //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  end2m = EEPROM.read(54);           //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  end3h = EEPROM.read(56);           //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  end3m = EEPROM.read(58);           //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  end4h = EEPROM.read(60);           //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"
  end4m = EEPROM.read(62);           //Le o valor armazenado na EEPROM e passa para a variável "statusAnt"



  // ------------------------------------------------------------------------------------------------------------

  status_x1start = _constBits(start1h, start1m);
  status_x2start = _constBits(start2h, start2m);
  status_x3start = _constBits(start3h, start3m);
  status_x4start = _constBits(start4h, start4m);
  status_x1end = _constBits(end1h, end1m);
  status_x2end = _constBits(end2h, end2m);
  status_x3end = _constBits(end3h, end3m);
  status_x4end = _constBits(end4h, end4m);

  // ------------------------------------------------------------------------------------------------------------

  settemp = status_settemp;
  setrange = status_setrange;
  xsettempmax = status_tempmax; 
  xRtempmin = status_Rtempmin;
  xRtempmax = status_Rtempmax;
  xRaction = status_Raction;
  xev1 = status_ev1;
  xev2 = status_ev2;
  xev3 = status_ev3;
  xev4 = status_ev4;
  en_aque = status_aq;
  en_res = status_res;
  timedelay = status_time * 100000;
  startev1 = status_x1start;
  startev2 = status_x2start;
  startev3 = status_x3start;
  startev4 = status_x4start;
  endev1 = status_x1end;
  endev2 = status_x2end;
  endev3 = status_x3end;
  endev4 = status_x4end;


  if((timedelay != 1800000) && (timedelay != 3600000)&& (timedelay != 5400000)&& (timedelay != 7200000)&& (timedelay != 9000000)&& (timedelay != 10800000)){

    timedelay = 3600000;
    gravartime = true;
  }


  

  // ------------------------------------------------------------------------------------------------------------

  imprimirSerial(true, " Valor recuperado na EEPROM");
  imprimirSerial(true, " setpoint Temperatura");
  imprimirSerial(false, String(settemp).c_str());
  imprimirSerial(true, "  ");

  imprimirSerial(true, " set Range");
  imprimirSerial(false, String(setrange).c_str());
  imprimirSerial(true, "  ");

  imprimirSerial(true, " settemp MAx");
  imprimirSerial(false, String(xsettempmax).c_str());
  imprimirSerial(true, "  ");

  imprimirSerial(true, " Registro de temperatura Minima");
  imprimirSerial(false, String(xRtempmin).c_str());
  imprimirSerial(true, "  ");

  imprimirSerial(true, " Registro de temperatura maxima");
  imprimirSerial(false, String(xRtempmax).c_str());
  imprimirSerial(true, "  ");

  imprimirSerial(true, " Numero de Acionamentos ");
  imprimirSerial(false, String(xRaction).c_str());
  imprimirSerial(true, "  ");

  imprimirSerial(true, " habilita aquecedor");
  imprimirSerial(false, String(en_aque).c_str());
  imprimirSerial(true, "  ");

  imprimirSerial(true, " habilita apoio eletrico");
  imprimirSerial(false, String(en_res).c_str());
  imprimirSerial(true, "  ");

  imprimirSerial(true, " habilita ev1");
  imprimirSerial(false, String(xev1).c_str());
  imprimirSerial(true, "  ");

  imprimirSerial(true, " habilita ev2");
  imprimirSerial(false, String(xev2).c_str());
  imprimirSerial(true, "  ");

  imprimirSerial(true, " habilita ev3");
  imprimirSerial(false, String(xev3).c_str());
  imprimirSerial(true, "  ");

  imprimirSerial(true, " habilita ev4");
  imprimirSerial(false, String(xev4).c_str());
  imprimirSerial(true, "  ");

  imprimirSerial(true, " Time acionamento em Manual");
  imprimirSerial(false, String(timedelay).c_str());
  imprimirSerial(true, "  ");

  imprimirSerial(true, "Start Evento 1 ");
  imprimirSerial(false, String(startev1).c_str());
  imprimirSerial(true, "  ");

  imprimirSerial(true, "Start Evento 2 ");
  imprimirSerial(false, String(startev2).c_str());
  imprimirSerial(true, "  ");

  imprimirSerial(true, "Start Evento 3 ");
  imprimirSerial(false, String(startev3).c_str());
  imprimirSerial(true, "  ");

  imprimirSerial(true, "Start Evento 4 ");
  imprimirSerial(false, String(startev4).c_str());
  imprimirSerial(true, "  ");

  imprimirSerial(true, "End Evento 1 ");
  imprimirSerial(false, String(endev1).c_str());
  imprimirSerial(true, "  ");

  imprimirSerial(true, "End Evento 2 ");
  imprimirSerial(false, String(endev2).c_str());
  imprimirSerial(true, "  ");

  imprimirSerial(true, "End Evento 3 ");
  imprimirSerial(false, String(endev3).c_str());
  imprimirSerial(true, "  ");

  imprimirSerial(true, "End Evento 4 ");
  imprimirSerial(false, String(endev4).c_str());
  imprimirSerial(true, "  ");

  Leitura_set_ok = true;
  EEPROM.end();
}
//#############################################################################################################





// =============================================================================================================
//                              FUNCAO GRAVA DADOS SETPOINT TEMP   NA MEMORIA EEPROM
// =============================================================================================================
void _gravatemp(uint16_t setTempEPR)
{ // grava valor setpoint temperatura minima na eeprom
  EEPROM.begin(memoria_alocada);
  EEPROM.write(0, setTempEPR);
  imprimirSerial(true, " Valor Gravado na EEPROM");
  imprimirSerial(false, String(setTempEPR).c_str());
  imprimirSerial(true, "  ");
  gravatemp = false;
  EEPROM.end();
}
//#############################################################################################################





// =============================================================================================================
//                              FUNCAO GRAVA DADOS SETPOINT RANGE   NA MEMORIA EEPROM
// =============================================================================================================
void _gravarange(uint16_t setRangeEPR)
{ // grava valor setpoint temperatura maxima na eeprom
  EEPROM.begin(memoria_alocada);
  EEPROM.write(2, setRangeEPR);
  imprimirSerial(true, " Valor Gravado na EEPROM");
  imprimirSerial(false, String(setRangeEPR).c_str());
  imprimirSerial(true, "  ");
  gravarange = false;
  EEPROM.end();
}
//#############################################################################################################




// =============================================================================================================
//                              FUNCAO GRAVA DADOS TEMP MAX   NA MEMORIA EEPROM
// =============================================================================================================
void _gravartempmax(uint16_t tempmaxepr)
{ // grava valor temp maxima na eeprom
  EEPROM.begin(memoria_alocada);
  EEPROM.write(4, tempmaxepr);
  imprimirSerial(true, " Valor Gravado na EEPROM");
  imprimirSerial(false, String(tempmaxepr).c_str());
  imprimirSerial(true, "  ");
  gravarsettempmax = false;
  EEPROM.end();
}
//#############################################################################################################




// =============================================================================================================
//                              FUNCAO GRAVA DADOS TEMP MINIMO   NA MEMORIA EEPROM
// =============================================================================================================
void _gravarRtempmin(uint16_t rtempminepr)
{ // grava valor temp maxima na eeprom
  EEPROM.begin(memoria_alocada);
  EEPROM.write(6, rtempminepr);
  imprimirSerial(true, " Valor Gravado na EEPROM");
  imprimirSerial(false, String(rtempminepr).c_str());
  imprimirSerial(true, "  ");
  gravarRtempmin = false;
  EEPROM.end();
}
//#############################################################################################################




// =============================================================================================================
//                              FUNCAO GRAVA DADOS TEMP MAX   NA MEMORIA EEPROM
// =============================================================================================================
void _gravarRtempmax(uint16_t rtempmaxepr)
{ // grava valor temp maxima na eeprom
  EEPROM.begin(memoria_alocada);
  EEPROM.write(8, rtempmaxepr);
  imprimirSerial(true, " Valor Gravado na EEPROM");
  imprimirSerial(false, String(rtempmaxepr).c_str());
  imprimirSerial(true, "  ");
  gravarRtempmax = false;
  EEPROM.end();
}
//#############################################################################################################




// =============================================================================================================
//                              FUNCAO GRAVA DADOS REGISTRO DE AÇÕES  NA MEMORIA EEPROM
// =============================================================================================================
void _gravarRaction(uint16_t ractionepr)
{ // grava valor temp maxima na eeprom
  EEPROM.begin(memoria_alocada);
  EEPROM.write(10, ractionepr);
  imprimirSerial(true, " Valor Gravado na EEPROM");
  imprimirSerial(false, String(ractionepr).c_str());
  imprimirSerial(true, "  ");
  gravarRaction = false;
  EEPROM.end();
}
//#############################################################################################################




// =============================================================================================================
//                              FUNCAO GRAVA DADOS TIME DE AÇÕES  NA MEMORIA EEPROM
// =============================================================================================================
void _gravarTime(int timeepr)
{ // grava valor temp maxima na eeprom
  int delayepr;

  delayepr = (timeepr / 100000);

  EEPROM.begin(memoria_alocada);
  EEPROM.write(12, delayepr);
  imprimirSerial(true, " Valor Gravado na EEPROM");
  imprimirSerial(false, String(delayepr).c_str());
  imprimirSerial(true, "  ");
  gravartime = false;
  EEPROM.end();
}
//#############################################################################################################




// =============================================================================================================
//                              FUNCAO GRAVA DADOS REGISTRO DE AÇÕES  NA MEMORIA EEPROM
// =============================================================================================================
void _gravarbits(uint8_t aqepr, uint8_t resepr, uint8_t ev1epr, uint8_t ev2epr, uint8_t ev3epr, uint8_t ev4epr)
{ // grava valor temp maxima na eeprom
  EEPROM.begin(memoria_alocada);
  EEPROM.write(14, aqepr);
  EEPROM.write(16, resepr);
  EEPROM.write(18, ev1epr);
  EEPROM.write(20, ev2epr);
  EEPROM.write(22, ev3epr);
  EEPROM.write(24, ev4epr);

  imprimirSerial(true, " Valor Gravado na EEPROM");
  imprimirSerial(false, String(aqepr).c_str());
  imprimirSerial(true, "  ");
  imprimirSerial(true, " Valor Gravado na EEPROM");
  imprimirSerial(false, String(resepr).c_str());
  imprimirSerial(true, "  ");
  imprimirSerial(true, " Valor Gravado na EEPROM");
  imprimirSerial(false, String(ev1epr).c_str());
  imprimirSerial(true, "  ");
  imprimirSerial(true, " Valor Gravado na EEPROM");
  imprimirSerial(false, String(ev2epr).c_str());
  imprimirSerial(true, "  ");
  imprimirSerial(true, " Valor Gravado na EEPROM");
  imprimirSerial(false, String(ev3epr).c_str());
  imprimirSerial(true, "  ");
  imprimirSerial(true, " Valor Gravado na EEPROM");
  imprimirSerial(false, String(ev4epr).c_str());
  imprimirSerial(true, "  ");
  gravarbits = false;
  EEPROM.end();
}
//#############################################################################################################





// =============================================================================================================
//                               FUNCAO DESCONTROI INT COM 2 BYTES
// =============================================================================================================
int _desconstBits(byte qb, uint16_t stat)
{
  int h = 0, m = 0;

  for (size_t i = 0; i <= 15; i++)
  {
    if (i <= 7)
    {
      if (bitRead(stat, i))
      {
        bitSet(h, i);
      }
      else
      {
        bitClear(h, i);
      }
    }
    else if (i > 7)
    {
      if (bitRead(stat, i))
      {
        bitSet(m, i - 8);
      }
      else
      {
        bitClear(m, i - 8);
      }
    }
  }
  if (qb == 0)
  {
    return h;
  }
  else if (qb == 8)
  {
    return m;
  }
}
//#############################################################################################################






// =============================================================================================================
//                              FUNCAO GRAVA DADOS AGENDAMENTOS  NA MEMORIA EEPROM
// =============================================================================================================
void _gravarAgenda(uint16_t start1epr, uint16_t start2epr, uint16_t start3epr, uint16_t start4epr, uint16_t end1epr, uint16_t end2epr, uint16_t end3epr, uint16_t end4epr)
{ // grava valor agendamentos  na eeprom

  byte start1hx = 0, start1mx = 0;
  byte start2hx = 0, start2mx = 0;
  byte start3hx = 0, start3mx = 0;
  byte start4hx = 0, start4mx = 0;

  byte end1hx = 0, end1mx = 0;
  byte end2hx = 0, end2mx = 0;
  byte end3hx = 0, end3mx = 0;
  byte end4hx = 0, end4mx = 0;

  // ------------------------------------------------------------------------------------------------------------

  start1hx = _desconstBits(0, start1epr);
  start1mx = _desconstBits(8, start1epr);
  start2hx = _desconstBits(0, start2epr);
  start2mx = _desconstBits(8, start2epr);
  start3hx = _desconstBits(0, start3epr);
  start3mx = _desconstBits(8, start3epr);
  start4hx = _desconstBits(0, start4epr);
  start4mx = _desconstBits(8, start4epr);
  end1hx = _desconstBits(0, end1epr);
  end1mx = _desconstBits(8, end1epr);
  end2hx = _desconstBits(0, end2epr);
  end2mx = _desconstBits(8, end2epr);
  end3hx = _desconstBits(0, end3epr);
  end3mx = _desconstBits(8, end3epr);
  end4hx = _desconstBits(0, end4epr);
  end4mx = _desconstBits(8, end4epr);

  // ------------------------------------------------------------------------------------------------------------

  EEPROM.begin(memoria_alocada);
  EEPROM.write(32, start1hx);
  EEPROM.write(34, start1mx);
  EEPROM.write(36, start2hx);
  EEPROM.write(38, start2mx);
  EEPROM.write(40, start3hx);
  EEPROM.write(42, start3mx);
  EEPROM.write(44, start4hx);
  EEPROM.write(46, start4mx);
  EEPROM.write(48, end1hx);
  EEPROM.write(50, end1mx);
  EEPROM.write(52, end2hx);
  EEPROM.write(54, end2mx);
  EEPROM.write(56, end3hx);
  EEPROM.write(58, end3mx);
  EEPROM.write(60, end4hx);
  EEPROM.write(62, end4mx);

  imprimirSerial(true, "Start Evento 1 ");
  imprimirSerial(false, String(start1epr).c_str());
  imprimirSerial(true, " Agendamento  Gravado na EEPROM ");
  gravarAgenda = false;
  EEPROM.end();
}
//#############################################################################################################





// =============================================================================================================
//                              FUNCAO RECEBE DADOS SERVIDOR MQTT BROKER
// =============================================================================================================
void retorno(char *topico, byte *mensagem, unsigned int tamanho)
{
  //Convertendo a mensagem recebida para string
  mensagem[tamanho] = '\0';
  String strMensagem = String((char *)mensagem);
  String strtopico = String((char *)topico);
  strMensagem.toLowerCase();
  //float f = s.toFloat();

  imprimirSerial(true, " ");
  imprimirSerial(false, "Mensagem recebida! Topico: ");
  imprimirSerial(false, topico);
  imprimirSerial(false, ". Tamanho: ");
  imprimirSerial(false, String(tamanho).c_str());
  imprimirSerial(false, ". Mensagem: ");
  imprimirSerial(true, strMensagem);
  imprimirSerial(true, " ");
  imprimirSerial(true, " ");
  imprimirSerial(true, " ");

  // ------------------- Agendamentos ---------------

  if (strtopico == mqtt_topico_ev1)
  {
    if (strMensagem == "true")
    {
      xev1 = true;
    }
    else if (strMensagem == "false")
    {
      xev1 = false;
    }
  }
  if (strtopico == mqtt_topico_ev2)
  {
    if (strMensagem == "true")
    {
      xev2 = true;
    }
    else if (strMensagem == "false")
    {
      xev2 = false;
    }
  }
  if (strtopico == mqtt_topico_ev3)
  {
    if (strMensagem == "true")
    {
      xev3 = true;
    }
    else if (strMensagem == "false")
    {
      xev3 = false;
    }
  }
  if (strtopico == mqtt_topico_ev4)
  {
    if (strMensagem == "true")
    {
      xev4 = true;
    }
    else if (strMensagem == "false")
    {
      xev4 = false;
    }
    gravarbits = true;
    delay(50);
  }
  // --------------- Start Evento ----------------------
  if (strtopico == mqtt_topico_timestart1)
  {
    startev1 = strMensagem.toInt();
    gravarAgenda = true;
    delay(50);
  }
  if (strtopico == mqtt_topico_timestart2)
  {
    startev2 = strMensagem.toInt();
    gravarAgenda = true;
    delay(50);
  }
  if (strtopico == mqtt_topico_timestart3)
  {
    startev3 = strMensagem.toInt();
    gravarAgenda = true;
    delay(50);
  }
  if (strtopico == mqtt_topico_timestart4)
  {
    startev4 = strMensagem.toInt();
    gravarAgenda = true;
    delay(50);
  }
  // --------------------------------------------------------
  // ---------------  End Evento ----------------------
  if (strtopico == mqtt_topico_timeend1)
  {
    endev1 = strMensagem.toInt();
    gravarAgenda = true;
    delay(50);
  }
  if (strtopico == mqtt_topico_timeend2)
  {
    endev2 = strMensagem.toInt();
    gravarAgenda = true;
    delay(50);
  }
  if (strtopico == mqtt_topico_timeend3)
  {
    endev3 = strMensagem.toInt();
    gravarAgenda = true;
    delay(50);
  }
  if (strtopico == mqtt_topico_timeend4)
  {
    endev4 = strMensagem.toInt();
    gravarAgenda = true;
    delay(50);
  }
  // --------------------------------------------------------



  // -------- Atualiza App -----------------------
  if (strMensagem == "appon")
  {
    refresh = true;
  }
  // ----------------------------------------------

  // ------------- Habilita manual aquecedor
  if (strMensagem == "mb1")
  {
    xmbomb = true;
    delay(50);
  }
  else if (strMensagem == "mb0")
  {
    xmbomb = false;
    delay(50);
  }
  // -------------- habilita Manual apoio eletrico
  if (strMensagem == "mr_1")
  {
    xmres = true;
    delay(50);
  }
  else if (strMensagem == "mr_0")
  {
    xmres = false;
    delay(50);
  }
  // ----------------------------------------------

  if (strtopico == mqtt_topico_settemp)
  { // SETPOINT TEMPERATURA MINIMA PELO APP

    aux_settemp = (strMensagem.toInt());

    if (aux_settemp != settemp)
    {

      settemp = aux_settemp;
      gravatemp = true;
    }
    delay(50);
  }
  //--------------------- Delay comando manual -------------------------
  if (strtopico == mqtt_topico_settime)
  {
    if (strMensagem == "00:30")
    {
      timedelay = 3600000;
      gravartime = true;
    }
    else if (strMensagem == "01:00")
    {
      timedelay = 3600000;
      gravartime = true;
    }
    else if (strMensagem == "01:30")
    {
      timedelay = 3600000;
      gravartime = true;
    }
    else if (strMensagem == "02:00")
    {
      timedelay = 3600000;
      gravartime = true;
    }
    else if (strMensagem == "02:30")
    {
      timedelay = 3600000;
      gravartime = true;
    }
    else if (strMensagem == "03:00")
    {
      timedelay = 3600000;
      gravartime = true;
    }
    delay(50);
  }
  // -------------------------------------------------------------------
  if (strtopico == mqtt_topico_settempmax)
  { // set temperatura maxima
    xsettempmax = (strMensagem.toInt());
    gravarsettempmax = true;
    delay(50);
  }

  if (strtopico == mqtt_topico_setrange)
  { // SETPOINT TEMPERATURA MAXIMA PELO APP

    aux_setrange = (strMensagem.toInt());

    if (aux_setrange != setrange)
    {

      setrange = aux_setrange;
      gravarange = true;
    }
    delay(50);
  }

  if (strMensagem == "aqon")
  {
    en_aque = true;
    gravarbits = true;
    delay(50);
  }
  else if (strMensagem == "aqoff")
  {
    en_aque = false;
    gravarbits = true;
    delay(50);
  }

  if (strMensagem == "reson")
  {
    en_res = true;
    gravarbits = true;
    delay(50);
  }
  else if (strMensagem == "resoff")
  {
    en_res = false;
    gravarbits = true;
    delay(50);
  }

   
 
  
  if (strMensagem == "restart")
  {
    
    ESP.restart();

  }
  
  if (strMensagem == "reset")
  {
    setrange = 1;
    xsettempmax = 60; 
    xRtempmin = 100;
    xRtempmax = 0;
    xRaction = 0;
    timedelay = 3600000;
    gravarange = true;
    gravarsettempmax = true;
    gravarRtempmin = true;
    gravarRtempmax = true;
    gravarRaction = true;
    gravartime = true;
  }

  if (strMensagem == "status")
  {
    publicar = true;
    x_pub_status = true;
  }
  
  
  // ---------------------------------------------------
}
//#############################################################################################################






// =============================================================================================================
//                              FUNCAO COMANDOS WEB PELO NAVEGADOR BROSER
// =============================================================================================================

void comandos_web()
{

  WiFiClient client = server.available();      //verifica se existe um cliente conectado com dados a serem transmitidos
  
  if(client)                                   //existe um cliente?
  {                                            //sim
    Serial.println("Novo Cliente Definido");   //informa por serial
    String currentLine = "";                   //string para armazenar a mensagem do cliente
    
    while(client.connected())                  //repete enquanto o cliente estiver conectado
    {
      
      if(client.available())                   //existem dados a serem lidos?
      {                                        //sim
        char c=client.read();                  //salva em c
        Serial.write(c);                       //mostra na Serial
        
        if(c=='\n')                            //é um caractere de nova linha?
        {                                      //sim
          if(currentLine.length()==0)          //a mensagem terminou?
          {                                    //sim
            //gera a página HTML
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            client.print("<h2>Clique <a href=\"/H1\">AQUI</a> para ligar o bomba .</h2><br>");
            client.print("<h2>Clique <a href=\"/L1\">AQUI</a> para desligar o bomba.</h2><br>");
            client.print("<h2>Clique <a href=\"/H2\">AQUI</a> para ligar o apoio eletrico.</h2><br>");
            client.print("<h2>Clique <a href=\"/L2\">AQUI</a> para desligar o apoio eletrico.</h2><br>");
            client.print("<h2>Clique <a href=\"/H3\">AQUI</a> para ligar o anel de Circulacao.</h2><br>");
            client.print("<h2>Clique <a href=\"/L3\">AQUI</a> para desligar o anel de Circulacao.</h2><br>");
            client.print("<h2> Temperatura Sensor 1 <br>");
            client.print(temp);
            client.print("<h2> Temperatura Sensor 2 <br>");
            client.print(tempS2);
            client.println();
            break;                             //encerra o laço
            
          } //end if currentLine.length
          
          else currentLine="";                 //senão, impede string de ficar com espaços em branco
        
        } //end if c
                  
        else if(c != '\r') currentLine += c;  //adicionamos o caractere como parte da mensagem, se diferene de retorno/nova linha

       //verifica para ligar ou desligar os relés
          if(currentLine.endsWith("GET /H1")) digitalWrite(bomb, HIGH);
          if(currentLine.endsWith("GET /L1")) digitalWrite(bomb,  LOW);
          if(currentLine.endsWith("GET /H2")) digitalWrite(res, HIGH);
          if(currentLine.endsWith("GET /L2")) digitalWrite(res,  LOW);
          if(currentLine.endsWith("GET /H3")) digitalWrite(circ, HIGH);
          if(currentLine.endsWith("GET /L3")) digitalWrite(circ,  LOW);
          
       
      } //end if client.available()
      
    } //end if while client.connected

    client.stop();                           //finaliza conexão
    Serial.println("Client Disconnected.");
    
  } //end if client  
  
  
}
//#############################################################################################################





// =============================================================================================================
//                                                  VOID SETUP
// =============================================================================================================
void setup()
{

// CONFIGURA VELOCIDADE  SERIAL  
// -------------------------------------------------------------------------------------------------------------

   Serial.begin(115200);


// DECLARAÇÃO PINOS ENTRADAS   
// -------------------------------------------------------------------------------------------------------------

  pinMode(bot_manu, INPUT_PULLDOWN);

// DECLARAÇÃO PINOS SAIDAS      
// -------------------------------------------------------------------------------------------------------------
  
  pinMode(bomb, OUTPUT);
  pinMode(res, OUTPUT);
  pinMode(circ,OUTPUT);
  //pinMode(wificonected, OUTPUT);

// GARANTE SAIDAS DESLIGADAS
// -------------------------------------------------------------------------------------------------------------

  digitalWrite(bomb, LOW);
  digitalWrite(res, LOW); 
  digitalWrite(circ, LOW); 
  //digitalWrite(wificonected, LOW);

  
// INICIA CONEXÃO COM WIFI LOCAL
// ------------------------------------------------------------------------------------------------------------- 

  Serial.println();                      //
  Serial.print("Conectando-se a ");      //
  Serial.println(ssid);                  //
  WiFi.begin(ssid, password);            //inicializa WiFi, passando o nome da rede e a senha

  while(WiFi.status() != WL_CONNECTED)   //aguarda conexão (WL_CONNECTED é uma constante que indica sucesso na conexão)
  {
    delay(741);                          //
    Serial.print(".");                   //vai imprimindo pontos até realizar a conexão...
  }

  Serial.println("");                    //mostra WiFi conectada
  Serial.println("WiFi conectada");      //
  Serial.println("Endereço de IP: ");    //
  Serial.println(WiFi.localIP());        //mostra o endereço IP

  // PEGA DATA E HORAINTERNET
  // -------------------------------------------------------------------------------------------------------------
   
  Udp.begin(localPort); //X
  setSyncProvider(getNtpTime);
  setSyncInterval(300); 

  
  // CARREGA PARAMETROS SERVIDOR MQTT
  // -------------------------------------------------------------------------------------------------------------
  
  WiFiManagerParameter custom_mqtt_server("server", "Servidor MQTT", servidor_mqtt, 40);
  WiFiManagerParameter custom_mqtt_port("port", "Porta", servidor_mqtt_porta, 6);
  WiFiManagerParameter custom_mqtt_user("user", "Usuario", servidor_mqtt_usuario, 20);
  WiFiManagerParameter custom_mqtt_pass("pass", "Senha", servidor_mqtt_senha, 20);
  WiFiManagerParameter custom_mqtt_topic_sub("topic_sub", "Topico para subscrever", mqtt_topico_sub, 30);

  //Inicialização do WiFiManager. Uma vez iniciado não é necessário mantê-lo em memória.
  // -------------------------------------------------------------------------------------------------------------

  WiFiManager wifiManager;

  //Definindo a função que informará a necessidade de salvar as configurações
  // -------------------------------------------------------------------------------------------------------------

  wifiManager.setSaveConfigCallback(precisaSalvarCallback);

 //Adicionando os parâmetros para conectar ao servidor MQTT
 // ------------------------------------------------------------------------------------------------------------- 

  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);
  wifiManager.addParameter(&custom_mqtt_topic_sub);


  //Se chegou até aqui é porque conectou na WiFi!
  // -------------------------------------------------------------------------------------------------------------

  imprimirSerial(true, "Conectado ao Servidor MQTT ");
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  imprimirSerial(false, "IP: ");
  imprimirSerial(true, WiFi.localIP().toString());
  

  //Informando ao client do PubSub a url do servidor e a porta.
  // -------------------------------------------------------------------------------------------------------------

  int portaInt = atoi(servidor_mqtt_porta);
  client.setServer(servidor_mqtt, portaInt);
  client.setCallback(retorno);

  //Obtendo o status SETPOINT APOS  o ESP ser ligado
  // -------------------------------------------------------------------------------------------------------------

  lerStatussetpoint();          // LE PARAMETROS SALVOS NA EEPROM DO ESP32
  server.begin();               // inicializa o servidor web
  sensor_1.begin();             // INICIALIZA LEITURA SENSO 1 DE TEMPERATURA
  sensor_2.begin();             // INICIALIZA LEITURA SENSO 2 DE TEMPERATURA

  // FUNÇÃO SETUP FINALIZADA
  // -------------------------------------------------------------------------------------------------------------

  imprimirSerial(true, "FUNÇÃO SETUP FINALIZADA COM SUCESSO ");
  imprimirSerial(true, "WIFI CONECTADO COM SUCESSO ");
  imprimirSerial(true, "SERVIDOR MQTT CONECTADO COM SUCESSO ");
  imprimirSerial(true, "WEB SERVER INICIALIZADO COM SUCESSO ");
  imprimirSerial(true, "SENSORES INICIALIZADOS COM SUCESSO ");
  imprimirSerial(true, "LEITURA STATUS NA EEPROM REALIZADOS COM SUCESSO ");


}
//#############################################################################################################





// =============================================================================================================
//                                                  VOID LOOP
// =============================================================================================================
void loop()
{

  // DECLARAÇÃO VARIAVEIS LOCAIS
  // -------------------------------------------------------------------------------------------------------------

  unsigned long time_127 = 0;                            // Auxiliar time
  unsigned long tempo_atual = millis();                  // Tempo em Milisegundos 
  unsigned long time_block = 0;                          // Bloqueia contagem de tempo
  horanow = (hour() * 100) + minute();                   // Tempo atual em milisegundos 
  sensor_1.requestTemperatures();                        // COLETA DADOS SENSOR 1 TEMPERATURA DIRETO DA BIBLIOTECA
  temp = sensor_1.getTempCByIndex(0);                    // MOVE VALOR SENSOR 1 TEMPERATURA PARA VARIAVEL GLOBAL
  sensor_2.requestTemperatures();                        // COLETA DADOS SENSOR 2 TEMPERATURA DIRETO DA BIBLIOTECA
  tempS2 = sensor_2.getTempCByIndex(0);                  // MOVE VALOR SENSOR 2 TEMPERATURA PARA VARIAVEL GLOBAL
  hora = (String(hour()) +":"+ String(minute()));        // CONVERTE HORA DE MILISEGUNDOS PARA PADRÃO

  // -------------------------------------------------------------------------------------------------------------


  // =============================================================================================================
  // ------------------------------------------  GESTÃO TIME -----------------------------------------------------
  // =============================================================================================================


  if ((temp == -127) && (!time_b127))
  {
    time_127 = tempo_atual;
    time_b127 = true;
  }
  

  if (temp == -127)
  {

    if ((tempo_atual >= (time_127 + 6000)) && time_b127)
    {
      temp_atual = temp; 
      time_b127 = false;  
      time_127 = tempo_atual; 
    }     
    
  }else{

    temp_atual = temp;

  }

  if ((tempo_atual >= (time_127 + 10000)) && time_b127 && temp != -127)
  {
      time_b127 = false;  
      time_127 = tempo_atual; 
  }
  
  

  if ((time_update_status == 0) || (tempo_atual >= (time_update_status + 3600000)))
  {
    time_update_status = tempo_atual;
    publicar = true;
    x_pub_status = true;
    imprimirSerial(true, "Publica status");
  }
  
  // =============================================================================================================
 



  // =============================================================================================================
  // ------------------------------------------- Modo Manual -----------------------------------------------------
  // =============================================================================================================

  if ((xmbomb) || (xmres)){

    if (!xt){xmdelay = tempo_atual;xt = true;} 

    if ((tempo_atual <= (xmdelay + timedelay)) && (xt)){xmstart = true;}

    else{xt = false;xmbomb = false;xmres = false;xmstart = false;}

  }else{xt = false;xmbomb = false;xmres = false;xmstart = false;}

  // =============================================================================================================




  // =============================================================================================================
  // ----------------------------------------- GESTÃO DE EVENTOS -------------------------------------------------
  // =============================================================================================================

  
  if (xev1) // --------------------- Evento 1 --------------------------------------------------------------------
  {
    x1start = ((horanow >= startev1) && (horanow <= endev1) && (okev1) ? true : false);
    
  }
  else
  {
    x1start = false;
  }
  
  
  if (xev2) // --------------------- Evento 2 --------------------------------------------------------------------
  {
    x2start = ((horanow >= startev2) && (horanow <= endev2) && (okev2) ? true : false);
    
  }
  else
  {
    x2start = false;
  }
  
  
  if (xev3) // --------------------- Evento 3 --------------------------------------------------------------------
  {
    x3start = ((horanow >= startev3) && (horanow <= endev3) && (okev3) ? true : false);
    
  }
  else
  {
    x3start = false;
  }
  
  
  if (xev4) // --------------------- Evento 4 --------------------------------------------------------------------
  {
    x4start = ((horanow >= startev4) && (horanow <= endev4) && (okev4) ? true : false);
    
  }
  else
  {
    x4start = false;
  }

  // =============================================================================================================




  // =============================================================================================================
  // ---------------------------------------  MODO AUTOMATICO ----------------------------------------------------
  // =============================================================================================================

  if ((x1start || x2start || x3start || x4start) && (temp != -127))
  {

    en_start = true;                // HABILITA CICLO AUTOMATICO
    
  }
  else
  {
    en_start = false;               // DESABILITA CICLO AUTOMATICO
  }

  ciclo();                          // CHAMADA CICLO AUTOMATICO  

  // =============================================================================================================




  // =============================================================================================================
  // ---------------------------------------  DADOS UPDATE   -----------------------------------------------------
  // =============================================================================================================

  if (refresh)
  {
    imprimirSerial(true, " Atualização de dados para Servidor!");
    x_pub_1 = true;
    x_pub_2 = true;
    x_pub_3 = true;
    x_pub_4 = true;
    x_pub_5 = true;
    x_pub_6 = true;
    x_pub_7 = true;
    x_pub_8 = true;
    x_pub_9 = true;
    x_pub_10 = true;
    x_pub_11 = true;
    x_pub_12 = true;
    x_pub_13 = true;
    x_pub_14 = true;
    publicar = true;
  }

  if (publicar) // CHAMA FUNCAO PUBLICAR
  {
    publicaComando();
    desconectar();
    imprimirSerial(true, "  ");
    imprimirSerial(true, " Publicação Feita pelo Ciclo Principal !!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    imprimirSerial(true, "  ");
    publicar = false;
  }
  // =============================================================================================================

 



  // =============================================================================================================
  // ---------------------------------------  GRAVA NA EEPROM   --------------------------------------------------
  // =============================================================================================================

  if (gravatemp)
  { // CHAMA FUNCAO QUE GRAVA TEMPERATURA MINIMA NA EEPROM
    _gravatemp(settemp);
  }
  // =============================================================================================================
  if (gravarange)
  { // CHAMA FUNCAO QUE GRAVA TEMPERATURA MAXIMA NA EEPROM
    _gravarange(setrange);
  }
  // =============================================================================================================
  // =============================================================================================================
  if (gravarsettempmax)
  { // CHAMA FUNCAO QUE GRAVA  NA EEPROM
    _gravartempmax(xsettempmax);
  }
  // =============================================================================================================
  // =============================================================================================================
  if (gravarRtempmin)
  { // CHAMA FUNCAO QUE GRAVA  NA EEPROM
    _gravarRtempmin(xRtempmin);
  }
  // =============================================================================================================
  // =============================================================================================================
  if (gravarRtempmax)
  { // CHAMA FUNCAO QUE GRAVA  NA EEPROM
    _gravarRtempmax(xRtempmax);
  }
  // =============================================================================================================
  // =============================================================================================================
  if (gravarRaction)
  { // CHAMA FUNCAO QUE GRAVA  NA EEPROM
    _gravarRaction(xRaction);
  }
  // =============================================================================================================
  // =============================================================================================================
  if (gravarbits)
  { // CHAMA FUNCAO QUE GRAVA  NA EEPROM
    _gravarbits(en_aque, en_res, xev1, xev2, xev3, xev4);
  }
  // =============================================================================================================

  // =============================================================================================================
  if (gravartime)
  { // CHAMA FUNCAO QUE GRAVA  NA EEPROM
    _gravarTime(timedelay);
  }
  // =============================================================================================================

  // =============================================================================================================
  if (gravarAgenda)
  { // CHAMA FUNCAO QUE GRAVA  NA EEPROM
    _gravarAgenda(startev1, startev2, startev3, startev4, endev1, endev2, endev3, endev4);
  }
  // =============================================================================================================

  // ----------------------------------
  if ((temp < xRtempmin) && (temp != -127))
  {
    imprimirSerial(true, "temp min");
    xRtempmin = temp;
    gravarRtempmin = true;
  }

  // ------------------------------------

  if (timedelay > 10800000)
  {
    timedelay = 1800000;
    gravartime = true;
  }

  if (xRtempmax >= 100)
  {
    xRtempmax = 0;
    xRtempmin = 100;
  }
  if ((xsettempmax > 100) || (xsettempmax < 0))
  {
    xsettempmax = 60;
    gravarsettempmax = true;
  }

  if (temp > xRtempmax)
  {
    imprimirSerial(true, "temp max");
    xRtempmax = temp;
    gravarRtempmax = true;
  }
  // ----------------------------------
  if (((digitalRead(bomb) == HIGH) || (digitalRead(res) == HIGH)) && (!actionon))
  {
    imprimirSerial(true, "temp action");
    xRaction = xRaction + 1;
    actionon = true;
    gravarRaction = true;
  }
  else if ((digitalRead(bomb) == LOW) && (digitalRead(res) == LOW))
  {
    actionon = false;
  }

  // =============================================================================================================






  // =============================================================================================================
  // ----------------------------------------- CONEXÃO WIFI LOCAL ------------------------------------------------
  // =============================================================================================================

  while(WiFi.status() != WL_CONNECTED)   
  {
    //digitalWrite(wificonected, LOW);
    WiFi.begin(ssid, password); 
    imprimirSerial(true, "WIFI E SERVIDOR DESCONECTADO");         
    Serial.print(".");                   

    delay(741);                         
  }
  
  // =============================================================================================================



  // =============================================================================================================
  // --------------------------------------- CONEXÃO SERVIDOR MQTT------------------------------------------------
  // =============================================================================================================

  if (!client.connected())
  {
    imprimirSerial(true, "Client desconectado");
    reconectar();
   
  }
 
  if(!client.loop()){

    imprimirSerial(true, "Entrou no client conect");

    
    }

  // =============================================================================================================

  /*
  if ((WiFi.status() == WL_CONNECTED) && (client.connected()))
  {
    
    digitalWrite(wificonected, HIGH);
    //imprimirSerial(true, "WIFI E SERVIDOR CONECTADO");
    
  }
*/
  // =============================================================================================================
  // ---------------------------------------- CHAMADA DE FUNÇÕES -------------------------------------------------
  // =============================================================================================================


  client.loop();           // CHAMADA FUNÇÃO MQTTT
  comandos_web();          // CHAMADA FUNÇÃO COMANDOS PELO WEB BROSER


  // =============================================================================================================
/*
      imprimirSerial(true,"SENSOR 1");
      imprimirSerial(true,String(temp).c_str());
      imprimirSerial(true,"SENSOR 2");
      imprimirSerial(true, String(tempS2).c_str());              
*/
} 

//#############################################################################################################

// =============================================================================================================
// ======================================           END         ================================================
// =============================================================================================================





                                











