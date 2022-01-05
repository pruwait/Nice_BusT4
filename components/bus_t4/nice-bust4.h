/*
  Nice BusT4
  Обмен данными по UART на скорости 19200 8n1
  Перед пакетом с данными отправляется break длительностью 10 бит
  Содержимое пакета CMD.SET :

  0x55 0x0c 0x00 0x03 0x05 0x81 0x01 0x05 0x83 0x01 0x82 0x03 0x64 0xe4 0x0c
  |    |    |    |    |    |    |    |    |    |    |    |    |    |    |
  |    |    |    |    |    |    |    |    |    |    |    |    |    |   CRC - количество бит минус 3
  |    |    |    |    |    |    |    |    |    |    |    |    |    |__ пока не знаю
  |    |    |    |    |    |    |    |    |    |    |    |    |_______ пока не знаю
  |    |    |    |    |    |    |    |    |    |    |    |____________ возможно, команда
  |    |    |    |    |    |    |    |    |    |    |_________________ flag.cmd (EVT/RSP/SET/GET/ERR)
  |    |    |    |    |    |    |    |    |    |______________________ пока не знаю
  |    |    |    |    |    |    |    |    |___________________________ CRC2
  |    |    |    |    |    |    |    |________________________________ возможно, тип команды
  |    |    |    |    |    |    |_____________________________________ возможно, группа команды CMD
  |    |    |    |    |    |__________________________________________ адрес от кого запрос
  |    |    |    |    |_______________________________________________ ряд (серия) от кого запрос
  |    |    |    |____________________________________________________ адрес кому запрос (0xFF для всех)
  |    |    |_________________________________________________________ ряд (серия) кому запрос(0xFF для всех)
  |    |______________________________________________________________ CRC - количество бит минус 3
  |___________________________________________________________________ начало пакета
 

  Для Oview к адресу всегда прибавляется 80.
  Адрес контроллера ворот без изменений.


Подключение

BusT4                       ESP8266

Стенка устройства        Rx Tx GND
9  7  5  3  1  
10 8  6  4  2
место для кабеля
            1 ---------- Tx
            2 ---------- GND
			4 ---------- Rx
			5 ---------- +24V
			
			
			
			
Из мануала nice_dmbm_integration_protocol.pdf

• ADR: это адрес сети NICE, где находятся устройства, которыми вы хотите управлять. Это может быть значение от 1 до 63 (от 1 до 3F).
Это значение должно быть в HEX. Если адресатом является модуль интеграции на DIN-BAR, это значение равно 0 (adr = 0), если адресатом
является интеллектуальным двигателем, это значение равно 1 (adr = 1).
• EPT: это адрес двигателя Nice, входящего в сетевой ADR. Это может быть значение от 1 до 127. Это значение должно быть в HEX.
• CMD: это команда, которую вы хотите отправить по назначению (ADR, EPT). Ценность зависит от того, что вы хотите сделать.
• PRF: команда установки профиля.
• FNC: это функция, которую вы хотите отправить по назначению (ADR, EPT).
• EVT: это событие, которое запускается в пункт назначения (ADR, EPT).



*/ 


#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h"           // для добавления Action
#include "esphome/components/cover/cover.h"
#include <HardwareSerial.h>
#include "esphome/core/helpers.h"              // парсим строки встроенными инструментами  



namespace esphome {
namespace bus_t4 {

/* для короткого обращения к членам класса */
using namespace esphome::cover;

#define _UART_NO UART0 /* номер uart */
#define TX_P 1         /* пин Tx */
#define baud_work 19200 /* рабочий бодрэйт */
#define baud_break 9200 /* бодрэйт для длинного импульса перед пакетом */


static const uint8_t START_CODE = 0x55; /*стартовый байт пакета */

/* сетевые настройки esp
  Ряд может принимать значения от 0 до 63, по-умолчанию 0
  Адрес OVIEW начинается с 8

  При объединении в сеть несколько приводов с OXI необходимо для разных приводов указать разные ряды.
  В этом случае У OXI ряд должен быть как у привода, которым он управляет.
*/
static const uint8_t DEF_SERIES = 0x00;
static const uint8_t DEF_ADDR = 0x83;


// осталось от dooya, нужно переписать
enum ReadType : uint8_t {
  GET_POSITION = 0x02,
  GET_STATUS = 0x05,
};



/* Группы пакетов
  пока нас интересует только CMD,
  остальные глубоко не изучал и номера не проверял
*/
enum grp_packet : uint8_t {
  CMD = 0x01,  /* номер проверен, отправка команд автоматике */
  LSC = 0x02,  /* работа со списками сценариев */
  LST = 0x03,  /* работа со списками автоматик */
  POS = 0x04,  /* запрос и изменение позиции автоматики */
  GRP = 0x05,  /* отправка команд группе автоматик с указанием битовой маски мотора */
  SCN = 0x06,  /* работа со сценариями */
  GRC = 0x07,  /* отправка команд группе автоматик, созданных через Nice Screen Configuration Tool */
  INF = 0x08,  /* возвращает информацию об устройстве */
  LGR = 0x09,  /* работа со списками групп */
  CGR = 0x0A,  /* работа с категориями групп, созданных через Nice Screen Configuration Tool */
};





/* в сочетании с командой определяет тип отправляемого сообщения */
enum flag_cmd : uint8_t {

  GET = 0x02, /* запрос текущего состояния параметра */
  SET = 0x05, /* запрос на изменение параметра */
  EVT = 0x04, /* ответ интерфейса, отправляющий запрошенную информацию */
  ERR = 0x01, /* ответ интерфейса, указывающий на синтаксическую ошибку в команде GET / SET */
  RSP = 0x07, /* ответ интерфейса, подтверждающий получение команды GET / SET */
};

/* Команда, которая должна быть выполнена
  Используется в запросах и ответах */
enum run_cmd : uint8_t {
  SBS = 0x01,    /* Step by Step */
  STOP = 0x02,   /* Stop */
  OPEN = 0x03,   /* Open */
  CLOSE = 0x04,  /* Close */
  P_OPN1 = 0x05, /* Partial opening 1 - частичное открывание, режим калитки */
  P_OPN2 = 0x06, /* Partial opening 2 */
  P_OPN3 = 0x07, /* Partial opening 3 */
};


/* используется в ответах EVT */
enum Status : uint8_t {
  LOCKED   = 0x01,
  UNLOCKED = 0x02,
  MOVING   = 0x03,
  OPENING  = 0x4,
  CLOSING  = 0x5,
  NO_LIM   = 0x6, /* no limits set */
  ERROR    = 0x07, /* automation malfunction/error */
  NO_INF   = 0x0F, /* no additional information */
};


// тело пакета RAW CMD
struct packet_ping_answer_body_t {
  uint8_t byte_55;              // первый байт всегда 0x55
  uint8_t crc11;                // CRC1, количество байт минус три
  uint8_t for_series;           // серия кому пакет ff = всем
  uint8_t for_address;          // адрес кому пакет ff = всем
  uint8_t from_series;           // серия от кого пакет
  uint8_t from_address;          // адрес от кого пакет
  uint8_t grp_packet;           // группа пакета
  uint8_t byte_8;              // пока не знаю
  uint8_t crc2;                // CRC2, последовательный xor байт со второго по восьмой
  uint8_t byte_10;            // пока не знаю
  uint8_t flag_cmd;            // в сочетании с командой определяет тип отправляемого сообщения
  uint8_t run_cmd;            // Команда, которая должна быть выполнена
  uint8_t byte_13;            // пока не знаю
  uint8_t byte_14;            // пока не знаю
  uint8_t CRC12;            // CRC1, количество байт минус три

};



// создаю класс, наследую членов классов Component и Cover
class NiceBusT4 : public Component, public Cover {
  public:
    void setup() override;
    void loop() override;
    void dump_config() override;
    void send_open();
    void send_raw_cmd(std::string data);


    void set_address(uint16_t address) {
      uint8_t start_code = START_CODE;
      uint8_t address_h = (uint8_t)(address >> 8);
      uint8_t address_l = (uint8_t)(address & 0xFF);
      //    this->header_ = {&start_code, &address_h, &address_l};
    }
    void set_update_interval(uint32_t update_interval) {  // интервал получения статуса привода
      this->update_interval_ = update_interval;
    }
    //  void on_rs485_data(const std::vector<uint8_t> &data) override;
    cover::CoverTraits get_traits() override;

  protected:
    void control(const cover::CoverCall &call) override;
    void send_command_(const uint8_t *data, uint8_t len);


    uint32_t update_interval_{500};
    uint32_t last_update_{0};
    uint8_t current_request_{GET_STATUS}; // осталось от dooya, возможно придется переписать согласно статусам от nice
    uint8_t last_published_op_;
    float last_published_pos_;

    // переменные для uart
    uint8_t _uart_nr;
    uart_t* _uart = nullptr;

    
    std::vector<char> raw_cmd_prepare (std::string data);             // подготовка введенных пользователем данных для возможности отправки
	std::string format_hex_pretty(std::vector<char> data);          // для более красивого вывода hex строк
	std::string format_hex_pretty(const char *data, size_t length);  // для более красивого вывода hex строк
	char format_hex_pretty_char(char v) ;                           // для более красивого вывода hex строк
    void send_array_cmd (std::vector<char> data);
    void send_array_cmd (const char *data, size_t len);
    //uint8_t *raw_cmd = nullptr;                                     // указатель на данные для отправки
	
	

}; //класс

} // namespace bus_t4
} // namespace esphome
