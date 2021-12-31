/*
Nice BusT4
Обмен данными по UART на скорости 19200 8n1
Перед пакетом с данными отправляется break длительностью 10 бит
Содержимое пакета:

0x55 0x0c 0x00 0x03 0x05 0x81 0x01 0x05 0x83 0x01 0x82 0x03 0x64 0xe4 0x0c
  |    |    |    |    |    |    |    |    |    |    |    |    |    |    |
  |    |    |    |    |    |    |    |    |    |    |    |    |    |   CRC - количество бит минус 3
  |    |    |    |    |    |    |    |    |    |    |    |    |    |__ пока не знаю
  |    |    |    |    |    |    |    |    |    |    |    |    |_______ пока не знаю
  |    |    |    |    |    |    |    |    |    |    |    |____________ возможно, команда
  |    |    |    |    |    |    |    |    |    |    |_________________
  |    |    |    |    |    |    |    |    |    |______________________
  |    |    |    |    |    |    |    |    |___________________________
  |    |    |    |    |    |    |    |________________________________
  |    |    |    |    |    |    |_____________________________________
  |    |    |    |    |    |__________________________________________ адрес от кого запрос
  |    |    |    |    |_______________________________________________ ряд от кого запрос
  |    |    |    |____________________________________________________ адрес кому запрос (0xFF для всех)
  |    |    |_________________________________________________________ ряд кому запрос(0xFF для всех)
  |    |______________________________________________________________ CRC - количество бит минус 3
  |___________________________________________________________________ начало пакета
  
  
  Для Oview к адресу всегда прибавляется 80.
  Адрес контроллера ворот без изменений.
  
  Список возможных команд  (некоторые команды могут не поддерживаться разными двигателями. См. Спецификацию вашего двигателя).




  
*/


#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/cover/cover.h"

/* для короткого обращения к членам класса */



namespace esphome {
namespace bus_t4 {


using namespace esphome::cover;




static const uint8_t START_CODE = 0x55; /*стартовый байт пакета */

/* сетевые настройки esp 
Ряд может принимать значения от 0 до 63, по-умолчанию 0
Адрес OVIEW начинается с 8

При объединении в сеть несколько приводов с OXI необходимо для разных приводов указать разные ряды.
В этом случае У OXI ряд должен быть как у привода, которым он кправляет.
*/
static const uint8_t DEF_SERIES = 0x00; 
static const uint8_t DEF_ADDR = 0x83;

/*команды
нас интересует только CMD,
остальные глубоко не изучал
*/
enum Command : uint8_t {
  POS = 0x01, /* запрос и изменение позиции автоматики */
  INF = 0x02, /* возвращает информацию об устройстве */
  LST = 0x03, /* работа со списками автоматик */
  CMD = 0x04, /* отправка команд автоматике */
  GRP = 0x05, /* отправка команд группе автоматик с указанием битовой маски мотора */
  SCN = 0x06, /* работа со сценариями */
  GRC = 0x07,  /* отправка команд группе автоматик, созданных через Nice Screen Configuration Tool */
  LSC = 0x08,  /* работа со списками сценариев */
  LGR = 0x09,  /* работа со списками групп */
  CGR = 0x0A,  /* работа с ксатегориями групп, созданных через Nice Screen Configuration Tool */
};

/* в сочетании с командой определяет тип отправляемого сообщения */
enum flag : uint8_t {
  SET = 0x01, /* запрос на изменение процентного положения позиции автоматики */
  GET = 0x02, /* запрос процентного положения позиции автоматики */
  RSP = 0x03, /* ответ интерфейса, подтверждающий получение команды GET / SET */
  EVT = 0x04, /* ответ интерфейса, отправляющий запрошенную информацию */
  ERR = 0x05, /* ответ интерфейса, указывающий на синтаксическую ошибку в команде GET / SET */
};

/* Команда, которая должна быть выполнена
Используется в запросах и ответах */
enum ControlType : uint8_t {
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
  LOCKED = 0x01,
  UNLOCKED = 0x02,
  MOVING = 0x03,
  OPENING = 0x4,
  CLOSING = 0x5, 
  NO_LIM = 0x6, /* no limits set */
  ERROR = 0x07, /* automation malfunction/error */
  NO_INF = 0x0F, /* no additional information */
};




// создаю класс, наследую членов классов Component и Cover
class NiceBusT4 : public Component, public Cover {
 public:
  void setup() override;
  void loop() override;
  CoverTraits get_traits() override;

/*  void control(const CoverCall &call) override {
    // This will be called every time the user requests a state change.
    if (call.get_position().has_value()) {
      float pos = *call.get_position();
      // Write pos (range 0-1) to cover
      // ...

      // Publish new state
      this->position = pos;
      this->publish_state();
    }
    if (call.get_stop()) {
      // User requested cover stop
    }
  }*/
  
  protected:
  void control(const cover::CoverCall &call) override;
  void send_command_(const uint8_t *data, uint8_t len);
  
//  uint32_t update_interval_{500};
//  uint32_t last_update_{0};
//  uint8_t current_request_{GET_STATUS};
//  uint8_t last_published_op_;
//  float last_published_pos_;
  
};




} // namespace bus_t4
} // namespace esphome