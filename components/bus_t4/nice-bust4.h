/*
  Nice BusT4
  Обмен данными по UART на скорости 19200 8n1
  Перед пакетом с данными отправляется break длительностью 519us (10 бит)
  Содержимое пакета, которое удалось понять, описано в структуре packet_cmd_body_t

 

  Для Oview к адресу всегда прибавляется 80.
  Адрес контроллера ворот без изменений.


Подключение

BusT4                       ESP8266

Стенка устройства        Rx Tx GND
9  7  5  3  1  
10 8  6  4  2
место для кабеля
            1 ---------- Rx
            2 ---------- GND
            4 ---------- Tx
            5 ---------- +24V




Из мануала nice_dmbm_integration_protocol.pdf

• ADR: это адрес сети NICE, где находятся устройства, которыми вы хотите управлять. Это может быть значение от 1 до 63 (от 1 до 3F).
Это значение должно быть в HEX. Если адресатом является модуль интеграции на DIN-BAR, это значение равно 0 (adr = 0), если адресат
является интеллектуальным двигателем, это значение равно 1 (adr = 1).
• EPT: это адрес двигателя Nice, входящего в сетевой ADR. Это может быть значение от 1 до 127. Это значение должно быть в HEX.
• CMD: это команда, которую вы хотите отправить по назначению (ADR, EPT).
• PRF: команда установки профиля.
• FNC: это функция, которую вы хотите отправить по назначению (ADR, EPT).
• EVT: это событие, которое отправляется в пункт назначения (ADR, EPT).



*/


#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h"           // для добавления Action
#include "esphome/components/cover/cover.h"
#include <HardwareSerial.h>
#include "esphome/core/helpers.h"              // парсим строки встроенными инструментами
#include <queue>                               // для работы с очередью



namespace esphome {
namespace bus_t4 {

/* для короткого обращения к членам класса */
using namespace esphome::cover;
//using esp8266::timeoutTemplate::oneShotMs;


static const int _UART_NO=UART0; /* номер uart */
static const int TX_P = 21;         /* пин Tx */
static const uint32_t BAUD_BREAK = 9200; /* бодрэйт для длинного импульса перед пакетом */
static const uint32_t BAUD_WORK = 19200; /* рабочий бодрэйт */
static const uint8_t START_CODE = 0x55; /*стартовый байт пакета */

static const float CLOSED_POSITION_THRESHOLD = 0.007;  // Значение положения привода в процентах, ниже которого ворота считаются полностью закрытыми
static const uint32_t POSITION_UPDATE_INTERVAL = 500;  // Интервал обновления текущего положения привода, мс

/* сетевые настройки esp
  Ряд может принимать значения от 0 до 63, по-умолчанию 0
  Адрес OVIEW начинается с 8

  При объединении в сеть несколько приводов с OXI необходимо для разных приводов указать разные ряды.
  В этом случае У OXI ряд должен быть как у привода, которым он управляет.
*/






/* Тип сообщения пакетов
  пока нас интересует только CMD и INF
  остальные глубоко не изучал и номера не проверял
  6-й байт пакетов CMD и INF
*/
enum mes_type : uint8_t {
  CMD = 0x01,  /* номер проверен, отправка команд автоматике */
//  LSC = 0x02,  /* работа со списками сценариев */
//  LST = 0x03,  /* работа со списками автоматик */
//  POS = 0x04,  /* запрос и изменение позиции автоматики */
//  GRP = 0x05,  /* отправка команд группе автоматик с указанием битовой маски мотора */
//  SCN = 0x06,  /* работа со сценариями */
//  GRC = 0x07,  /* отправка команд группе автоматик, созданных через Nice Screen Configuration Tool */
  INF = 0x08,  /* возвращает или устанавливает информацию об устройстве */
//  LGR = 0x09,  /* работа со списками групп */
//  CGR = 0x0A,  /* работа с категориями групп, созданных через Nice Screen Configuration Tool */
};




/* 
меню команды в иерархии oview
9-й байт пакетов CMD
*/
enum cmd_mnu  : uint8_t {
  CONTROL = 0x01,
};


/* используется в ответах STA*/
enum sub_run_cmd2 : uint8_t {
  STA_OPENING = 0x02,
  STA_CLOSING = 0x03,
       OPENED = 0x04,
       CLOSED = 0x05,
      ENDTIME = 0x06,  // закончен маневр по таймауту
      STOPPED = 0x08,
  PART_OPENED = 0x10,  // частичное открывание
};

/* Ошибки */
enum errors_byte  : uint8_t {
  NOERR = 0x00, // Нет ошибок
  FD = 0xFD,    // Нет команды для этого устройства
  };

// Типы моторов
enum motor_type  : uint8_t {
  SLIDING = 0x01, 
  SECTIONAL = 0x02,
  SWING = 0x03,
  BARRIER = 0x04,
  UPANDOVER = 0x05, // up-and-over подъемно-поворотные ворота
  };

//  девятый байт
enum whose_pkt  : uint8_t {
  FOR_ALL = 0x00,  /* пакет для/от всех */
  FOR_CU = 0x04,  /* пакет для/от блока управления */
  FOR_OXI = 0x0A,  /* пакет для/от приемника OXI */
  };
	
// десятый байт GET/SET пакетов EVT, для пакетов CMD встречалось только значение RUN
enum command_pkt  : uint8_t {
  TYPE_M         = 0x00,   /* Запрос типа привода */
  INF_STATUS     = 0x01, //	Состояние ворот (Открыто/Закрыто/Остановлено)	
  WHO	         = 0x04,  /* Кто в сети?     */
  MAC            = 0x07,    // mac address.
  MAN            = 0x08,   // manufacturer.
  PRD            = 0x09,   // product.
  INF_SUPPORT    = 0x10, //  Доступные INF команды
  HWR            = 0x0a,   // hardware version.
  FRM            = 0x0b,   // firmware version.
  DSC            = 0x0c,   // description.
  CUR_POS        = 0x11,  // текущее условное положение автоматики, DPRO924 после этого ждет выставления положений
  MAX_OPN        = 0x12,   // Максимально возможное открывание по энкодеру.
  POS_MAX        = 0x18,   // Максимальное положение (открывания) по энкодеру
  POS_MIN        = 0x19,   // Минимальное положение (закрывания) по энкодеру	
  INF_P_OPN1     = 0x21, //	Частичное открывание1 
  INF_P_OPN2     = 0x22, //	Частичное открывание2
  INF_P_OPN3     = 0x23, //	Частичное открывание3
  INF_SLOW_OPN   = 0x24, // Замедление в открывании
  INF_SLOW_CLS   = 0x25, // Замедление в закрывании	
  OPN_OFFSET     = 0x28, /* Задержка открывания  open offset */
  CLS_OFFSET     = 0x29, /* Задержка закрывания  close offset */
  OPN_DIS        = 0x2a, /* Основные параметры - Разгрузка открытия Open discharge */
  CLS_DIS        = 0x2b, /* Основные параметры - Разгрузка закрытия Close discharge */
  REV_TIME       = 0x31, /* Основные параметры - Длительность реверса (Brief inversion value) */
  OPN_PWR        = 0x4A,    /* Основные параметры - Управление усилием - Усилие открывания */	  	  	  	  	  
  CLS_PWR        = 0x4B,    /* Основные параметры - Управление усилием - Усилие закрывания */	  	  	  	  	  	  
  SPEED_OPN      = 0x42,    /* Основные параметры - Настройка скорости - Скорость открывания */	  	  	  	  	  	  	  
  SPEED_CLS      = 0x43,    /* Основные параметры - Настройка скорости - Скорость закрывания */	  
  SPEED_SLW_OPN  = 0x45,    /* Основные параметры - Настройка скорости - Скорость замедленного открывания */	
  SPEED_SLW_CLS  = 0x46,    /* Основные параметры - Настройка скорости - Скорость замедленного закрывания */	
  OUT1           = 0x51,  /* Настройка выходов */	  
  OUT2           = 0x52,  /* Настройка выходов */	  	  
  LOCK_TIME      = 0x5A,  /* Настройка выходов - Время работы замка */
  S_CUP_TIME     = 0x5C,  /* Настройка выходов - Время работы присоски Suction Cup Time*/	  
  LAMP_TIME      = 0x5B,  /* Настройка выходов - Время работы лампы освещения courtesy light Time*/
  COMM_SBS       = 0x61,  /* Настройка команд - Пошагово */	  
  COMM_POPN      = 0x62,  /* Настройка команд - Открыть частично */	  	  
  COMM_OPN       = 0x63,  /* Настройка команд - Открыть */	  	  	  
  COMM_CLS       = 0x64,  /* Настройка команд - Закрыть */	  
  COMM_STP       = 0x65,  /* Настройка команд - СТОП */		  
  COMM_PHOTO     = 0x68,  /* Настройка команд - Фото */		  
  COMM_PHOTO2    = 0x69,  /* Настройка команд - Фото2 */
  COMM_PHOTO3    = 0x6A,  /* Настройка команд - Фото3 */
  COMM_OPN_STP   = 0x6B,  /* Настройка команд - Стоп при открывании */	  
  COMM_CLS_STP   = 0x6C,  /* Настройка команд - Стоп при закрывании */	 
  IN1            = 0x71,  /* Настройка входов */
  IN2            = 0x72,  /* Настройка входов */
  IN3            = 0x73,  /* Настройка входов */
  IN4            = 0x74,  /* Настройка входов */
  COMM_LET_OPN   = 0x78,  /* Настройка команд - Помеха открыванию */	  	  	  
  COMM_LET_CLS   = 0x79,  /* Настройка команд - Помеха закрыванию */	  	  	  	  

  AUTOCLS        = 0x80,    /* Основные параметры - Автозакрывание */
  P_TIME         = 0x81,    /* Основные параметры - Время паузы */
  PH_CLS_ON      = 0x84,    /* Основные параметры - Закрыть после Фото - Активно */	  
  PH_CLS_VAR     = 0x86,    /* Основные параметры - Закрыть после Фото - Режим */	  	  
  PH_CLS_TIME    = 0x85,    /* Основные параметры - Закрыть после Фото - Время ожидания */	  	  	  
  ALW_CLS_ON     = 0x88,    /* Основные параметры - Всегда закрывать - Активно */	  	  
  ALW_CLS_VAR    = 0x8A,    /* Основные параметры - Всегда закрывать - Режим */	  
  ALW_CLS_TIME   = 0x89,    /* Основные параметры - Всегда закрывать - Время ожидания */	  	  	  
  STAND_BY_ACT   = 0x8c,    /* Основные параметры - Режим ожидания - Активно  ON / OFF */
  WAIT_TIME      = 0x8d,    /* Основные параметры - Режим ожидания - Время ожидания */
  STAND_BY_MODE  = 0x8e,    /* Основные параметры - Режим ожидания - Режим -  safety = 0x00, bluebus=0x01, all=0x02*/
  START_ON       = 0x90,    /* Основные параметры - Настройка пуска - Активно */		  	  
  START_TIME     = 0x91,    /* Основные параметры - Настройка пуска - Время пуска */		  	  	  
  SLOW_ON        = 0xA2,    /* Основные параметры - Замедление */	
  DIS_VAL        = 0xA4,    /* Положение - Значение недопустимо disable value */

  BLINK_ON       = 0x94,    /* Основные параметры - Предмерцание - Активно */		  	  	  	  
  BLINK_OPN_TIME = 0x95,    /* Основные параметры - Предмерцание - Время при открывании */		  	  	  	  	  
  BLINK_CLS_TIME = 0x99,    /* Основные параметры - Предмерцание - Время при закрывании */
  OP_BLOCK       = 0x9a,    /* Основные параметры - Блокирование мотора (Operator block)*/
  KEY_LOCK       = 0x9c,    /* Основные параметры - Блокирование кнопок */
  T_VAL          = 0xB1,    /*Alarm threshold value Порог до обслуживания в количестве маневров*/
  P_COUNT        = 0xB2,    /* Partial count Выделенный счетчик*/
  C_MAIN         = 0xB4,    /* Cancel maintenance Отмена обслуживания */
  DIAG_BB        = 0xD0,     /*   DIAGNOSTICS of bluebus devices */  
  INF_IO         = 0xD1,    /*	состояние входов-выходов	*/
  DIAG_PAR       = 0xD2,    /*  DIAGNOSTICS of other parameters   */
  
  
  
  


  

  CUR_MAN = 0x02,  // Текущий Маневр
  SUBMNU  = 0x04,  // Подменю
  STA = 0xC0,   // статус в движении
  MAIN_SET = 0x80,   // Основные параметры
  RUN = 0x82,   // Команда для выполнения	  

  };	

	
/* run cmd 11-й байт EVT пакетов */
enum run_cmd  : uint8_t {
  SET = 0xA9,  /* запрос на изменение параметров */
  GET = 0x99,   /* запрос на получение параметров */
  GET_SUPP_CMD = 0x89, /* получить поддерживаемые команды */
  };


/* Команда, которая должна быть выполнена.   
11-й байт пакета CMD
Используется в запросах и ответах */
enum control_cmd : uint8_t { 
  SBS = 0x01,    /* Step by Step */
  STOP = 0x02,   /* Stop */
  OPEN = 0x03,   /* Open */
  CLOSE = 0x04,  /* Close */
  P_OPN1 = 0x05, /* Partial opening 1 - частичное открывание, режим калитки */
  P_OPN2 = 0x06, /* Partial opening 2 */
  P_OPN3 = 0x07, /* Partial opening 3 */
  RSP = 0x19, /* ответ интерфейса, подтверждающий получение команды  */
  EVT = 0x29, /* ответ интерфейса, отправляющий запрошенную информацию */
 
  P_OPN4 = 0x0b, /* Partial opening 4 - Коллективно */
  P_OPN5 = 0x0c, /* Partial opening 5 - Приоритет пошагово */
  P_OPN6 = 0x0d, /* Partial opening 6 - Открыть и блокировать */
  UNLK_OPN = 0x19, /* Разблокировать и открыть */
  CLS_LOCK = 0x0E, /* Закрыть и блокировать */
  UNLCK_CLS = 0x1A, /*  Разблокировать и Закрыть */
  LOCK = 0x0F, /* Блокировать*/
  UNLOCK = 0x10, /* Разблокировать */
  LIGHT_TIMER = 0x11, /* Таймер освещения */
  LIGHT_SW = 0x12, /* Освещение вкл/выкл */
  HOST_SBS = 0x13, /* Ведущий SBS */
  HOST_OPN = 0x14, /* Ведущий открыть */
  HOST_CLS = 0x15, /* Ведущий закрыть */
  SLAVE_SBS = 0x16, /*  Ведомый SBS */
  SLAVE_OPN = 0x17, /* Ведомый открыть */
  SLAVE_CLS = 0x18, /* Ведомый закрыть */
  AUTO_ON = 0x1B, /* Автооткрывание активно */
  AUTO_OFF = 0x1C, /* Автооткрывание неактивно	  */
  
};
	
	
	
	
	
	
/* Информация для лучшего понимания состава пакетов в протоколе */
// тело пакета запроса CMD
// пакеты с размером тела 0x0c=12 байт 
	/*
struct packet_cmd_body_t {
  uint8_t byte_55;              // Заголовок, всегда 0x55
  uint8_t pct_size1;                // размер тела пакета (без заголовка и CRC. Общее количество  байт минус три), для команд = 0x0c
  uint8_t for_series;           // серия кому пакет ff = всем
  uint8_t for_address;          // адрес кому пакет ff = всем
  uint8_t from_series;           // серия от кого пакет
  uint8_t from_address;          // адрес от кого пакет
  uint8_t mes_type;           // тип сообщения, 1 = CMD, 8 = INF
  uint8_t mes_size;              // количество байт дальше за вычетом двух байт CRC в конце, для команд = 5
  uint8_t crc1;                // CRC1, XOR шести предыдущих байт
  uint8_t cmd_mnu;                // Меню команды. cmd_mnu = 1 для команд управления
  uint8_t setup_submnu;            // Подменю, в сочетании с группой команды определяет тип отправляемого сообщения
  uint8_t control_cmd;            // Команда, которая должна быть выполнена
  uint8_t offset;            //  Смещение для ответов. Влияет на запросы вроде списка поддерживаемых комманд
  uint8_t crc2;            // crc2, XOR четырех предыдущих байт
  uint8_t pct_size2;            // размер тела пакета (без заголовка и CRC. Общее количество  байт минус три), для команд = 0x0c

};





// тело пакета ответа RSP
// пакеты с размером тела 0x0e=14 байт 
struct packet_rsp_body_t {
  uint8_t byte_55;              // Заголовок, всегда 0x55
  uint8_t pct_size1;                // размер тела пакета (без заголовка и CRC. Общее количество  байт минус три), >= 0x0e
  uint8_t to_series;           // серия кому пакет ff = всем
  uint8_t to_address;          // адрес кому пакет ff = всем
  uint8_t from_series;           // серия от кого пакет
  uint8_t from_address;          // адрес от кого пакет
  uint8_t mes_type;           // тип сообщения, для этих пакетов всегда  8 = INF
  uint8_t mes_size;              // количество байт дальше за вычетом двух байт CRC в конце, для команд = 5
  uint8_t crc1;                // CRC1, XOR шести предыдущих байт
  uint8_t cmd_mnu;                // Меню команды. cmd_mnu = 1 для команд управления
  uint8_t sub_inf_cmd;            // Из какого подменю получил команду. Значение меньше на 0x80, чем первоначальное подменю
  uint8_t sub_run_cmd;            // Какую команду получил. Значение больше на 0x80, чем полученная команда
  uint8_t hb_data;             // данные, старший бит
  uint8_t lb_data;            // данные, младший бит
  uint8_t err;               // Ошибки
  uint8_t crc2;            // crc2, XOR четырех предыдущих байт
  uint8_t pct_size2;            // размер тела пакета (без заголовка и CRC. Общее количество  байт минус три), >= 0x0e

};
	
 // тело пакета ответа с данными EVT
 
 struct packet_evt_body_t {
  uint8_t byte_55;              // Заголовок, всегда 0x55
  uint8_t pct_size1;                // размер тела пакета (без заголовка и CRC. Общее количество  байт минус три), >= 0x0e
  uint8_t to_series;           // серия кому пакет ff = всем
  uint8_t to_address;          // адрес кому пакет ff = всем
  uint8_t from_series;           // серия от кого пакет
  uint8_t from_address;          // адрес от кого пакет
  uint8_t mes_type;           // тип сообщения, для этих пакетов всегда  8 = INF
  uint8_t mes_size;              // количество байт дальше за вычетом двух байт CRC в конце, для команд = 5
  uint8_t crc1;                // CRC1, XOR шести предыдущих байт
  uint8_t whose;                // Чей пакет. Варианты: 00 - общий, 04 - контроллера привода, 0A - приемника OXI
  uint8_t setup_submnu;            // Из какого подменю получил команду. Значение равно первоначальному подменю
  uint8_t sub_run_cmd;            // На какую команду отвечаем. Значение меньше на 0x80, чем отправленная ранее команда
  uint8_t next_data;            // Следующий блок данных
  uint8_t err;               // Ошибки
  uint8_t data_blk;            // Блок данных, может занимать несколько байт
  uint8_t crc2;            // crc2, XOR всех предыдущих байт до девятого (Чей пакет)
  uint8_t pct_size2;            // размер тела пакета (без заголовка и CRC. Общее количество  байт минус три), >= 0x0e

};
 
 
*/

enum position_hook_type : uint8_t {
     IGNORE = 0x00,
    STOP_UP = 0x01,
  STOP_DOWN = 0x02
 };

// создаю класс, наследую членов классов Component и Cover
class NiceBusT4 : public Component, public Cover {
  public:
	
    // настройки привода
    bool autocls_flag; // Автозакрывание - L1
    bool photocls_flag; // Закрыть после фото - L2
    bool alwayscls_flag; // Всегда закрывать - L3
    bool init_ok = false; //  определение привода при включении
    bool is_walky = false; // для walky отличается команда запроса положения
    bool is_robus = false; // для robus не нужно переодически запрашивать позицию
		
    void setup() override;
    void loop() override;
    void dump_config() override; // для вывода в лог информации об оборудовнии

    void send_raw_cmd(std::string data);
    void send_cmd(uint8_t data) {this->tx_buffer_.push(gen_control_cmd(data));}	
    void send_inf_cmd(std::string to_addr, std::string whose, std::string command, std::string type_command,  std::string next_data, bool data_on, std::string data_command); // длинная команда
    void set_mcu(std::string command, std::string data_command); // команда контроллеру мотора
		

    void set_class_gate(uint8_t class_gate) { class_gate_ = class_gate; }
    
 /*   void set_update_interval(uint32_t update_interval) {  // интервал получения статуса привода
      this->update_interval_ = update_interval;
    }*/

    cover::CoverTraits get_traits() override;

  protected:
    void control(const cover::CoverCall &call) override;
    void send_command_(const uint8_t *data, uint8_t len);
    void request_position(void);  // Запрос условного текущего положения привода
    void update_position(uint16_t newpos);  // Обновление текущего положения привода

    uint32_t last_position_time{0};  // Время последнего обновления текущего положения
    uint32_t update_interval_{500};
    uint32_t last_update_{0};
    uint32_t last_uart_byte_{0};

    CoverOperation last_published_op;  // Последние опубликованные состояние и положение
    float last_published_pos{-1};

    void publish_state_if_changed(void);

    uint8_t position_hook_type{IGNORE};  // Флаг и позиция установки заданного положения привода
    uint16_t position_hook_value;

    uint8_t class_gate_ = 0x55; // 0x01 sliding, 0x02 sectional, 0x03 swing, 0x04 barrier, 0x05 up-and-over
//    uint8_t last_init_command_;
	
    bool init_cu_flag = false;	
    bool init_oxi_flag = false;	

	
    // переменные для uart
    uint8_t _uart_nr;
    uart_t* _uart = nullptr;
    uint16_t _max_opn = 0;  // максимальная позиция энкодера или таймера
    uint16_t _pos_opn = 2048;  // позиция открытия энкодера или таймера, не для всех приводов.
    uint16_t _pos_cls = 0;  // позиция закрытия энкодера или таймера, не для всех приводов
    uint16_t _pos_usl = 0;  // условная текущая позиция энкодера или таймера, не для всех приводов	
    // настройки заголовка формируемого пакета
    uint8_t addr_from[2] = {0x00, 0x66}; //от кого пакет, адрес bust4 шлюза
    uint8_t addr_to[2]; // = 0x00ff;	 // кому пакет, адрес контроллера привода, которым управляем
    uint8_t addr_oxi[2]; // = 0x000a;	 // адрес приемника

    std::vector<uint8_t> raw_cmd_prepare (std::string data);             // подготовка введенных пользователем данных для возможности отправки	
	
    // генерация inf команд
    std::vector<uint8_t> gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, const std::vector<uint8_t> &data, size_t len);	 // все поля
    std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd) {return gen_inf_cmd(this->addr_to[0], this->addr_to[1], whose, inf_cmd, run_cmd, 0x00, {0x00}, 0 );} // для команд без данных
    std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, std::vector<uint8_t> data){
	    return gen_inf_cmd(this->addr_to[0], this->addr_to[1], whose, inf_cmd, run_cmd, next_data, data, data.size());} // для команд c данными
    std::vector<uint8_t> gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data){
	    return gen_inf_cmd(to_addr1, to_addr2, whose, inf_cmd, run_cmd, next_data, {0x00}, 0);} // для команд с адресом и без данных 	
    	    
    // генерация cmd команд
    std::vector<uint8_t> gen_control_cmd(const uint8_t control_cmd);	    	
	
    void init_device (const uint8_t addr1, const uint8_t addr2, const uint8_t device );
    void send_array_cmd (std::vector<uint8_t> data);	
    void send_array_cmd (const uint8_t *data, size_t len);


    void parse_status_packet (const std::vector<uint8_t> &data); // разбираем пакет статуса
    
    void handle_char_(uint8_t c);                                         // обработчик полученного байта
    void handle_datapoint_(const uint8_t *buffer, size_t len);          // обработчик полученных данных
    bool validate_message_();                                         // функция проверки полученного сообщения

    std::vector<uint8_t> rx_message_;                          // здесь побайтно накапливается принятое сообщение
    std::queue<std::vector<uint8_t>> tx_buffer_;             // очередь команд для отправки	
    bool ready_to_tx_{true};	                           // флаг возможности отправлять команды
	
    std::vector<uint8_t> manufacturer_ = {0x55, 0x55};  // при инициализации неизвестный производитель
    std::vector<uint8_t> product_;
    std::vector<uint8_t> hardware_;
    std::vector<uint8_t> firmware_;
    std::vector<uint8_t> description_;	
    std::vector<uint8_t> oxi_product;
    std::vector<uint8_t> oxi_hardware;
    std::vector<uint8_t> oxi_firmware;
    std::vector<uint8_t> oxi_description;	

}; //класс

} // namespace bus_t4
} // namespace esphome
