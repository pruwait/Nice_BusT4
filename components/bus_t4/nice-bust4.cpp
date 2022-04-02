#include "nice-bust4.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"  // для использования вспомогательных функция работ со строками






namespace esphome {
namespace bus_t4 {

static const char *TAG = "bus_t4.cover";

using namespace esphome::cover;


/*
  uint16_t crc16(const uint8_t *data, uint8_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++) {
      if ((crc & 0x01) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
  } */ 

CoverTraits NiceBusT4::get_traits() {
  auto traits = CoverTraits();
  traits.set_supports_position(true);
  return traits;
}


/*
Пока отправляю дампы команд OVIEW
Может, со временем буду учиться генерировать свои команды
SBS               55 0c 00 ff 00 66 01 05 9D 01 82 01 64 E6 0c
STOP              55 0c 00 ff 00 66 01 05 9D 01 82 02 64 E5 0c
OPEN              55 0c 00 ff 00 66 01 05 9D 01 82 03 00 80 0c
CLOSE             55 0c 00 ff 00 66 01 05 9D 01 82 04 64 E3 0c
PARENTAL OPEN 1   55 0c 00 ff 00 66 01 05 9D 01 82 05 64 E2 0c
PARENTAL OPEN 2   55 0c 00 ff 00 66 01 05 9D 01 82 06 64 E1 0c



*/



void NiceBusT4::control(const CoverCall &call) {  
    if (call.get_stop()) {
     // uint8_t data[2] = {CONTROL, STOP};
	  std::string data = "55 0c 00 03 00 66 01 05 61 01 82 02 64 E5 0c"; // пока здесь дамп sbs
	  std::vector < char > v_cmd = raw_cmd_prepare (data);
      this->send_array_cmd (&v_cmd[0], v_cmd.size());
      
    } else if (call.get_position().has_value()) {
      auto pos = *call.get_position();
      if (pos != this->position) {
        if (pos == COVER_OPEN) {
          std::string data = "55 0c 00 ff 00 66 01 05 9D 01 82 03 00 80 0c"; // пока здесь дамп open
	      std::vector < char > v_cmd = raw_cmd_prepare (data);
          this->send_array_cmd (&v_cmd[0], v_cmd.size());
        } else if (pos == COVER_CLOSED) {
          std::string data = "55 0c 00 03 00 66 01 05 61 01 82 04 64 E3 0c"; // пока здесь дамп close
	      std::vector < char > v_cmd = raw_cmd_prepare (data);
          this->send_array_cmd (&v_cmd[0], v_cmd.size());
        } /*else {
          uint8_t data[3] = {CONTROL, SET_POSITION, (uint8_t)(pos * 100)};
          this->send_command_(data, 3);
        }*/
      }
    }
}

void NiceBusT4::setup() {
  _uart =  uart_init(_UART_NO, BAUD_WORK, SERIAL_8N1, SERIAL_FULL, TX_P, 256, false);
  
   this->last_init_command_ = 0;
 // запрос типа привода 
//  send_raw_cmd("55.0d.FF.FF.00.66.08.06.68.04.00.99.00.00.9D.0d");
  //std::string  to_hex = "55.0d.FF.FF.00.66.08.06.68.04.00.99.00.00.9D.0d";
//  std::vector < char > v_cmd = raw_cmd_prepare ("55.0d.FF.FF.00.66.08.06.68.04.00.99.00.00.9D.0d");
//  send_array_cmd (&v_cmd[0], v_cmd.size());
//  delay(1000);
  
  // запрос производителя
//  v_cmd = raw_cmd_prepare ("55.0d.FF.FF.00.66.08.06.68.00.08.99.00.00.91.0d");
//  send_array_cmd (&v_cmd[0], v_cmd.size());
//  delay(1000);
  
    // запрос прошивки
//  v_cmd = raw_cmd_prepare ("55.0d.FF.FF.00.66.08.06.68.00.0b.99.00.00.92.0d");
//  send_array_cmd (&v_cmd[0], v_cmd.size());
  
  //запрос продукта
//  v_cmd = raw_cmd_prepare ("55.0d.FF.FF.00.66.08.06.68.00.09.99.00.00.90.0d");
//  send_array_cmd (&v_cmd[0], v_cmd.size());
  
  //запрос железа
//  v_cmd = raw_cmd_prepare ("55.0d.FF.FF.00.66.08.06.68.00.0a.99.00.00.93.0d");
//  send_array_cmd (&v_cmd[0], v_cmd.size());
  
  
  //запрос описания
//v_cmd = raw_cmd_prepare ("55.0d.FF.FF.00.66.08.06.68.00.0c.99.00.00.95.0d");
//send_array_cmd (&v_cmd[0], v_cmd.size());
  
  
  
 // ESP_LOGCONFIG(TAG, "Setting up Nice ESP BusT4...");
  /*  if (this->header_.empty()) {                                                             // заполняем адреса значениями по умолчанию, если они не указаны явно в конфигурации yaml
      this->header_ = {(uint8_t *)&START_CODE, (uint8_t *)&DEF_ADDR, (uint8_t *)&DEF_ADDR};
    }*/
}

void NiceBusT4::loop() {
  
  if ((millis() - this->last_update_) > this->update_interval_) {
     
    if (this->last_init_command_ <30 ) {
      if (last_init_command_ == 2  ) send_raw_cmd("55.0d.FF.FF.00.66.08.06.68.04.00.99.00.00.9D.0d"); // запрос типа привода 
      if (last_init_command_ == 6  ) send_raw_cmd("55.0d.FF.FF.00.66.08.06.68.00.08.99.00.00.91.0d"); // запрос производителя
      if (last_init_command_ == 10  ) send_raw_cmd("55.0d.FF.FF.00.66.08.06.68.00.09.99.00.00.90.0d");  //запрос продукта
      if (last_init_command_ == 14  ) send_raw_cmd("55.0d.FF.FF.00.66.08.06.68.00.0a.99.00.00.93.0d");  //запрос железа
      if (last_init_command_ == 20  ) send_raw_cmd("55.0d.FF.FF.00.66.08.06.68.00.0b.99.00.00.92.0d");  // запрос прошивки
     
         
     this->last_init_command_++;         
     }
     
     
     
  //   uint8_t data[3] = {READ, this->current_request_, 0x01};
//      this->send_command_(data, 3);
      this->last_update_ = millis(); 
  }
  
  /*if ((millis() - this->last_update_) > this->update_interval_) {
     uint8_t data[3] = {READ, this->current_request_, 0x01};
      this->send_command_(data, 3);
      this->last_update_ = millis(); */
	//   char crc1;
//  		char data[47];
    while (uart_rx_available(_uart) > 0) {
      uint8_t c = (uint8_t)uart_read_char(_uart);                // считываем байт
      this->handle_char_(c);                                     // отправляем байт на обработку
	} //while
  
} //loop


void NiceBusT4::handle_char_(uint8_t c) {
  this->rx_message_.push_back(c);                      // кидаем байт в конец полученного сообщения
  if (!this->validate_message_()) {                    // проверяем получившееся сообщение
    this->rx_message_.clear();                         // если проверка не прошла, то в сообщении мусор, нужно удалить
  }
}


bool NiceBusT4::validate_message_() {                    // проверка получившегося сообщения
  uint32_t at = this->rx_message_.size() - 1;       // номер последнего полученного байта
  uint8_t *data = &this->rx_message_[0];               // указатель на первый байт сообщения
  uint8_t new_byte = data[at];                      // последний полученный байт

  // Byte 0: HEADER1 (всегда 0x00)
  if (at == 0)
    return new_byte == 0x00;
  // Byte 1: HEADER2 (всегда 0x55)
  if (at == 1)
    return new_byte == 0x55;

  // Byte 2: packet_size - количество байт дальше + 1
  // Проверка не проводится
  
  if (at == 2)
    return true;
  uint8_t packet_size = data[2];
  uint8_t length = (packet_size + 3); // длина ожидаемого сообщения понятна


  // Byte 3: Серия (ряд) кому пакет
  // Проверка не проводится
//  uint8_t command = data[3];
  if (at == 3)
    return true;

  // Byte 4: Адрес кому пакет
  // Byte 5: Серия (ряд) от кого пакет
  // Byte 6: Адрес от кого пакет 
  // Byte 7: Тип сообшения CMD или INF
  // Byte 8: Количество байт дальше за вычетом двух байт CRC в конце.
  
  if (at <= 8)
    // Проверка не проводится
    return true;

  uint8_t crc1 = (data[3]^data[4]^data[5]^data[6]^data[7]^data[8]);

  // Byte 9: crc1 = XOR (Byte 3 : Byte 8) XOR шести предыдущих байт
  if (at == 9)
    if (data[9] != crc1) {
      ESP_LOGW(TAG, "Received invalid message checksum 1 %02X!=%02X", data[9], crc1);
      return false;
     }
  // Byte 10: 
  // ...
     
// ждем пока поступят все данные пакета
  if (at  < length)
    return true;

// считаем crc2
  uint8_t crc2 = data[10];
  for (uint8_t i = 11; i < length-1; i++) {
      crc2 = (crc2^data[i]);
    }
    
  if (data[length-1] != crc2 ) {
    ESP_LOGW(TAG, "Received invalid message checksum 2 %02X!=%02X", data[length-1], crc2);
    return false;
    }  

  // Byte Last: packet_size
//  if (at  ==  length) {
  if (data[length] != packet_size ) {
    ESP_LOGW(TAG, "Received invalid message size %02X!=%02X", data[length], packet_size);
    return false;
    }

 // Если сюда дошли - правильное сообщение получено и лежит в буфере rx_message_
 
// Удаляем 0x00 в начале сообщения
rx_message_.erase(rx_message_.begin());

// здесь что-то делаем с сообщением
 parse_status_packet(rx_message_);

// для вывода команды в лог
 std::string pretty_cmd = format_hex_pretty_uint8_t(rx_message_);                   
 ESP_LOGI(TAG,  "Ответ Nice: %S ", pretty_cmd.c_str() );
 
 // возвращаем false чтобы обнулить rx buffer
 return false;

}


// разбираем полученные пакеты
void NiceBusT4::parse_status_packet (const std::vector<uint8_t> &data) {

// для пакета с информацией об оборудовании
  if ((data[1] == 0x0E) && (data[6] == INF) && (data[9] == SETUP) && (data[10] == TYPE_M) && (data[11] == 0x19)) {  // узнаём пакет статуса по содержимому в определённых байтах
  //ESP_LOGD(TAG, "Тип привода: %#X ", data[14]);
    switch (data[14]) {
      case SLIDING:
        this->class_gate_ = SLIDING;
//        ESP_LOGD(TAG, "Тип ворот: Откатные %#X ", data[14]);
        break;
      case SECTIONAL:
        this->class_gate_ = SECTIONAL;
//        ESP_LOGD(TAG, "Тип ворот: Секционные %#X ", data[14]);
        break;
      case SWING:
        this->class_gate_ = SWING;
//        ESP_LOGD(TAG, "Тип ворот: Распашные %#X ", data[14]);
        break;
      case BARRIER:
        this->class_gate_ = BARRIER;
//        ESP_LOGD(TAG, "Тип ворот: Шлагбаум %#X ", data[14]);
        break;
      case UPANDOVER:
        this->class_gate_ = UPANDOVER;
//        ESP_LOGD(TAG, "Тип ворот: Подъемно-поворотные %#X ", data[14]);
        break;
    }  // switch
 //   dump_config();
  } //if

// RSP ответ (ReSPonce) на простой прием команды CMD, а не ее выполнение. Также докладывает о завершении операции.
  if ((data[1] == 0x0E) && (data[6] == CMD) && (data[9] == SETUP)&& (data[10] == CUR_MAN) && (data[12] == 0x19)) {  // узнаём пакет статуса по содержимому в определённых байтах
  ESP_LOGD(TAG, "Получен пакет RSP. cmd = %#x", data[11]);
  
  	switch (data[11]) {
	  case OPENING:
	    this->current_operation = COVER_OPERATION_OPENING;
		ESP_LOGD(TAG, "Статус: Открывается");
        break;
	  case CLOSING:
	    this->current_operation = COVER_OPERATION_CLOSING;
		ESP_LOGD(TAG, "Статус: Закрывается");
        break;
	  case OPENED:
	    this->position = COVER_OPEN;
		ESP_LOGD(TAG, "Статус: Открыто");
        break;
	  case CLOSED:
	    this->position = COVER_CLOSED;
		ESP_LOGD(TAG, "Статус: Закрыто");
        break;
	  case STOPPED:
	    this->current_operation = COVER_OPERATION_IDLE;
		ESP_LOGD(TAG, "Статус: Остановлено");
		break;

  	}  // switch
  	
  	this->publish_state();  // публикуем состояние
 	
  } //if


    



// STA = 0x40,   // статус в движении
  if ((data[1] == 0x0E) && (data[6] == CMD) && (data[9] == SETUP)&& (data[10] == STA) ) {  // узнаём пакет статуса по содержимому в определённых байтах
  uint16_t ipos = (data[12]<<8) + data[13];
  ESP_LOGD(TAG, "Текущий маневр: %#X Позиция: %#X %#X, ipos = %#x,", data[11], data[12], data[13], ipos);
  this->position = ipos/2100.0f;   // передаем позицию компоненту
  
  	switch (data[11]) {
	  case OPENING:
	    this->current_operation = COVER_OPERATION_OPENING;
		ESP_LOGD(TAG, "Статус: Открывается");
        break;
	  case CLOSING:
	    this->current_operation = COVER_OPERATION_CLOSING;
		ESP_LOGD(TAG, "Статус: Закрывается");
        break;
	  case OPENED:
	    this->position = COVER_OPEN;
	    this->current_operation = COVER_OPERATION_IDLE;
		ESP_LOGD(TAG, "Статус: Открыто");
        break;
	  case CLOSED:
	    this->position = COVER_CLOSED;
	    this->current_operation = COVER_OPERATION_IDLE;
		ESP_LOGD(TAG, "Статус: Закрыто");
        break;
	  case STOPPED:
	    this->current_operation = COVER_OPERATION_IDLE;
		ESP_LOGD(TAG, "Статус: Остановлено");
		break;

  	}  // switch
  	
  	this->publish_state();  // публикуем состояние
  	
  } //if





// пакет с данными
  if (data[1] == (data[12]+0xd)) { 
//    std::vector<char> data_mes (this->rx_message_.begin()+14,this->rx_message_.end()-2);
//    std::string str(data_mes.begin(), data_mes.end());
      std::string str(this->rx_message_.begin()+14,this->rx_message_.end()-2);
      
    ESP_LOGI(TAG,  "Пакет с данными: %S ", str.c_str() );
      
      
    if ((data[9] == 0x00) && (data[11] == 0x19)){
        
        switch (data[10]) {
	      case 0x08:
//       ESP_LOGCONFIG(TAG, "  Производитель: %S ", str.c_str());
          this->manufacturer_.assign(this->rx_message_.begin()+14,this->rx_message_.end()-2);
          break;
          case 0x09:
//	        ESP_LOGCONFIG(TAG, "  Продукт: %S ", str.c_str());
          this->product_.assign(this->rx_message_.begin()+14,this->rx_message_.end()-2);
          break;  
          case 0x0a:
//	        ESP_LOGCONFIG(TAG, "  Железо: %S ", str.c_str());
          this->hardware_.assign(this->rx_message_.begin()+14,this->rx_message_.end()-2);
          break;  
          case 0x0b:
//	        ESP_LOGCONFIG(TAG, "  Прошивка: %S ", str.c_str());
            this->firmware_.assign(this->rx_message_.begin()+14,this->rx_message_.end()-2);
          break;            
	    }  // switch
    } //if2 

  } //if	



	
} // function






/*
  void NiceBusT4::on_rs485_data(const std::vector<uint8_t> &data) {
  std::vector<uint8_t> frame(data.begin(), data.end() - 2);
  uint16_t crc = crc16(&frame[0], frame.size());
  if (((crc & 0xFF) == data.end()[-2]) && ((crc >> 8) == data.end()[-1])) {
  switch (data[3]) {
    case CONTROL:
      switch (data[4]) {
        case STOP:
          this->current_operation = COVER_OPERATION_IDLE;
          break;
        case OPEN:
          this->current_operation = COVER_OPERATION_OPENING;
          break;
        case CLOSE:
          this->current_operation = COVER_OPERATION_CLOSING;
          break;
        case SET_POSITION:
          if (data[5] > (uint8_t)(this->position * 100))
            this->current_operation = COVER_OPERATION_OPENING;
          else
            this->current_operation = COVER_OPERATION_CLOSING;
          break;
        default:
          ESP_LOGE(TAG, "Invalid control operation received");
          return;
      }
      break;
    case READ:
      switch (this->current_request_) {
        case GET_POSITION:
          this->position = clamp((float) data[5] / 100, 0.0f, 1.0f);
          this->current_request_ = GET_STATUS;
          break;
        case GET_STATUS:
          switch (data[5]) {
            case 0:
              this->current_operation = COVER_OPERATION_IDLE;
              break;
            case 1:
              this->current_operation = COVER_OPERATION_OPENING;
              break;
            case 2:
              this->current_operation = COVER_OPERATION_CLOSING;
              break;
            default:
              ESP_LOGE(TAG, "Invalid status operation received");
              return;
          }
          this->current_request_ = GET_POSITION;
          break;
        default:
          ESP_LOGE(TAG, "Invalid read operation received");
          return;
      }
      break;
    default:
      ESP_LOGE(TAG, "Invalid data type received");
      return;
  }
  if (this->current_operation != this->last_published_op_ || this->position != this->last_published_pos_) {
    this->publish_state(false);
    this->last_published_op_ = this->current_operation;
    this->last_published_pos_ = this->position;
  }
  } else {
  ESP_LOGE(TAG, "Incoming data CRC check failed");
  }
  }

*/
void NiceBusT4::send_command_(const uint8_t *data, uint8_t len) {                      // генерирует команду для отправки
  /*std::vector<uint8_t> frame = {START_CODE, *this->header_[1], *this->header_[2]};
    for (size_t i = 0; i < len; i++) {
    frame.push_back(data[i]);
    }
    uint16_t crc = crc16(&frame[0], frame.size());
    frame.push_back(crc >> 0);
    frame.push_back(crc >> 8);

    this->send(frame);*/
}

void NiceBusT4::dump_config() {    //  добавляем в  лог информацию о подключенном контроллере
  ESP_LOGCONFIG(TAG, "  Bus T4 Cover");
  /*ESP_LOGCONFIG(TAG, "  Address: 0x%02X%02X", *this->header_[1], *this->header_[2]);*/
  switch (this->class_gate_) {
      case SLIDING:
        ESP_LOGCONFIG(TAG, "  Тип: Откатные ворота");
        break;
      case SECTIONAL:
        ESP_LOGCONFIG(TAG, "  Тип: Секционные ворота");
        break;
      case SWING:
        ESP_LOGCONFIG(TAG, "  Тип: Распашные ворота");
        break;
      case BARRIER:
        ESP_LOGCONFIG(TAG, "  Тип: Шлагбаум");
        break;
      case UPANDOVER:
        ESP_LOGCONFIG(TAG, "  Тип: Подъёмно-поворотные ворота");
        break;
      default:
        ESP_LOGCONFIG(TAG, "  Тип: Неизвестные ворота, 0x%02X", this->class_gate_);
  } // switch
  
  
  std::string manuf_str(this->manufacturer_.begin(),this->manufacturer_.end());
  ESP_LOGCONFIG(TAG, "  Производитель: %S ", manuf_str.c_str());  
  
  std::string prod_str(this->product_.begin(),this->product_.end());
  ESP_LOGCONFIG(TAG, "  Контроллер: %S ", prod_str.c_str());  
  
  std::string hard_str(this->hardware_.begin(),this->hardware_.end());
  ESP_LOGCONFIG(TAG, "  Железо: %S ", hard_str.c_str());  
  
  std::string firm_str(this->firmware_.begin(),this->firmware_.end());
  ESP_LOGCONFIG(TAG, "  Прошивка: %S ", firm_str.c_str());  
  
  
}


void NiceBusT4::send_open() {             // функция для отладки компонента при написании

  std::string  to_hex = "55 0c 00 ff 00 0a 01 05 F1 01 82 01 64 E6 0c";
// 55.0D.FF.FF.00.88.08.06.86.04.00.99.00.00.9D.0D

  std::vector < char > v_cmd = raw_cmd_prepare (to_hex);
  send_array_cmd (&v_cmd[0], v_cmd.size());

}

void NiceBusT4::send_raw_cmd(std::string data) {

  std::vector < char > v_cmd = raw_cmd_prepare (data);
  send_array_cmd (&v_cmd[0], v_cmd.size());

}


//  Сюда нужно добавить проверку на неправильные данные от пользователя
std::vector<char> NiceBusT4::raw_cmd_prepare (std::string data) { // подготовка введенных пользователем данных для возможности отправки

//  data.erase(remove_if(data.begin(), data.end(), ::isspace), data.end()); //удаляем пробелы
  data.erase(remove_if(data.begin(), data.end(), [](const unsigned char ch){return (!(iswalnum(ch)) );}), data.end()); //удаляем всё кроме букв и цифр

  //assert (data.size () % 2 == 0); // проверяем чётность
  std::vector < char > frame;
  frame.resize(0); // обнуляем размер команды

  for (uint8_t i = 0; i < data.size (); i += 2 ) { // заполняем массив команды
    std::string sub_str(data, i, 2); // берём 2 байта из команды
    char hexstoi = (char)std::strtol(&sub_str[0], 0 , 16); // преобразуем в число
    frame.push_back(hexstoi);  // записываем число в элемент  строки  новой команды
  }


  return frame;

}


void NiceBusT4::send_array_cmd (std::vector<char> data) {          // отправляет break + подготовленную ранее в массиве команду
  return send_array_cmd(data.data(), data.size());
}
void NiceBusT4::send_array_cmd (const char *data, size_t len) {
  // отправка данных в uart

  char br_ch = 0x00;                                               // для break
  uart_flush(_uart);                                               // очищаем uart
  uart_set_baudrate(_uart, BAUD_BREAK);                            // занижаем бодрэйт
  uart_write(_uart, &br_ch, 1);                                    // отправляем ноль на низкой скорости, длиинный ноль
  //uart_write(_uart, (char *)&dummy, 1);
  uart_wait_tx_empty(_uart);                                       // ждём, пока отправка завершится. Здесь в библиотеке uart.h (esp8266 core 3.0.2) ошибка, ожидания недостаточно при дальнейшем uart_set_baudrate().
  delayMicroseconds(90);                                          // добавляем задержку к ожиданию, иначе скорость переключится раньше отправки. С задержкой 83us на d1-mini я получил идеальный сигнал, break = 520us
  uart_set_baudrate(_uart, BAUD_WORK);                             // возвращаем рабочий бодрэйт
  uart_write(_uart, &data[0], len);                                // отправляем основную посылку
  //uart_write(_uart, (char *)raw_cmd_buf, sizeof(raw_cmd_buf));
  uart_wait_tx_empty(_uart);                                       // ждем завершения отправки

  
  //std::string pretty_cmd = "00." + format_hex_pretty_(&data[0], len);                    // для вывода команды в лог
  std::string pretty_cmd = format_hex_pretty_(&data[0], len);                    // для вывода команды в лог
  ESP_LOGI(TAG,  "Отправлено: %S ", pretty_cmd.c_str() );

}


// работа со строками, взято из dev esphome/core/helpers.h, изменен тип на char
  char NiceBusT4::format_hex_pretty_char_(char v) { return v >= 10 ? 'A' + (v - 10) : '0' + v; }
  std::string NiceBusT4::format_hex_pretty_(const char *data, size_t length) {
  if (length == 0)
    return "";
  std::string ret;
  ret.resize(3 * length - 1);
  for (size_t i = 0; i < length; i++) {
    ret[3 * i] = format_hex_pretty_char_((data[i] & 0xF0) >> 4);
    ret[3 * i + 1] = format_hex_pretty_char_(data[i] & 0x0F);
    if (i != length - 1)
      ret[3 * i + 2] = '.';
  }
  if (length > 4)
    return ret + " (" + to_string(length) + ")";
  return ret;
  }
  std::string NiceBusT4::format_hex_pretty_(std::vector<char> data) { return format_hex_pretty_(data.data(), data.size()); }

char NiceBusT4::format_hex_pretty_char_uint8_t(uint8_t v) { return v >= 10 ? 'A' + (v - 10) : '0' + v; }
std::string NiceBusT4::format_hex_pretty_uint8_t(const uint8_t *data, size_t length) {
  if (length == 0)
    return "";
  std::string ret;
  ret.resize(3 * length - 1);
  for (size_t i = 0; i < length; i++) {
    ret[3 * i] = format_hex_pretty_char_uint8_t((data[i] & 0xF0) >> 4);
    ret[3 * i + 1] = format_hex_pretty_char_uint8_t(data[i] & 0x0F);
    if (i != length - 1)
      ret[3 * i + 2] = '.';
  }
  if (length > 4)
    return ret + " (" + to_string(length) + ")";
  return ret;
}




}  // namespace bus_t4
}  // namespace esphome
