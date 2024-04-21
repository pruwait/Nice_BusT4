#include "nice-bust4.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"  // для использования вспомогательных функция работ со строками






namespace esphome {
namespace bus_t4 {

static const char *TAG = "bus_t4.cover";

using namespace esphome::cover;




CoverTraits NiceBusT4::get_traits() {
  auto traits = CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  return traits;
}


/*
  дампы команд OVIEW

  SBS               55 0c 00 ff 00 66 01 05 9D 01 82 01 64 E6 0c
  STOP              55 0c 00 ff 00 66 01 05 9D 01 82 02 64 E5 0c
  OPEN              55 0c 00 ff 00 66 01 05 9D 01 82 03 00 80 0c
  CLOSE             55 0c 00 ff 00 66 01 05 9D 01 82 04 64 E3 0c
  PARENTAL OPEN 1   55 0c 00 ff 00 66 01 05 9D 01 82 05 64 E2 0c
  PARENTAL OPEN 2   55 0c 00 ff 00 66 01 05 9D 01 82 06 64 E1 0c



*/

void NiceBusT4::control(const CoverCall &call) {
  position_hook_type = IGNORE;
  if (call.get_stop()) {
    send_cmd(STOP);

  } else if (call.get_position().has_value()) {
    float newpos = *call.get_position();
    if (newpos != position) {
      if (newpos == COVER_OPEN) {
        if (current_operation != COVER_OPERATION_OPENING) send_cmd(OPEN);

      } else if (newpos == COVER_CLOSED) {
        if (current_operation != COVER_OPERATION_CLOSING) send_cmd(CLOSE);

      } else { // Произвольное положение
        position_hook_value = (_pos_opn - _pos_cls) * newpos + _pos_cls;
        ESP_LOGI(TAG, "Требуемое положение привода: %d", position_hook_value);
        if (position_hook_value > _pos_usl) {
          position_hook_type = STOP_UP;
          if (current_operation != COVER_OPERATION_OPENING) send_cmd(OPEN);
        } else {
          position_hook_type = STOP_DOWN;
          if (current_operation != COVER_OPERATION_CLOSING) send_cmd(CLOSE);
        }
      }
    }
  }
}

void NiceBusT4::setup() {


  _uart =  uart_init(_UART_NO, BAUD_WORK, SERIAL_8N1, SERIAL_FULL, TX_P, 256, false);
  // кто в сети?
//  this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
  

}

void NiceBusT4::loop() {

    if ((millis() - this->last_update_) > 10000) {    // каждые 10 секунд
// если привод не определился с первого раза, попробуем позже
        std::vector<uint8_t> unknown = {0x55, 0x55};
        if (this->init_ok == false) {
          this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
          this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, PRD, GET, 0x00)); //запрос продукта
        }
        
        else if (this->class_gate_ == 0x55) {
		init_device(this->addr_to[0], this->addr_to[1], 0x04);  
	//        this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
        //        this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, PRD, GET, 0x00)); //запрос продукта
	}
        else if (this->manufacturer_ == unknown)  {
                init_device(this->addr_to[0], this->addr_to[1], 0x04);  
        //        this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
        //        this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, PRD, GET, 0x00)); //запрос продукта
		
        }
        this->last_update_ = millis();
    }  // if  каждую минуту

	
  // разрешаем отправку каждые 100 ms
    uint32_t now = millis();
  if (now - this->last_uart_byte_ > 100) {
    this->ready_to_tx_ = true;
    this->last_uart_byte_ = now;
  } 


  while (uart_rx_available(_uart) > 0) {
    uint8_t c = (uint8_t)uart_read_char(_uart);                // считываем байт
    this->handle_char_(c);                                     // отправляем байт на обработку
    this->last_uart_byte_ = now;
  } //while

  if (this->ready_to_tx_) {   // если можно отправлять
    if (!this->tx_buffer_.empty()) {  // если есть что отправлять
      this->send_array_cmd(this->tx_buffer_.front()); // отправляем первую команду в очереди
      this->tx_buffer_.pop();
      this->ready_to_tx_ = false;
    }
  }

  // Опрос текущего положения привода
  if (!is_robus) {
  
  now = millis();
  if (init_ok && (current_operation != COVER_OPERATION_IDLE) && (now - last_position_time > POSITION_UPDATE_INTERVAL)) {
  	last_position_time = now;
    request_position();
  } 
  } // not robus

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
    return new_byte == START_CODE;

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

  uint8_t crc1 = (data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7] ^ data[8]);

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
  for (uint8_t i = 11; i < length - 1; i++) {
    crc2 = (crc2 ^ data[i]);
  }

  if (data[length - 1] != crc2 ) {
    ESP_LOGW(TAG, "Received invalid message checksum 2 %02X!=%02X", data[length - 1], crc2);
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

  // для вывода пакета в лог
  std::string pretty_cmd = format_hex_pretty(rx_message_);
  ESP_LOGI(TAG,  "Получен пакет: %S ", pretty_cmd.c_str() );

  // здесь что-то делаем с сообщением
  parse_status_packet(rx_message_);



  // возвращаем false чтобы обнулить rx buffer
  return false;

}


// разбираем полученные пакеты
void NiceBusT4::parse_status_packet (const std::vector<uint8_t> &data) {
  if ((data[1] == 0x0d) && (data[13] == 0xFD)) { // ошибка
    ESP_LOGE(TAG,  "Команда недоступна для этого устройства" );
  }

  if (((data[11] == GET - 0x80) || (data[11] == GET - 0x81)) && (data[13] == NOERR)) { // if evt
  //  ESP_LOGD(TAG, "Получен пакет EVT с данными. Последняя ячейка %d ", data[12]);
    std::vector<uint8_t> vec_data(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
    std::string str(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
    ESP_LOGI(TAG,  "Строка с данными: %S ", str.c_str() );
    std::string pretty_data = format_hex_pretty(vec_data);
    ESP_LOGI(TAG,  "Данные HEX %S ", pretty_data.c_str() );
    // получили пакет с данными EVT, начинаем разбирать

    if ((data[6] == INF) && (data[9] == FOR_CU)  && (data[11] == GET - 0x80) && (data[13] == NOERR)) { // интересуют завершенные ответы на запросы GET, пришедшие без ошибок от привода
      ESP_LOGI(TAG,  "Получен ответ на запрос %X ", data[10] );
      switch (data[10]) { // cmd_submnu
        case TYPE_M:
          //           ESP_LOGI(TAG,  "Тип привода %X",  data[14]);
          switch (data[14]) { //14
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
          }  // switch 14
          break; //  TYPE_M
        case INF_IO: // ответ на запрос положения концевика откатных ворот
          switch (data[16]) { //16
            case 0x00:
              ESP_LOGI(TAG, "  Концевик не сработал ");
              break; // 0x00
            case 0x01:
              ESP_LOGI(TAG, "  Концевик на закрытие ");
              this->position = COVER_CLOSED;
              break; //  0x01
            case 0x02:
              ESP_LOGI(TAG, "  Концевик на открытие ");
              this->position = COVER_OPEN;
              break; // 0x02

          }  // switch 16
          this->publish_state_if_changed();  // публикуем состояние

          break; //  INF_IO


        //положение максимального открытия энкодера, открытия, закрытия

        case MAX_OPN:
          if (is_walky) {
            this->_max_opn = data[15];
            this->_pos_opn = data[15];
          }
          else {  
            this->_max_opn = (data[14] << 8) + data[15];
          }
          ESP_LOGI(TAG, "Максимальное положение энкодера: %d", this->_max_opn);
          break;

        case POS_MIN:
          this->_pos_cls = (data[14] << 8) + data[15];
          ESP_LOGI(TAG, "Положение закрытых ворот: %d", this->_pos_cls);
          break;

        case POS_MAX:
          if (((data[14] << 8) + data[15])>0x00) { // если в ответе от привода есть данные о положении открытия
          this->_pos_opn = (data[14] << 8) + data[15];}
          ESP_LOGI(TAG, "Положение открытых ворот: %d", this->_pos_opn);
          break;

        case CUR_POS:
          if (is_walky)
            update_position(data[15]);
          else
            update_position((data[14] << 8) + data[15]);
          break;

        case INF_STATUS:
          switch (data[14]) {
            case OPENED:
              ESP_LOGI(TAG, "  Ворота открыты");
              this->current_operation = COVER_OPERATION_IDLE;
              this->position = COVER_OPEN;
              break;
            case CLOSED:
              ESP_LOGI(TAG, "  Ворота закрыты");
              this->current_operation = COVER_OPERATION_IDLE;
              this->position = COVER_CLOSED;
              break;
            case 0x01:
              ESP_LOGI(TAG, "  Ворота остановлены");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
            case 0x00:
              ESP_LOGI(TAG, "  Статус ворот неизвестен");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
             case 0x0b:
              ESP_LOGI(TAG, "  Поиск положений сделан");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
              case STA_OPENING:
              ESP_LOGI(TAG, "  Идёт открывание");
              this->current_operation = COVER_OPERATION_OPENING;
              break;
              case STA_CLOSING:
              ESP_LOGI(TAG, "  Идёт закрывание");
              this->current_operation = COVER_OPERATION_CLOSING;
              break;
          }  // switch
          this->publish_state_if_changed();  // публикуем состояние
          break;

          //      default: // cmd_mnu
        case AUTOCLS:
          this->autocls_flag = data[14];
	  ESP_LOGCONFIG(TAG, "  Автозакрытие - L1: %S ", autocls_flag ? "Да" : "Нет");	
          break;
          
        case PH_CLS_ON:
          this->photocls_flag = data[14];
          break;  
          
        case ALW_CLS_ON:
          this->alwayscls_flag = data[14];
          break;  
          
      } // switch cmd_submnu
    } // if завершенные ответы на запросы GET, пришедшие без ошибок от привода

     if ((data[6] == INF) &&  (data[11] == GET - 0x81) && (data[13] == NOERR)) { // интересуют незавершенные ответы на запросы GET, пришедшие без ошибок от всех
	ESP_LOGI(TAG,  "Получен незавершенный ответ на запрос %X, продолжение со смещением %X", data[10], data[12] );
	     // повторяем команду с новым смещением
	tx_buffer_.push(gen_inf_cmd(data[4], data[5], data[9], data[10], GET, data[12]));
     
     } // незавершенные ответы на запросы GET, пришедшие без ошибок от привода

	  
    
    if ((data[6] == INF) && (data[9] == FOR_CU)  && (data[11] == SET - 0x80) && (data[13] == NOERR)) { // интересуют ответы на запросы SET, пришедшие без ошибок от привода    
      switch (data[10]) { // cmd_submnu
        case AUTOCLS:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, AUTOCLS, GET)); // Автозакрытие
          break;
          
        case PH_CLS_ON:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, PH_CLS_ON, GET)); // Закрыть после Фото
          break;  
          
        case ALW_CLS_ON:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, ALW_CLS_ON, GET)); // Всегда закрывать
          break;  
      }// switch cmd_submnu
    }// if ответы на запросы SET, пришедшие без ошибок от привода

    if ((data[6] == INF) && (data[9] == FOR_ALL)  && ((data[11] == GET - 0x80) || (data[11] == GET - 0x81)) && (data[13] == NOERR)) { // интересуют FOR_ALL ответы на запросы GET, пришедшие без ошибок

      switch (data[10]) {
        case MAN:
          //       ESP_LOGCONFIG(TAG, "  Производитель: %S ", str.c_str());
          this->manufacturer_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          break;
        case PRD:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) { // если пакет от приемника
//            ESP_LOGCONFIG(TAG, "  Приёмник: %S ", str.c_str());
            this->oxi_product.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } // если пакет от приемника
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) { // если пакет от контроллера привода
//            ESP_LOGCONFIG(TAG, "  Привод: %S ", str.c_str());
            this->product_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
            std::vector<uint8_t> wla1 = {0x57,0x4C,0x41,0x31,0x00,0x06,0x57}; // для понимания, что привод Walky
            std::vector<uint8_t> ROBUSHSR10 = {0x52,0x4F,0x42,0x55,0x53,0x48,0x53,0x52,0x31,0x30,0x00}; // для понимания, что привод ROBUSHSR10
            if (this->product_ == wla1) { 
              this->is_walky = true;
         //     ESP_LOGCONFIG(TAG, "  Привод WALKY!: %S ", str.c_str());
                                        }
            if (this->product_ == ROBUSHSR10) { 
              this->is_robus = true;
          //    ESP_LOGCONFIG(TAG, "  Привод ROBUS!: %S ", str.c_str());
                                        }		  
		  
          }
          break;
        case HWR:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) { // если пакет от приемника
            this->oxi_hardware.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) { // если пакет от контроллера привода          
          this->hardware_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } //else
          break;
        case FRM:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) { // если пакет от приемника
            this->oxi_firmware.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) { // если пакет от контроллера привода          
            this->firmware_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } //else
          break;
        case DSC:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) { // если пакет от приемника
            this->oxi_description.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) { // если пакет от контроллера привода          
            this->description_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } //else
          break;
        case WHO:
          if (data[12] == 0x01) {
            if (data[14] == 0x04) { // привод
              this->addr_to[0] = data[4];
              this->addr_to[1] = data[5];
              this->init_ok = true;
     //         init_device(data[4], data[5], data[14]);
            }
            else if (data[14] == 0x0A) { // приёмник
              this->addr_oxi[0] = data[4];
              this->addr_oxi[1] = data[5];
              init_device(data[4], data[5], data[14]);
            }
          }
          break;
      }  // switch

    }  // if  FOR_ALL ответы на запросы GET, пришедшие без ошибок

    if ((data[9] == 0x0A) &&  (data[10] == 0x25) &&  (data[11] == 0x01) &&  (data[12] == 0x0A) &&  (data[13] == NOERR)) { //  пакеты от приемника с информацией о списке пультов, пришедшие без ошибок
      ESP_LOGCONFIG(TAG, "Номер пульта: %X%X%X%X, команда: %X, кнопка: %X, режим: %X, счётчик нажатий: %d", vec_data[5], vec_data[4], vec_data[3], vec_data[2], vec_data[8] / 0x10, vec_data[5] / 0x10, vec_data[7] + 0x01, vec_data[6]);
    }  // if

    if ((data[9] == 0x0A) &&  (data[10] == 0x26) &&  (data[11] == 0x41) &&  (data[12] == 0x08) &&  (data[13] == NOERR)) { //  пакеты от приемника с информацией о считанной кнопке пульта
      ESP_LOGCONFIG(TAG, "Кнопка %X, номер пульта: %X%X%X%X", vec_data[0] / 0x10, vec_data[0] % 0x10, vec_data[1], vec_data[2], vec_data[3]);
    }  // if

  } //  if evt



  //else if ((data[14] == NOERR) && (data[1] > 0x0d)) {  // иначе пакет Responce - подтверждение полученной команды
  else if (data[1] > 0x0d) {  // иначе пакет Responce - подтверждение полученной команды
    ESP_LOGD(TAG, "Получен пакет RSP");
    std::vector<uint8_t> vec_data(this->rx_message_.begin() + 12, this->rx_message_.end() - 3);
    std::string str(this->rx_message_.begin() + 12, this->rx_message_.end() - 3);
    ESP_LOGI(TAG,  "Строка с данными: %S ", str.c_str() );
    std::string pretty_data = format_hex_pretty(vec_data);
    ESP_LOGI(TAG,  "Данные HEX %S ", pretty_data.c_str() );
    switch (data[9]) { // cmd_mnu
      case FOR_CU:
        ESP_LOGI(TAG, "Пакет контроллера привода");
        switch (data[10] + 0x80) { // sub_inf_cmd
          case RUN:
            ESP_LOGI(TAG, "Подменю RUN");
			if (data[11] >= 0x80) {
			  switch (data[11] - 0x80) {  // sub_run_cmd1
			    case SBS:
			      ESP_LOGI(TAG, "Команда: Пошагово");
			      break;
			    case STOP:
			      ESP_LOGI(TAG, "Команда: STOP");
			      break;
			    case OPEN:
			      ESP_LOGI(TAG, "Команда: OPEN");
			      this->current_operation = COVER_OPERATION_OPENING;
			      break;
			    case CLOSE:
			      ESP_LOGI(TAG, "Команда: CLOSE");
			      this->current_operation = COVER_OPERATION_CLOSING;
			      break;
			    case P_OPN1:
			      ESP_LOGI(TAG, "Команда: Частичное открывание 1");
			      break;
			    case STOPPED:
			      ESP_LOGI(TAG, "Команда: Остановлено");
			      this->current_operation = COVER_OPERATION_IDLE;
			      request_position();
			      break;
			    case ENDTIME:
			      ESP_LOGI(TAG, "Операция завершена по таймауту");
			      this->current_operation = COVER_OPERATION_IDLE;
			      request_position();
			      break;
			    default:
			      ESP_LOGI(TAG, "Неизвестная команда: %X", data[11]);
			  }  // switch sub_run_cmd1
			} else {
			  switch (data[11]) {  // sub_run_cmd2
			    case STA_OPENING:
			      ESP_LOGI(TAG, "Операция: Открывается");
			      this->current_operation = COVER_OPERATION_OPENING;
			      break;
			    case STA_CLOSING:
			      ESP_LOGI(TAG, "Операция: Закрывается");
			      this->current_operation = COVER_OPERATION_CLOSING;
			      break;
			    case CLOSED:
			      ESP_LOGI(TAG, "Операция: Закрыто");
			      this->current_operation = COVER_OPERATION_IDLE;
			      this->position = COVER_CLOSED;
			      break;
			    case OPENED:
			      ESP_LOGI(TAG, "Операция: Открыто");
			      this->current_operation = COVER_OPERATION_IDLE;
			      this->position = COVER_OPEN;
			      // calibrate opened position if the motor does not report max supported position (Road 400)
                  if (this->_max_opn == 0) {
                    this->_max_opn = this->_pos_opn = this->_pos_usl;
                    ESP_LOGI(TAG, "Opened position calibrated");
                  }
			      break;
			    case STOPPED:
			      ESP_LOGI(TAG, "Операция: Остановлено");
			      this->current_operation = COVER_OPERATION_IDLE;
			      request_position();
			      break;
			    case PART_OPENED:
			      ESP_LOGI(TAG, "Операция: Частично открыто");
			      this->current_operation = COVER_OPERATION_IDLE;
			      request_position();
			      break;
			    default:
			      ESP_LOGI(TAG, "Неизвестная операция: %X", data[11]);
			  }  // switch sub_run_cmd2
			}
			this->publish_state_if_changed();  // публикуем состояние
            break; //RUN

          case STA:
            ESP_LOGI(TAG,  "Подменю Статус в движении" );
            switch (data[11]) { // sub_run_cmd2
              case STA_OPENING:
              case 0x83: // Road 400
                ESP_LOGI(TAG, "Движение: Открывается" );
                this->current_operation = COVER_OPERATION_OPENING;
                break;
              case STA_CLOSING:
              case 0x84: // Road 400
                ESP_LOGI(TAG,  "Движение: Закрывается" );
                this->current_operation = COVER_OPERATION_CLOSING;
                break;
              case CLOSED:
                ESP_LOGI(TAG,  "Движение: Закрыто" );
                this->current_operation = COVER_OPERATION_IDLE;
                this->position = COVER_CLOSED;
                break;
              case OPENED:
                ESP_LOGI(TAG, "Движение: Открыто");
                this->current_operation = COVER_OPERATION_IDLE;
                this->position = COVER_OPEN;
                break;
              case STOPPED:
                ESP_LOGI(TAG, "Движение: Остановлено");
                this->current_operation = COVER_OPERATION_IDLE;
                request_position();
                break;
              default: // sub_run_cmd2
                ESP_LOGI(TAG,  "Движение: %X", data[11] );

                
            } // switch sub_run_cmd2

            update_position((data[12] << 8) + data[13]);
            break; //STA

          default: // sub_inf_cmd
            ESP_LOGI(TAG,  "Подменю %X", data[10] );
        }  // switch sub_inf_cmd

        break; // Пакет контроллера привода
      case CONTROL:
        ESP_LOGI(TAG,  "Пакет CONTROL" );
        break; // CONTROL
      case FOR_ALL:
        ESP_LOGI(TAG,  "Пакет для всех" );
        break; // FOR_ALL
      case 0x0A:
        ESP_LOGI(TAG,  "Пакет приёмника" );
        break; // пакет приёмника
      default: // cmd_mnu
        ESP_LOGI(TAG,  "Меню %X", data[9] );
    }  // switch  cmd_mnu


  } // else


  ///////////////////////////////////////////////////////////////////////////////////


  // RSP ответ (ReSPonce) на простой прием команды CMD, а не ее выполнение. Также докладывает о завершении операции.
  /* if ((data[1] == 0x0E) && (data[6] == CMD) && (data[9] == FOR_CU) && (data[10] == CUR_MAN) && (data[12] == 0x19)) { // узнаём пакет статуса по содержимому в определённых байтах
     //  ESP_LOGD(TAG, "Получен пакет RSP. cmd = %#x", data[11]);

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
         this->current_operation = COVER_OPERATION_IDLE;
         break;


       case CLOSED:
         this->position = COVER_CLOSED;
         ESP_LOGD(TAG, "Статус: Закрыто");
         this->current_operation = COVER_OPERATION_IDLE;
         break;
       case STOPPED:
         this->current_operation = COVER_OPERATION_IDLE;
         ESP_LOGD(TAG, "Статус: Остановлено");
         break;

     }  // switch

     this->publish_state();  // публикуем состояние

    } //if
  */
  /*
    // статус после достижения концевиков
    if ((data[1] == 0x0E) && (data[6] == CMD) && (data[9] == FOR_CU) && (data[10] == CUR_MAN) &&  (data[12] == 0x00)) { // узнаём пакет статуса по содержимому в определённых байтах
      ESP_LOGD(TAG, "Получен пакет концевиков. Статус = %#x", data[11]);
      switch (data[11]) {
        case OPENED:
          this->position = COVER_OPEN;
          ESP_LOGD(TAG, "Статус: Открыто");
          this->current_operation = COVER_OPERATION_IDLE;
          break;
        case CLOSED:
          this->position = COVER_CLOSED;
          ESP_LOGD(TAG, "Статус: Закрыто");
          this->current_operation = COVER_OPERATION_IDLE;
          break;
        case OPENING:
          this->current_operation = COVER_OPERATION_OPENING;
          ESP_LOGD(TAG, "Статус: Открывается");
          break;
        case CLOSING:
          this->current_operation = COVER_OPERATION_CLOSING;
          ESP_LOGD(TAG, "Статус: Закрывается");
          break;
      } //switch
      this->publish_state();  // публикуем состояние
    } //if
  */
  // STA = 0x40,   // статус в движении
  /*
    if ((data[1] == 0x0E) && (data[6] == CMD) && (data[9] == FOR_CU) && (data[10] == STA) ) { // узнаём пакет статуса по содержимому в определённых байтах
      uint16_t ipos = (data[12] << 8) + data[13];
      ESP_LOGD(TAG, "Текущий маневр: %#X Позиция: %#X %#X, ipos = %#x,", data[11], data[12], data[13], ipos);
      this->position = ipos / 2100.0f; // передаем позицию компоненту

      switch (data[11]) {
        case OPENING:
          this->current_operation = COVER_OPERATION_OPENING;
          ESP_LOGD(TAG, "Статус: Открывается");
          break;

        case OPENING2:
          this->current_operation = COVER_OPERATION_OPENING;
          ESP_LOGD(TAG, "Статус: Открывается");
          break;

        case CLOSING:
          this->current_operation = COVER_OPERATION_CLOSING;
          ESP_LOGD(TAG, "Статус: Закрывается");
          break;
        case CLOSING2:
          this->current_operation = COVER_OPERATION_CLOSING;
          ESP_LOGD(TAG, "Статус: Закрывается");
          break;
        case OPENED:
          this->position = COVER_OPEN;
          this->current_operation = COVER_OPERATION_IDLE;
          ESP_LOGD(TAG, "Статус: Открыто");
          //      this->current_operation = COVER_OPERATION_OPENING;
          //    ESP_LOGD(TAG, "Статус: Открывается");
          break;
        case CLOSED:
          this->position = COVER_CLOSED;
          this->current_operation = COVER_OPERATION_IDLE;
          ESP_LOGD(TAG, "Статус: Закрыто");
          //      this->current_operation = COVER_OPERATION_CLOSING;
          //ESP_LOGD(TAG, "Статус: Закрывается");
          break;
        case STOPPED:
          this->current_operation = COVER_OPERATION_IDLE;
          ESP_LOGD(TAG, "Статус: Остановлено");
          break;

      }  // switch

      this->publish_state();  // публикуем состояние

    } //if
  */


  ////////////////////////////////////////////////////////////////////////////////////////
} // function







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


  ESP_LOGCONFIG(TAG, "  Максимальное положение энкодера или таймера: %d", this->_max_opn);
  ESP_LOGCONFIG(TAG, "  Положение отрытых ворот: %d", this->_pos_opn);
  ESP_LOGCONFIG(TAG, "  Положение закрытых ворот: %d", this->_pos_cls);

  std::string manuf_str(this->manufacturer_.begin(), this->manufacturer_.end());
  ESP_LOGCONFIG(TAG, "  Производитель: %S ", manuf_str.c_str());

  std::string prod_str(this->product_.begin(), this->product_.end());
  ESP_LOGCONFIG(TAG, "  Привод: %S ", prod_str.c_str());

  std::string hard_str(this->hardware_.begin(), this->hardware_.end());
  ESP_LOGCONFIG(TAG, "  Железо привода: %S ", hard_str.c_str());

  std::string firm_str(this->firmware_.begin(), this->firmware_.end());
  ESP_LOGCONFIG(TAG, "  Прошивка привода: %S ", firm_str.c_str());
  
  std::string dsc_str(this->description_.begin(), this->description_.end());
  ESP_LOGCONFIG(TAG, "  Описание привода: %S ", dsc_str.c_str());


  ESP_LOGCONFIG(TAG, "  Адрес шлюза: 0x%02X%02X", addr_from[0], addr_from[1]);
  ESP_LOGCONFIG(TAG, "  Адрес привода: 0x%02X%02X", addr_to[0], addr_to[1]);
  ESP_LOGCONFIG(TAG, "  Адрес приёмника: 0x%02X%02X", addr_oxi[0], addr_oxi[1]);
  
  std::string oxi_prod_str(this->oxi_product.begin(), this->oxi_product.end());
  ESP_LOGCONFIG(TAG, "  Приёмник: %S ", oxi_prod_str.c_str());
  
  std::string oxi_hard_str(this->oxi_hardware.begin(), this->oxi_hardware.end());
  ESP_LOGCONFIG(TAG, "  Железо приёмника: %S ", oxi_hard_str.c_str());

  std::string oxi_firm_str(this->oxi_firmware.begin(), this->oxi_firmware.end());
  ESP_LOGCONFIG(TAG, "  Прошивка приёмника: %S ", oxi_firm_str.c_str());
  
  std::string oxi_dsc_str(this->oxi_description.begin(), this->oxi_description.end());
  ESP_LOGCONFIG(TAG, "  Описание приёмника: %S ", oxi_dsc_str.c_str());
 
  ESP_LOGCONFIG(TAG, "  Автозакрытие - L1: %S ", autocls_flag ? "Да" : "Нет");
  ESP_LOGCONFIG(TAG, "  Закрыть после фото - L2: %S ", photocls_flag ? "Да" : "Нет");
  ESP_LOGCONFIG(TAG, "  Всегда закрывать - L3: %S ", alwayscls_flag ? "Да" : "Нет");
  
}




//формирование команды управления
std::vector<uint8_t> NiceBusT4::gen_control_cmd(const uint8_t control_cmd) {
  std::vector<uint8_t> frame = {this->addr_to[0], this->addr_to[1], this->addr_from[0], this->addr_from[1]}; // заголовок
  frame.push_back(CMD);  // 0x01
  frame.push_back(0x05);
  uint8_t crc1 = (frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]);
  frame.push_back(crc1);
  frame.push_back(CONTROL);
  frame.push_back(RUN);
  frame.push_back(control_cmd);
  frame.push_back(0x64); // OFFSET CMD, DPRO924 отказался работать с 0x00, хотя остальные приводы реагировали на команды
  uint8_t crc2 = (frame[7] ^ frame[8] ^ frame[9] ^ frame[10]);
  frame.push_back(crc2);
  uint8_t f_size = frame.size();
  frame.push_back(f_size);
  frame.insert(frame.begin(), f_size);
  frame.insert(frame.begin(), START_CODE);

  // для вывода команды в лог
  //  std::string pretty_cmd = format_hex_pretty(frame);
  //  ESP_LOGI(TAG,  "Сформирована команда: %S ", pretty_cmd.c_str() );

  return frame;
}

// формирование команды INF с данными и без
std::vector<uint8_t> NiceBusT4::gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, const std::vector<uint8_t> &data, size_t len) {
  std::vector<uint8_t> frame = {to_addr1, to_addr2, this->addr_from[0], this->addr_from[1]}; // заголовок
  frame.push_back(INF);  // 0x08 mes_type
  frame.push_back(0x06 + len); // mes_size
  uint8_t crc1 = (frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]);
  frame.push_back(crc1);
  frame.push_back(whose);
  frame.push_back(inf_cmd);
  frame.push_back(run_cmd);
  frame.push_back(next_data); // next_data
  frame.push_back(len);
  if (len > 0) {
    frame.insert(frame.end(), data.begin(), data.end()); // блок данных
  }
  uint8_t crc2 = frame[7];
  for (size_t i = 8; i < 12 + len; i++) {
    crc2 = crc2 ^ frame[i];
  }
  frame.push_back(crc2);
  uint8_t f_size = frame.size();
  frame.push_back(f_size);
  frame.insert(frame.begin(), f_size);
  frame.insert(frame.begin(), START_CODE);

  // для вывода команды в лог
  //  std::string pretty_cmd = format_hex_pretty(frame);
  //  ESP_LOGI(TAG,  "Сформирован INF пакет: %S ", pretty_cmd.c_str() );

  return frame;

}


void NiceBusT4::send_raw_cmd(std::string data) {

  std::vector < uint8_t > v_cmd = raw_cmd_prepare (data);
  send_array_cmd (&v_cmd[0], v_cmd.size());

}


//  Сюда нужно добавить проверку на неправильные данные от пользователя
std::vector<uint8_t> NiceBusT4::raw_cmd_prepare (std::string data) { // подготовка введенных пользователем данных для возможности отправки
// удаляем всё кроме шестнадцатеричных букв и цифр
data.erase(remove_if(data.begin(), data.end(), [](const unsigned char ch) {
    return (!(isxdigit(ch)) );
  }), data.end()); 

  //assert (data.size () % 2 == 0); // проверяем чётность
  std::vector < uint8_t > frame;
  frame.resize(0); // обнуляем размер команды

  for (uint8_t i = 0; i < data.size (); i += 2 ) { // заполняем массив команды
    std::string sub_str(data, i, 2); // берём 2 байта из команды
    char hexstoi = (char)std::strtol(&sub_str[0], 0 , 16); // преобразуем в число
    frame.push_back(hexstoi);  // записываем число в элемент  строки  новой команды
  }


  return frame;

}



void NiceBusT4::send_array_cmd (std::vector<uint8_t> data) {          // отправляет break + подготовленную ранее в массиве команду
  return send_array_cmd((const uint8_t *)data.data(), data.size());
}
void NiceBusT4::send_array_cmd (const uint8_t *data, size_t len) {
  // отправка данных в uart

  char br_ch = 0x00;                                               // для break
  uart_flush(_uart);                                               // очищаем uart
  uart_set_baudrate(_uart, BAUD_BREAK);                            // занижаем бодрэйт
  uart_write(_uart, &br_ch, 1);                                    // отправляем ноль на низкой скорости, длиинный ноль
  //uart_write(_uart, (char *)&dummy, 1);
  uart_wait_tx_empty(_uart);                                       // ждём, пока отправка завершится. Здесь в библиотеке uart.h (esp8266 core 3.0.2) ошибка, ожидания недостаточно при дальнейшем uart_set_baudrate().
  delayMicroseconds(90);                                          // добавляем задержку к ожиданию, иначе скорость переключится раньше отправки. С задержкой на d1-mini я получил идеальный сигнал, break = 520us
  uart_set_baudrate(_uart, BAUD_WORK);                             // возвращаем рабочий бодрэйт
  uart_write(_uart, (char *)&data[0], len);                                // отправляем основную посылку
  //uart_write(_uart, (char *)raw_cmd_buf, sizeof(raw_cmd_buf));
  uart_wait_tx_empty(_uart);                                       // ждем завершения отправки



  std::string pretty_cmd = format_hex_pretty((uint8_t*)&data[0], len);                    // для вывода команды в лог
  ESP_LOGI(TAG,  "Отправлено: %S ", pretty_cmd.c_str() );

}


// генерация и отправка inf команд из yaml конфигурации
void NiceBusT4::send_inf_cmd(std::string to_addr, std::string whose, std::string command, std::string type_command, std::string next_data, bool data_on, std::string data_command) {
  std::vector < uint8_t > v_to_addr = raw_cmd_prepare (to_addr);
  std::vector < uint8_t > v_whose = raw_cmd_prepare (whose);
  std::vector < uint8_t > v_command = NiceBusT4::raw_cmd_prepare (command);
  std::vector < uint8_t > v_type_command = raw_cmd_prepare (type_command);
  std::vector < uint8_t > v_next_data = raw_cmd_prepare (next_data);
  std::vector < uint8_t > v_data_command = raw_cmd_prepare (data_command);


  if (data_on) {
    tx_buffer_.push(gen_inf_cmd(v_to_addr[0], v_to_addr[1], v_whose[0], v_command[0], v_type_command[0], v_next_data[0], v_data_command, v_data_command.size()));
  } else {
    tx_buffer_.push(gen_inf_cmd(v_to_addr[0], v_to_addr[1], v_whose[0], v_command[0], v_type_command[0], v_next_data[0]));
  } // else
}

// генерация и отправка команд установки контроллеру привода из yaml конфигурации с минимальными параметрами
void NiceBusT4::set_mcu(std::string command, std::string data_command) {
    std::vector < uint8_t > v_command = raw_cmd_prepare (command);
    std::vector < uint8_t > v_data_command = raw_cmd_prepare (data_command);
    tx_buffer_.push(gen_inf_cmd(0x04, v_command[0], 0xa9, 0x00, v_data_command));
  }
  
// инициализация устройства
void NiceBusT4::init_device (const uint8_t addr1, const uint8_t addr2, const uint8_t device ) {
  if (device == FOR_CU) {
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, TYPE_M, GET, 0x00)); // запрос типа привода
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, MAN, GET, 0x00)); // запрос производителя
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, FRM, GET, 0x00)); //  запрос прошивки
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, PRD, GET, 0x00)); //запрос продукта
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, HWR, GET, 0x00)); //запрос железа
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, POS_MAX, GET, 0x00));   //запрос позиции открытия
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, POS_MIN, GET, 0x00)); // запрос позиции закрытия
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, DSC, GET, 0x00)); //запрос описания
    if (is_walky)  // запрос максимального значения для энкодера
      tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, MAX_OPN, GET, 0x00, {0x01}, 1));
    else
      tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, MAX_OPN, GET, 0x00));
    request_position();  // запрос текущего положения
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, INF_STATUS, GET, 0x00)); //Состояние ворот (Открыто/Закрыто/Остановлено)
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, AUTOCLS, GET, 0x00)); // Автозакрытие
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, PH_CLS_ON, GET, 0x00)); // Закрыть после Фото
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, ALW_CLS_ON, GET, 0x00)); // Всегда закрывать
  }
  if (device == FOR_OXI) {
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, PRD, GET, 0x00)); //запрос продукта
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, HWR, GET, 0x00)); //запрос железа    
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, FRM, GET, 0x00)); //  запрос прошивки    
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, DSC, GET, 0x00)); //запрос описания    
  }
  
}

// Запрос условного текущего положения привода
void NiceBusT4::request_position(void) {
  if (is_walky)
    tx_buffer_.push(gen_inf_cmd(this->addr_to[0], this->addr_to[1], FOR_CU, CUR_POS, GET, 0x00, {0x01}, 1));
  else
    tx_buffer_.push(gen_inf_cmd(FOR_CU, CUR_POS, GET));
}

// Обновление текущего положения привода
void NiceBusT4::update_position(uint16_t newpos) {
  last_position_time = millis();
  _pos_usl = newpos;
  position = (_pos_usl - _pos_cls) * 1.0f / (_pos_opn - _pos_cls);
  ESP_LOGI(TAG, "Условное положение ворот: %d, положение в %%: %.3f", newpos, position);
  if (position < CLOSED_POSITION_THRESHOLD) position = COVER_CLOSED;
  publish_state_if_changed();  // публикуем состояние
  
  if ((position_hook_type == STOP_UP && _pos_usl >= position_hook_value) || (position_hook_type == STOP_DOWN && _pos_usl <= position_hook_value)) {
  	ESP_LOGI(TAG, "Достигнуто требуемое положение. Останавливаем ворота");
  	send_cmd(STOP);
  	position_hook_type = IGNORE;
  }
}

// Публикация состояния ворот при изменении
void NiceBusT4::publish_state_if_changed(void) {
  if (current_operation == COVER_OPERATION_IDLE) position_hook_type = IGNORE;
  if (last_published_op != current_operation || last_published_pos != position) {
    publish_state();
    last_published_op = current_operation;
    last_published_pos = position;
  }
}

}  // namespace bus_t4
}  // namespace esphome
