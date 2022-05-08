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
  if (call.get_stop()) {
    // uint8_t data[2] = {CONTROL, STOP};
    this->tx_buffer_.push(gen_control_cmd(STOP));
    this->tx_buffer_.push(gen_inf_cmd(FOR_CU, INF_STATUS, GET));   //Состояние ворот (Открыто/Закрыто/Остановлено)
    this->tx_buffer_.push(gen_inf_cmd(FOR_CU, CUR_POS, GET));    // запрос условного текущего положения привода



  } else if (call.get_position().has_value()) {
    auto pos = *call.get_position();
    if (pos != this->position) {
      if (pos == COVER_OPEN) {
        this->tx_buffer_.push(gen_control_cmd(OPEN));

      } else if (pos == COVER_CLOSED) {
        this->tx_buffer_.push(gen_control_cmd(CLOSE));

      } /*else {
          uint8_t data[3] = {CONTROL, SET_POSITION, (uint8_t)(pos * 100)};
          this->send_command_(data, 3);
        }*/
    }
  }
}

void NiceBusT4::setup() {
  delay (5000);   // пока привод не стартанёт, на команды отвечать не будет

  _uart =  uart_init(_UART_NO, BAUD_WORK, SERIAL_8N1, SERIAL_FULL, TX_P, 256, false);
  delay (500);
  //  this->last_init_command_ = 0;
  // кто в сети?
  this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));

}

void NiceBusT4::loop() {

  //  if ((millis() - this->last_update_) > this->update_interval_) {    // каждые 500ms


  //      this->last_update_ = millis();
  //  }  // if  каждые 500ms








  // разрешаем отправку каждые 100 ms
  const uint32_t now = millis();
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
  ESP_LOGI(TAG,  "Ответ Nice: %S ", pretty_cmd.c_str() );

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

  if ((data[1] == (data[12] + 0xd)) && (data[13] == NOERR)) { // if evt
    ESP_LOGD(TAG, "Получен пакет EVT с данными. Размер данных %d ", data[12]);
    std::vector<uint8_t> vec_data(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
    std::string str(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
    ESP_LOGI(TAG,  "Строка с данными: %S ", str.c_str() );
    std::string pretty_data = format_hex_pretty(vec_data);
    ESP_LOGI(TAG,  "Данные HEX %S ", pretty_data.c_str() );
    // получили пакет с данными EVT, начинаем разбирать

    if ((data[6] == INF) && (data[9] == FOR_CU)  && (data[11] == GET - 0x80) && (data[13] == NOERR)) { // интересуют ответы на запросы GET, пришедшие без ошибок от привода
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
          this->publish_state();  // публикуем состояние

          break; //  INF_IO


        //положение максимального открытия энкодера, открытия, закрытия

        case MAX_OPN:
          this->_max_opn = (data[14] << 8) + data[15];
          ESP_LOGI(TAG, "Максимальное положение энкодера: %d", this->_max_opn);
          break;

        case POS_MIN:
          this->_pos_cls = (data[14] << 8) + data[15];
          ESP_LOGI(TAG, "Положение закрытых ворот: %d", this->_pos_cls);
          break;

        case POS_MAX:
          this->_pos_opn = (data[14] << 8) + data[15];
          ESP_LOGI(TAG, "Положение открытых ворот: %d", this->_pos_opn);
          break;

        case CUR_POS:
          this->_pos_usl = (data[14] << 8) + data[15];
          this->position = (_pos_usl - _pos_cls) * 1.0f / (_pos_opn - _pos_cls);
          ESP_LOGI(TAG, "Условное положение ворот: %d, положение в %%: %f", _pos_usl, (_pos_usl - _pos_cls) * 100.0f / (_pos_opn - _pos_cls));
          this->publish_state();  // публикуем состояние
          break;

        case 0x01:
          switch (data[14]) {
            case OPENED:
              ESP_LOGI(TAG, "  Ворота открыты");
              this->position = COVER_OPEN;
              this->current_operation = COVER_OPERATION_IDLE;
              break;
            case CLOSED:
              ESP_LOGI(TAG, "  Ворота закрыты");
              this->position = COVER_CLOSED;
              this->current_operation = COVER_OPERATION_IDLE;
              break;
            case 0x01:
              ESP_LOGI(TAG, "  Ворота остановлены");
              this->current_operation = COVER_OPERATION_IDLE;
              //          this->position = COVER_OPEN;
              break;

          }  // switch
          this->publish_state();  // публикуем состояние
          break;

          //      default: // cmd_mnu
      } // switch cmd_submnu
    } // if ответы на запросы GET, пришедшие без ошибок от привода

    if ((data[6] == INF) && (data[9] == FOR_ALL)  && (data[11] == GET - 0x80) && (data[13] == NOERR)) { // интересуют FOR_ALL ответы на запросы GET, пришедшие без ошибок

      switch (data[10]) {
        case MAN:
          //       ESP_LOGCONFIG(TAG, "  Производитель: %S ", str.c_str());
          this->manufacturer_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          break;
        case PRD:
          if (((uint8_t)(this->oxi_addr >> 8) == data[4]) && ((uint8_t)(this->oxi_addr & 0xFF) == data[5])) { // если пакет от приемника
            ESP_LOGCONFIG(TAG, "  Приёмник: %S ", str.c_str());
            this->oxi_product.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } // если пакет от приемника
          else if (((uint8_t)(this->to_addr >> 8) == data[4]) && ((uint8_t)(this->to_addr & 0xFF) == data[5])) { // если пакет от контроллера привода
            ESP_LOGCONFIG(TAG, "  Привод: %S ", str.c_str());
            this->product_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          break;
        case HWR:
          //          ESP_LOGCONFIG(TAG, "  Железо: %S ", str.c_str());
          this->hardware_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          break;
        case FRM:
          //          ESP_LOGCONFIG(TAG, "  Прошивка: %S ", str.c_str());
          this->firmware_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          break;
        case WHO:
          //          ESP_LOGCONFIG(TAG, "  Прошивка: %S ", str.c_str());
          if (data[12] == 0x01) {
            if (data[14] == 0x04) { // привод
              this-> to_addr = ((uint16_t)data[4] << 8) | data[5];
              init_device(data[4], data[5], data[14]);
            }
            else if (data[14] == 0x0A) { // приёмник
              this-> oxi_addr = ((uint16_t)data[4] << 8) | data[5];
              init_device(data[4], data[5], data[14]);
            }
          }

          //this->firmware_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
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



  else if ((data[14] == NOERR) && (data[1] > 0x0d)) {  // иначе пакет Responce - подтверждение полученной команды
    ESP_LOGD(TAG, "Получен пакет RSP");
    std::vector<uint8_t> vec_data(this->rx_message_.begin() + 12, this->rx_message_.end() - 3);
    std::string str(this->rx_message_.begin() + 12, this->rx_message_.end() - 3);
    ESP_LOGI(TAG,  "Строка с данными: %S ", str.c_str() );
    std::string pretty_data = format_hex_pretty(vec_data);
    ESP_LOGI(TAG,  "Данные HEX %S ", pretty_data.c_str() );
    switch (data[9]) { // cmd_mnu
      case FOR_CU:
        ESP_LOGI(TAG,  "Пакет контроллера привода" );
        switch (data[10] + 0x80) { // sub_inf_cmd
          case RUN:
            ESP_LOGI(TAG,  "Подменю RUN" );
            switch (data[11] - 0x80) { // sub_run_cmd1
              case SBS:
                ESP_LOGI(TAG,  "Команда: Пошагово" );
                break; // SBS
              case STOP:
                ESP_LOGI(TAG,  "Команда: STOP" );
                break; // STOP
              case OPEN:
                ESP_LOGI(TAG,  "Команда: OPEN" );
                this->current_operation = COVER_OPERATION_OPENING;
                break; // OPEN
              case CLOSE:
                ESP_LOGI(TAG,  "Команда: CLOSE" );
                this->current_operation = COVER_OPERATION_CLOSING;                
                break;  // CLOSE
              case P_OPN1:
                ESP_LOGI(TAG,  "Команда: Частичное открывание" );
                break; // P_OPN1
              case STOPPED:
                this->current_operation = COVER_OPERATION_IDLE;
                ESP_LOGI(TAG, "Команда: Остановлено");
                break; // STOPPED
              case ENDTIME:
                ESP_LOGI(TAG, "Операция завершена по таймауту");
                break; // 

            } // switch sub_run_cmd1
            
            switch (data[11]) { // sub_run_cmd2
              case STA_OPENING:
                ESP_LOGI(TAG,  "Операция: Открывается" );
                this->current_operation = COVER_OPERATION_OPENING;
                break; // OPEN
              case STA_CLOSING:
                ESP_LOGI(TAG,  "Операция: Закрывается" );
                this->current_operation = COVER_OPERATION_CLOSING;                
                break;  // CLOSING
              case CLOSED:
                ESP_LOGI(TAG,  "Операция: Закрыто" );
                this->position = COVER_CLOSED;
                this->current_operation = COVER_OPERATION_IDLE;
                break;  // CLOSED  
              case OPENED:
                this->position = COVER_OPEN;
                ESP_LOGI(TAG, "Операция: Открыто");
                this->current_operation = COVER_OPERATION_IDLE;
                break;
              case STOPPED:
                this->current_operation = COVER_OPERATION_IDLE;
                ESP_LOGI(TAG, "Операция: Остановлено");
                break;
              default: // sub_run_cmd1
                ESP_LOGI(TAG,  "Операция: %X", data[11] );                            
            } // switch sub_run_cmd2                 
            this->publish_state();  // публикуем состояние
            break; //RUN

          case STA:
            ESP_LOGI(TAG,  "Подменю Статус в движении" );
            switch (data[11]) { // sub_run_cmd2
              case STA_OPENING:
                ESP_LOGI(TAG,  "Движение: Открывается" );
                this->current_operation = COVER_OPERATION_OPENING;
                break; // STA_OPENING
              case STA_CLOSING:
                ESP_LOGI(TAG,  "Движение: Закрывается" );
                this->current_operation = COVER_OPERATION_CLOSING;
                break; // STA_CLOSING
              case CLOSED:
                ESP_LOGI(TAG,  "Движение: Закрыто" );
                this->position = COVER_CLOSED;
                this->current_operation = COVER_OPERATION_IDLE;
                break;  // CLOSED  
              case OPENED:
                this->position = COVER_OPEN;
                ESP_LOGI(TAG, "Движение: Открыто");
                this->current_operation = COVER_OPERATION_IDLE;
                break;
              case STOPPED:
                this->current_operation = COVER_OPERATION_IDLE;
                ESP_LOGI(TAG, "Движение: Остановлено");
                break;
              default: // sub_run_cmd2
                ESP_LOGI(TAG,  "Движение: %X", data[11] );

                
            } // switch sub_run_cmd2

            this->_pos_usl = (data[12] << 8) + data[13];
            this->position = (_pos_usl - _pos_cls) * 1.0f / (_pos_opn - _pos_cls);
            ESP_LOGD(TAG, "Условное положение ворот: %d, положение в %%: %f", _pos_usl, (_pos_usl - _pos_cls) * 100.0f / (_pos_opn - _pos_cls));
            this->publish_state();  // публикуем состояние

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
  ESP_LOGCONFIG(TAG, "  Железо: %S ", hard_str.c_str());

  std::string firm_str(this->firmware_.begin(), this->firmware_.end());
  ESP_LOGCONFIG(TAG, "  Прошивка: %S ", firm_str.c_str());

  ESP_LOGCONFIG(TAG, "  Адрес шлюза: 0x%04X", from_addr);
  ESP_LOGCONFIG(TAG, "  Адрес привода: 0x%04X", to_addr);
  ESP_LOGCONFIG(TAG, "  Адрес приёмника: 0x%04X", oxi_addr);
  
  std::string oxi_prod_str(this->oxi_product.begin(), this->oxi_product.end());
  ESP_LOGCONFIG(TAG, "  Приёмник: %S ", oxi_prod_str.c_str());
}




//формирование команды управления
std::vector<uint8_t> NiceBusT4::gen_control_cmd(const uint8_t control_cmd) {
  std::vector<uint8_t> frame = {(uint8_t)(this->to_addr >> 8), (uint8_t)(this->to_addr & 0xFF), (uint8_t)(this->from_addr >> 8), (uint8_t)(this->from_addr & 0xFF)}; // заголовок
  frame.push_back(CMD);  // 0x01
  frame.push_back(0x05);
  uint8_t crc1 = (frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]);
  frame.push_back(crc1);
  frame.push_back(CONTROL);
  frame.push_back(RUN);
  frame.push_back(control_cmd);
  frame.push_back(0x00);
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
  std::vector<uint8_t> frame = {to_addr1, to_addr2, (uint8_t)(this->from_addr >> 8), (uint8_t)(this->from_addr & 0xFF)}; // заголовок
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

  //  data.erase(remove_if(data.begin(), data.end(), ::isspace), data.end()); //удаляем пробелы
  data.erase(remove_if(data.begin(), data.end(), [](const unsigned char ch) {
    return (!(iswalnum(ch)) );
  }), data.end()); //удаляем всё кроме букв и цифр

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
  delayMicroseconds(90);                                          // добавляем задержку к ожиданию, иначе скорость переключится раньше отправки. С задержкой 83us на d1-mini я получил идеальный сигнал, break = 520us
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
  }


}

// инициализация устройства
void NiceBusT4::init_device (const uint8_t addr1, const uint8_t addr2, const uint8_t device ) {
  if (device == 0x04) {
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, TYPE_M, GET, 0x00)); // запрос типа привода
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, MAN, GET, 0x00)); // запрос производителя
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, FRM, GET, 0x00)); //  запрос прошивки
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, PRD, GET, 0x00)); //запрос продукта
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, HWR, GET, 0x00)); //запрос железа
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, POS_MAX, GET, 0x00));   //запрос позиции открытия
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, POS_MIN, GET, 0x00)); // запрос позиции закрытия
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, DSC, GET, 0x00)); //запрос описания
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, MAX_OPN, GET, 0x00));   // запрос максимального значения для энкодера
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, CUR_POS, GET, 0x00));  // запрос текущей позиции для энкодера
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, INF_STATUS, GET, 0x00)); //Состояние ворот (Открыто/Закрыто/Остановлено)
  }
  if (device == 0x0a) {
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, PRD, GET, 0x00)); //запрос продукта
  }
  
}






}  // namespace bus_t4
}  // namespace esphome
