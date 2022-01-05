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

void NiceBusT4::control(const CoverCall &call) {  
    if (call.get_stop()) {
     // uint8_t data[2] = {CONTROL, STOP};
	  std::string data = "55 0C 00 FF 00 0A 01 05 F1 0A 82 01 80 09 0C"; // пока здесь дамп sbs
	  std::vector < char > v_cmd = raw_cmd_prepare (data);
      this->send_array_cmd (&v_cmd[0], v_cmd.size());
      
    } else if (call.get_position().has_value()) {
      auto pos = *call.get_position();
      if (pos != this->position) {
        if (pos == COVER_OPEN) {
          std::string data = "55 0c 00 03 05 81 01 05 83 01 82 03 64 e4 0c"; // пока здесь дамп open
	      std::vector < char > v_cmd = raw_cmd_prepare (data);
          this->send_array_cmd (&v_cmd[0], v_cmd.size());
        } else if (pos == COVER_CLOSED) {
          std::string data = "55 0c 00 03 05 81 01 05 83 01 82 04 64 e3 0c"; // пока здесь дамп close
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
  _uart =  uart_init(_UART_NO, baud_work, SERIAL_8N1, SERIAL_FULL, TX_P, 256, false);
  ESP_LOGCONFIG(TAG, "Setting up Nice ESP BusT4...");
  /*  if (this->header_.empty()) {                                                             // заполняем адреса значениями по умолчанию, если они не указаны явно в конфигурации yaml
      this->header_ = {(uint8_t *)&START_CODE, (uint8_t *)&DEF_ADDR, (uint8_t *)&DEF_ADDR};
    }*/
}

void NiceBusT4::loop() {
  if ((millis() - this->last_update_) > this->update_interval_) {
    /* uint8_t data[3] = {READ, this->current_request_, 0x01};
      this->send_command_(data, 3);
      this->last_update_ = millis(); */
  }
}

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
  ESP_LOGCONFIG(TAG, "Nice Bus T4:");
  /*ESP_LOGCONFIG(TAG, "  Address: 0x%02X%02X", *this->header_[1], *this->header_[2]);*/
}


void NiceBusT4::send_open() {             // функция для отладки компонента при написании

  std::string  to_hex = "55 0C 00 FF 00 0A 01 05 F1 0A 82 01 80 09 0C";
  
  
  std::vector < char > v_cmd = raw_cmd_prepare (to_hex);
  send_array_cmd (&v_cmd[0], v_cmd.size());
 
}

void NiceBusT4::send_raw_cmd(std::string data) {             

   
  std::vector < char > v_cmd = raw_cmd_prepare (data);
  send_array_cmd (&v_cmd[0], v_cmd.size());
 
}


//  Сюда нужно добавить проверку на неправильные данные от пользователя
std::vector<char> NiceBusT4::raw_cmd_prepare (std::string data) { // подготовка введенных пользователем данных для возможности отправки
	/*std::vector<uint8_t> frame = {START_CODE, *this->header_[1], *this->header_[2]};
    for (size_t i = 0; i < len; i++) {
    frame.push_back(data[i]);
    }*/
	
	data.erase(remove_if(data.begin(), data.end(), ::isspace), data.end()); //удаляем пробелы
  //assert (data.size () % 2 == 0); // проверяем чётность
  std::vector < char > frame;
  frame.resize(0); // обнуляем размер команды

  for (uint8_t i = 0; i < data.size (); i += 2 ) { // заполняем массив команды
    std::string sub_str(data, i, 2); // берём 2 символа из команды
    char hexstoi = (char)std::strtol(&sub_str[0], 0 , 16); // преобразуем в число
    //char c = hexstoi;
    frame.push_back(hexstoi);  // записываем число в элемент  строки  новой команды
  }

 // ESP_LOGI(TAG,  "Команда: %s Длина %d ", data.c_str(), frame.size() );
  //  ESP_LOGI(TAG, "HEX команда %s",  str_v.c_str());

  return frame;
	
		
		
}


void NiceBusT4::send_array_cmd (std::vector<char> data) {          // отправляет break + подготоаленную ранее в массиве команду
  return send_array_cmd(data.data(), data.size());
}
void NiceBusT4::send_array_cmd (const char *data, size_t len) {
  // отправка данных в uart
  
  char br_ch = 0x00;                                               // для break
  uart_flush(_uart);                                               // очищаем uart
  uart_set_baudrate(_uart, baud_break);                            // занижаем бодрэйт
  uart_write(_uart, &br_ch, 1);                                    // отправляем ноль на низкой скорости, длиинный ноль
  //uart_write(_uart, (char *)&dummy, 1);
  uart_wait_tx_empty(_uart);                                       // ждём, пока отправка завершится. Здесь в библиотеке uart.h (esp8266 core 3.0.2) ошибка, ожидания недостаточно при дальнейшем uart_set_baudrate().
  delayMicroseconds(90);                                          // добавляем задержку к ожиданию, иначе скорость переключится раньше отправки. С задержкой 83us на d1-mini я получил идеальный сигнал, break = 520us
  uart_set_baudrate(_uart, baud_work);                             // возвращаем рабочий бодрэйт
  uart_write(_uart, &data[0], len);                      // отправляем основную посылку
  //uart_write(_uart, (char *)raw_cmd_buf, sizeof(raw_cmd_buf));
  uart_wait_tx_empty(_uart);                                       // ждем завершения отправки

  
  std::string pretty_cmd = format_hex_pretty(&data[0], len);                    // для вывода команды в лог
  ESP_LOGI(TAG,  "Отправлено: %S ", pretty_cmd.c_str() );

}


// работа со строками, взято из dev esphome/core/helpers.h, изменен тип на char
  char NiceBusT4::format_hex_pretty_char(char v) { return v >= 10 ? 'A' + (v - 10) : '0' + v; }
  std::string NiceBusT4::format_hex_pretty(const char *data, size_t length) {
  if (length == 0)
    return "";
  std::string ret;
  ret.resize(3 * length - 1);
  for (size_t i = 0; i < length; i++) {
    ret[3 * i] = format_hex_pretty_char((data[i] & 0xF0) >> 4);
    ret[3 * i + 1] = format_hex_pretty_char(data[i] & 0x0F);
    if (i != length - 1)
      ret[3 * i + 2] = '.';
  }
  if (length > 4)
    return ret + " (" + to_string(length) + ")";
  return ret;
  }
  std::string NiceBusT4::format_hex_pretty(std::vector<char> data) { return format_hex_pretty(data.data(), data.size()); }







/** Parse bytes from a hex-encoded string into a byte array.

   When \p len is less than \p 2*count, the result is written to the back of \p data (i.e. this function treats \p str
   as if it were padded with zeros at the front).

   @param str String to read from.
   @param len Length of \p str (excluding optional null-terminator), is a limit on the number of characters parsed.
   @param data Byte array to write to.
   @param count Length of \p data.
   @return The number of characters parsed from \p str.
*/
/*
  static size_t parse_hex(const char *str, size_t length, uint8_t *data, size_t count) {
  uint8_t val;
  size_t chars = std::min(length, 2 * count);
  for (size_t i = 2 * count - chars; i < 2 * count; i++, str++) {
    if (*str >= '0' && *str <= '9')
      val = *str - '0';
    else if (*str >= 'A' && *str <= 'F')
      val = 10 + (*str - 'A');
    else if (*str >= 'a' && *str <= 'f')
      val = 10 + (*str - 'a');
    else
      return 0;
    data[i >> 1] = !(i & 1) ? val << 4 : data[i >> 1] | val;
  }
  return chars;
  }
*/

// работа со строками


/** Parse bytes from a hex-encoded string into a byte array.

   When \p len is less than \p 2*count, the result is written to the back of \p data (i.e. this function treats \p str
   as if it were padded with zeros at the front).

   @param str String to read from.
   @param len Length of \p str (excluding optional null-terminator), is a limit on the number of characters parsed.
   @param data Byte array to write to.
   @param count Length of \p data.
   @return The number of characters parsed from \p str.
*/

/*

  static size_t parse_hex(const char *str, size_t len, uint8_t *data, size_t count);
  /// Parse \p count bytes from the hex-encoded string \p str of at least \p 2*count characters into array \p data.
    inline bool parse_hex(const char *str, uint8_t *data, size_t count) {
      return parse_hex(str, strlen(str), data, count) == 2 * count;
    }
  /// Parse \p count bytes from the hex-encoded string \p str of at least \p 2*count characters into array \p data.
    inline bool parse_hex(const std::string &str, uint8_t *data, size_t count) {
      return parse_hex(str.c_str(), str.length(), data, count) == 2 * count;
    }
  /// Parse \p count bytes from the hex-encoded string \p str of at least \p 2*count characters into vector \p data.
    inline bool parse_hex(const char *str, std::vector<uint8_t> &data, size_t count) {
       data.resize(count);
      return parse_hex(str, strlen(str), data.data(), count) == 2 * count;
    }
  /// Parse \p count bytes from the hex-encoded string \p str of at least \p 2*count characters into vector \p data.
    inline bool parse_hex(const std::string &str, std::vector<uint8_t> &data, size_t count) {
      data.resize(count);
      return parse_hex(str.c_str(), str.length(), data.data(), count) == 2 * count;
    }






  /// Format the byte array \p data of length \p len in lowercased hex.
  static std::string format_hex(const uint8_t *data, size_t length);
  /// Format the vector \p data in lowercased hex.
  static std::string format_hex(std::vector<uint8_t> data);
  /// Format an unsigned integer in lowercased hex, starting with the most significant byte.
  template<typename T, enable_if_t<std::is_unsigned<T>::value, int> = 0> std::string format_hex(T val) {
  val = convert_big_endian(val);
  return format_hex(reinterpret_cast<uint8_t *>(&val), sizeof(T));
  }




  /// Format the byte array \p data of length \p len in pretty-printed, human-readable hex.
  static std::string format_hex_pretty(const uint8_t *data, size_t length);
  /// Format the vector \p data in pretty-printed, human-readable hex.
  static std::string format_hex_pretty(std::vector<uint8_t> data);
  /// Format an unsigned integer in pretty-printed, human-readable hex, starting with the most significant byte.
  template<typename T, enable_if_t<std::is_unsigned<T>::value, int> = 0> std::string format_hex_pretty(T val) {
  val = convert_big_endian(val);
  return format_hex_pretty(reinterpret_cast<uint8_t *>(&val), sizeof(T));
  }






  static char format_hex_char(uint8_t v) { return v >= 10 ? 'a' + (v - 10) : '0' + v; }
  std::string format_hex(const uint8_t *data, size_t length) {
  std::string ret;
  ret.resize(length * 2);
  for (size_t i = 0; i < length; i++) {
    ret[2 * i] = format_hex_char((data[i] & 0xF0) >> 4);
    ret[2 * i + 1] = format_hex_char(data[i] & 0x0F);
  }
  return ret;
  }
  std::string format_hex(std::vector<uint8_t> data) { return format_hex(data.data(), data.size()); }

*/

}  // namespace bus_t4
}  // namespace esphome
