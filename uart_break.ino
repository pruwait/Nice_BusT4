/*
Отправка UART Break на Wemos D1 mini
Arduino Core 3.0.2

Логический анализатор помог отточить форму сигнала
*/

unsigned long timing; 

#define _UART_NO UART0
#define TX_P 1
#define baud_work 19200
#define baud_break 9200
#define LIN_BREAK_BAUDRATE(BAUD) ((BAUD * 9) / 13)

// переменные для uart
    int _uart_nr;
    uart_t* _uart = nullptr;
    
    

    uint8_t master_tx_buf[] = {0x55, 0x00, 0x33};
    uint8_t break_tx_buf[] = {0x00};
    //uint8_t break_tx_buf[] = {0x00, 0x66, 0x77, 0x00};
    //master_tx_buf[0] = 0x55; // sync byte
    //master_tx_buf[1] = 0x22;

    //uint8_t dummy = 0;


void send_break() {
  
/* Serial on/off Break
 // Работает плохо. 
 // 1. В момент включения uart виден короткий лишний импульс
 // 2. Последний символ посылки низкого бодрэйта не успевает отправится до переключения, уходит в линию на высоком бодрэйте

  //Serial.end(); // HardwareSerial::begin consist end();
  //Serial.flush();
  Serial.begin(110);  // Starting low baudrate uart
  Serial.write(0x00); // send low baud packet
  Serial.write(0x66);
  
  Serial.end();       // close low baud uart
 
  //Serial.flush();
  Serial.begin(19200); // Starting high baudrate uart
  Serial.write(0x00);  // send high baud packet
  Serial.write(0x55);
*/


//uart.h

/* UART on/off Break
//Работает плохо. 
// 1. В момент включения uart виден короткий лишний импульс
// 2. Последний символ посылки низкого бодрэйта не успевает отправится до переключения, уходит в линию на высоком бодрэйте


 uart_flush(_uart);
 uart_uninit(_uart); 
 _uart =  uart_init(_UART_NO, baud_break, SERIAL_8N1, UART_TX_ONLY, TX_P, 2, false);
 uart_set_baudrate(_uart, baud_break);
  //uart_write(_uart, (char *)&dummy, 1);
 uart_write(_uart, (char *)break_tx_buf, sizeof(break_tx_buf));
 uart_wait_tx_empty(_uart);
  //delayMicroseconds(100);
 uart_uninit(_uart); 
 _uart =  uart_init(_UART_NO, baud_work, SERIAL_8N1, SERIAL_FULL, TX_P, 2, false);
 uart_write(_uart, (char *)master_tx_buf, sizeof(master_tx_buf));
 uart_wait_tx_empty(_uart);
 */



// UART set_baudrate Break
// Работает хорошо с wemos d1 mini и Arduino Core 3.0.2

// Отправка break + посылка

 uart_flush(_uart);                                               // очищаем uart
 uart_set_baudrate(_uart, baud_break);                            // занижаем бодрэйт
 //uart_write(_uart, (char *)&dummy, 1);        
 uart_write(_uart, (char *)break_tx_buf, sizeof(break_tx_buf));   // отправляем ноль на низкой скорости, длиинный ноль
 uart_wait_tx_empty(_uart);                                       // ждём, пока отправка завершится. Здесь в библиотеке uart.h (esp8266 core 3.0.2) ошибка, ожидания недостаточно при дальнейшем uart_set_baudrate().
 delayMicroseconds(90);                                          // добавляем ожидания, иначе скорость переключится раньше отправки. С задержкой на d1-mini я получил идеальный сигнал, break = 520us? отправлял 0x00
 uart_set_baudrate(_uart, baud_work);                             // возвращаем рабочий бодрэйт
 uart_write(_uart, (char *)master_tx_buf, sizeof(master_tx_buf)); // отправляем основную посылку
 uart_wait_tx_empty(_uart);                                       // ждем завершения отправки

  
}

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
 _uart =  uart_init(_UART_NO, baud_work, SERIAL_8N1, SERIAL_FULL, TX_P, 256, false);
}


void loop() {


  if (millis() - timing > 2000) {
    timing = millis();
    bool on_led = digitalRead(LED_BUILTIN);
    digitalWrite(LED_BUILTIN, !on_led);
    send_break(); 
  }
}

