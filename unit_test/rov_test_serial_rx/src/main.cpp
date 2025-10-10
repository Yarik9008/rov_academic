// подключаем библиотеки 
#include <Arduino.h>
#include <AsyncStream.h>
#include <Config.h>


AsyncStream<200> serial1(&Serial1, '\n');
// TODO подобрать параметры измерения вольтажа

void setup() {
  // подключение отладочного сериала 
  Serial.begin(115200);

  Serial1.setRX(UART_RX);
  Serial1.setTX(UART_TX);
  pinMode(UART_COM, OUTPUT);
  digitalWrite(UART_COM, LOW);
  Serial1.begin(115200);
}

void loop() {
  // если данные получены
  if (serial1.available()) {

    Serial.println(serial1.buf);
  }  
}
