#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>
#include "Config.h"
#include "ROVController.h"

// Определения Serial портов
// Serial1 уже определен в фреймворке Arduino для STM32
// Определяем только дополнительные Serial порты
HardwareSerial Serial2(SERIAL2_RX, SERIAL2_TX);

// Основной контроллер ROV
ROVController rov_controller;

// ARDUINO FUNCTIONS

void setup() {
    
    // Инициализация Serial портов
    Serial1.setRx(SERIAL1_RX);
    Serial1.setTx(SERIAL1_TX);
    Serial1.begin(SERIAL_BAUD);
    
    Serial2.setRx(SERIAL2_RX);
    Serial2.setTx(SERIAL2_TX);
    Serial2.begin(DEBUG_BAUD);
    
    // Инициализация основного контроллера
    rov_controller.initialize();
    
    // Вывод сообщения об успешной инициализации
    Serial2.println("System initialized successfully");
}

void loop() {
    // Основной цикл обработки
    rov_controller.update();
}