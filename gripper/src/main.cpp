#include <Arduino.h>
#include "Config.h"
#include "PulseMeter.h"
#include "CurrentSensor.h"
#include "MotorDriver.h"

// Создание экземпляров
PulseMeter pulseMeter(PULSE_INPUT_PIN);
CurrentSensor currentSensor(I2C_SDA_PIN, I2C_SCL_PIN);
MotorDriver gripperMotor(MOTOR_IA_PIN, MOTOR_IB_PIN);

// Глобальные переменные

void setup() {
  // Инициализация компонентов
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial) delay(10); // Ждем готовности Serial порта
    
    pulseMeter.begin();
    currentSensor.begin();
    gripperMotor.begin();
    
    Serial.println("=== ROV Gripper System ===");
    Serial.println("Готов к работе...");
    Serial.println();
}

// Глобальные переменные для упрощения
static unsigned long lastUpdate = 0;
static int16_t motor_speed = 0;
static bool current_protection_active = false;
static int16_t protection_direction = 0; // Направление при срабатывании защиты
static unsigned long motor_start_time = 0;
static bool motor_was_running = false;
static bool motor_startup_delay_active = false; // Флаг активной задержки после старта


// Функция проверки защиты от перегрузки
void checkCurrentProtection() {
    if (!currentSensor.isInitialized()) return;
    
    unsigned long currentTime = millis();
    
    // Проверяем, работает ли двигатель
    uint8_t pin_a_value, pin_b_value;
    gripperMotor.getDiagnostics(pin_a_value, pin_b_value);
    
    if (pin_a_value > 0 || pin_b_value > 0) {
        // Двигатель работает
        if (!motor_was_running) {
            // Двигатель только что запустился
            motor_start_time = currentTime;
            motor_was_running = true;
            motor_startup_delay_active = true;
            Serial.println("Motor started, waiting " + String(MOTOR_START_DELAY_MS) + "ms for startup...");
        }
        
        // Проверяем ток только после завершения задержки старта
        if (motor_startup_delay_active && currentTime - motor_start_time >= MOTOR_START_DELAY_MS) {
            motor_startup_delay_active = false;
            Serial.println("Startup delay completed, current protection active");
        }
        
        // Измеряем ток только если задержка старта завершена
        if (!motor_startup_delay_active) {
            float current_mA = currentSensor.getCurrent_mA();
            
            // Защита при превышении абсолютного значения тока
            if (current_mA >= CURRENT_PROTECTION_THRESHOLD_MA) {
                if (!current_protection_active) {
                    current_protection_active = true;
                    protection_direction = motor_speed; // Запоминаем направление при срабатывании
                    Serial.println("ЗАЩИТА! Ток: " + String(current_mA, 1) + "mA, направление: " + 
                                 (motor_speed > 0 ? "ВПЕРЕД" : "НАЗАД"));
                }
                gripperMotor.stop();
            }
        }
    } else {
        // Двигатель остановлен
        motor_was_running = false;
        motor_startup_delay_active = false;
    }
}

// Функция обработки PWM сигнала
void processPWMControl() {
    uint32_t current_pulse_width = pulseMeter.getPulseWidthAndClear();
    
    
    int16_t new_speed = MOTOR_SPEED_STOP;
    
    if (current_pulse_width >= PWM_MIN_US && current_pulse_width <= PWM_MAX_US) {
        if (current_pulse_width < PWM_DEADZONE_MIN_US) {
            new_speed = MOTOR_SPEED_REVERSE;
        } else if (current_pulse_width > PWM_DEADZONE_MAX_US) {
            new_speed = MOTOR_SPEED_FORWARD;
        }
        
        // Сброс защиты при движении в противоположном направлении
        if (current_protection_active && new_speed != MOTOR_SPEED_STOP) {
            if ((protection_direction > 0 && new_speed < 0) || 
                (protection_direction < 0 && new_speed > 0)) {
                current_protection_active = false;
            }
        }
    }
    
    // Принудительная остановка при защите
    if (current_protection_active) {
        new_speed = MOTOR_SPEED_STOP;
    }
    
    // Применяем новую скорость
    if (new_speed != motor_speed) {
        motor_speed = new_speed;
        gripperMotor.setSpeedSmooth(motor_speed);
        
        Serial.print("Motor: ");
        if (motor_speed == MOTOR_SPEED_STOP) {
            Serial.print("STOP");
        } else if (motor_speed > MOTOR_SPEED_STOP) {
            Serial.print("FORWARD");
        } else {
            Serial.print("REVERSE");
        }
        Serial.println(" (pulse: " + String(current_pulse_width) + "us)");
    }
}

// Функция вывода диагностики
void printDiagnostics() {
    uint32_t current_width = pulseMeter.getPulseWidth();
    float current_mA, voltage_V, power_mW;
    currentSensor.getAllMeasurements(current_mA, voltage_V, power_mW);
    
    // Диагностика PulseMeter
    bool pin_state, waiting_for_rising, new_pulse_available;
    pulseMeter.getDiagnostics(pin_state, waiting_for_rising, new_pulse_available);
    
    Serial.print("Pulse: " + String(current_width) + "us");
    Serial.print(" (pin:" + String(pin_state ? "H" : "L"));
    Serial.print(", wait:" + String(waiting_for_rising ? "R" : "F"));
    Serial.print(", new:" + String(new_pulse_available ? "Y" : "N") + ") | ");
    
    if (currentSensor.isInitialized()) {
        Serial.print("I: " + String(current_mA, 2) + "mA | ");
        Serial.print("V: " + String(voltage_V, 2) + "V | ");
        Serial.print("P: " + String(power_mW, 1) + "mW");
    } else {
        Serial.print("Ток: недоступен");
    }
    
    Serial.print(" | Motor: " + String(gripperMotor.getSpeed()));
    
    // Индикация состояния
    if (current_protection_active) {
        Serial.print(" [ЗАЩИТА]");
    } else if (motor_startup_delay_active) {
        Serial.print(" [СТАРТ]");
    } else {
        Serial.print(" [OK]");
    }
    
    Serial.println();
}

void loop() {
    unsigned long currentTime = millis();
    
    // Обновить измерения
    currentSensor.update();
    gripperMotor.update();
    
    // Основной цикл обновления каждые 20мс
    if (currentTime - lastUpdate >= 20) {
        checkCurrentProtection();
        
        // Обработка PWM сигнала
        if (pulseMeter.isNewPulseAvailable()) {
            processPWMControl();
        }
        
        // Вывод данных каждые 100мс
        static unsigned long lastPrint = 0;
        if (currentTime - lastPrint >= DATA_PRINT_INTERVAL_MS) {
            printDiagnostics();
            lastPrint = currentTime;
        }
        
        lastUpdate = currentTime;
    }
    
    // Небольшая задержка для стабильности
    delay(1);
}