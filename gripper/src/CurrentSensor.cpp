#include "CurrentSensor.h"

/**
 * Конструктор класса CurrentSensor
 * @param sda_pin_number - номер пина SDA для I2C
 * @param scl_pin_number - номер пина SCL для I2C
 */
CurrentSensor::CurrentSensor(uint8_t sda_pin_number, uint8_t scl_pin_number) 
    : sda_pin(sda_pin_number), scl_pin(scl_pin_number), sensor_initialized(false), 
      current_mA(0.0), voltage_V(0.0), power_mW(0.0), last_valid_current(0.0), last_measurement(0) {
}

/**
 * Инициализация датчика тока
 * @return true если инициализация прошла успешно
 */
bool CurrentSensor::begin() {
    // Инициализация I2C шины с указанными пинами
    Wire.begin(sda_pin, scl_pin);
    
    // Инициализация датчика INA219
    if (ina219.begin()) {
        sensor_initialized = true;
        
        // Настройка диапазона измерения для более точных измерений малых токов
        // 32V, 1A - лучшая точность для малых токов
        ina219.setCalibration_32V_1A();
        
        return true;
    } else {
        sensor_initialized = false;
        return false;
    }
}

/**
 * Обновить измерения тока, напряжения и мощности
 * Вызывать периодически для получения актуальных данных
 */
void CurrentSensor::update() {
    // Проверяем, инициализирован ли датчик
    if (!sensor_initialized) {
        return;
    }
    
    // Проверяем интервал измерения
    unsigned long current_time = millis();
    if (current_time - last_measurement < measurement_interval) {
        return;
    }
    
    // Выполняем измерения с проверкой ошибок
    float raw_current = 0.0;
    float voltage = 0.0;
    float power = 0.0;
    
    // Безопасное чтение с проверкой
    if (ina219.getCurrent_mA() != NAN) {
        raw_current = ina219.getCurrent_mA();
    }
    if (ina219.getBusVoltage_V() != NAN) {
        voltage = ina219.getBusVoltage_V();
    }
    if (ina219.getPower_mW() != NAN) {
        power = ina219.getPower_mW();
    }
    
    voltage_V = voltage;
    power_mW = power;
    
    // Простая фильтрация (скользящее среднее)
    static float current_history[3] = {0, 0, 0};
    static uint8_t history_index = 0;
    
    current_history[history_index] = raw_current;
    history_index = (history_index + 1) % 3;
    
    // Вычисляем среднее значение
    float filtered_current = (current_history[0] + current_history[1] + current_history[2]) / 3.0;
    
    // Применяем мертвую зону
    float current_diff = filtered_current - last_valid_current;
    if (current_diff < 0) current_diff = -current_diff; // abs() для float
    
    if (current_diff >= dead_zone_mA) {
        current_mA = filtered_current;
        last_valid_current = filtered_current;
    }
    
    // Обновляем время последнего измерения
    last_measurement = current_time;
}

/**
 * Получить текущий ток в миллиамперах
 * @return ток в мА
 */
float CurrentSensor::getCurrent_mA() const {
    return current_mA;
}

/**
 * Получить текущее напряжение в вольтах
 * @return напряжение в В
 */
float CurrentSensor::getVoltage_V() const {
    return voltage_V;
}

/**
 * Получить текущую мощность в милливаттах
 * @return мощность в мВт
 */
float CurrentSensor::getPower_mW() const {
    return power_mW;
}

/**
 * Проверить, инициализирован ли датчик
 * @return true если датчик готов к работе
 */
bool CurrentSensor::isInitialized() const {
    return sensor_initialized;
}

/**
 * Получить все измерения за один вызов
 * @param current_mA - ссылка для записи тока в мА
 * @param voltage_V - ссылка для записи напряжения в В
 * @param power_mW - ссылка для записи мощности в мВт
 */
void CurrentSensor::getAllMeasurements(float& current_mA, float& voltage_V, float& power_mW) const {
    current_mA = this->current_mA;
    voltage_V = this->voltage_V;
    power_mW = this->power_mW;
}

/**
 * Получить значение мертвой зоны
 * @return мертвая зона в мА
 */
float CurrentSensor::getDeadZone() const {
    return dead_zone_mA;
}
