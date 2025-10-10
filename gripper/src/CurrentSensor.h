#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "Config.h"

/**
 * Класс для измерения тока с помощью датчика INA219
 * Обеспечивает простое измерение тока, напряжения и мощности
 */
class CurrentSensor {
private:
    Adafruit_INA219 ina219;    // Экземпляр датчика INA219
    uint8_t sda_pin;          // Пин SDA для I2C
    uint8_t scl_pin;          // Пин SCL для I2C
    bool sensor_initialized;   // Флаг инициализации датчика
    float current_mA;          // Текущее значение тока в миллиамперах
    float voltage_V;           // Текущее значение напряжения в вольтах
    float power_mW;            // Текущее значение мощности в милливаттах
    float last_valid_current;  // Последнее валидное значение тока
    unsigned long last_measurement;  // Время последнего измерения
    const unsigned long measurement_interval = CURRENT_MEASUREMENT_INTERVAL; // Интервал измерения в мс
    const float dead_zone_mA = CURRENT_DEADZONE_MA;  // Мертвая зона в мА

public:
    /**
     * Конструктор класса CurrentSensor
     * @param sda_pin_number - номер пина SDA для I2C
     * @param scl_pin_number - номер пина SCL для I2C
     */
    CurrentSensor(uint8_t sda_pin_number, uint8_t scl_pin_number);
    
    /**
     * Инициализация датчика тока
     * @return true если инициализация прошла успешно
     */
    bool begin();
    
    /**
     * Обновить измерения тока, напряжения и мощности
     * Вызывать периодически для получения актуальных данных
     */
    void update();
    
    /**
     * Получить текущий ток в миллиамперах
     * @return ток в мА
     */
    float getCurrent_mA() const;
    
    /**
     * Получить текущее напряжение в вольтах
     * @return напряжение в В
     */
    float getVoltage_V() const;
    
    /**
     * Получить текущую мощность в милливаттах
     * @return мощность в мВт
     */
    float getPower_mW() const;
    
    /**
     * Проверить, инициализирован ли датчик
     * @return true если датчик готов к работе
     */
    bool isInitialized() const;
    
    /**
     * Получить все измерения за один вызов
     * @param current_mA - ссылка для записи тока в мА
     * @param voltage_V - ссылка для записи напряжения в В
     * @param power_mW - ссылка для записи мощности в мВт
     */
    void getAllMeasurements(float& current_mA, float& voltage_V, float& power_mW) const;
    
    /**
     * Получить значение мертвой зоны
     * @return мертвая зона в мА
     */
    float getDeadZone() const;
};

#endif // CURRENT_SENSOR_H
