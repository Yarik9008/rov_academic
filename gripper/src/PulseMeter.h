#ifndef PULSE_METER_H
#define PULSE_METER_H

#include <Arduino.h>

/**
 * Класс для измерения длины импульсов на заданном пине
 * Использует прерывания для точного измерения времени
 */
class PulseMeter {
private:
    // Константы
    static constexpr uint32_t MICROS_MAX = 0xFFFFFFFF;
    static constexpr uint32_t DEBOUNCE_US = 10;  // Защита от дребезга
    
    // Поля класса
    const uint8_t pin;                    // Номер пина для измерения
    volatile uint32_t pulse_width_us;     // Длина импульса в микросекундах
    volatile bool waiting_for_rising;     // Флаг ожидания переднего фронта
    bool new_pulse_available;             // Флаг наличия нового измерения
    
    // Статический обработчик прерываний
    static PulseMeter* instance;
    static void handleInterrupt();
    
    // Обработчик прерываний для конкретного экземпляра
    void handlePulseInterrupt();

public:
    /**
     * Конструктор
     * @param pin_number - номер пина для измерения импульсов
     */
    PulseMeter(uint8_t pin_number);
    
    /**
     * Деструктор - отключает прерывания
     */
    ~PulseMeter();
    
    /**
     * Инициализация измерения импульсов
     * Настраивает пин и подключает прерывания
     */
    void begin();
    
    /**
     * Получить последнюю измеренную длину импульса в микросекундах
     * @return длина импульса в микросекундах
     */
    uint32_t getPulseWidth() const;
    
    /**
     * Проверить наличие нового измерения импульса
     * @return true если есть новое измерение
     */
    bool isNewPulseAvailable() const;
    
    /**
     * Получить длину импульса и сбросить флаг за один вызов
     * @return длина импульса в микросекундах
     */
    uint32_t getPulseWidthAndClear();
    
    /**
     * Получить диагностическую информацию
     * @param pin_state - текущее состояние пина
     * @param waiting_for_rising - флаг ожидания переднего фронта
     * @param new_pulse_available - флаг наличия нового измерения
     */
    void getDiagnostics(bool& pin_state, bool& waiting_for_rising, bool& new_pulse_available) const;
};

#endif // PULSE_METER_H
