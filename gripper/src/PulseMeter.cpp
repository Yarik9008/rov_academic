#include "PulseMeter.h"
#include "Config.h"

// Инициализация статического указателя на экземпляр
PulseMeter* PulseMeter::instance = nullptr;

/**
 * Конструктор класса PulseMeter
 * @param pin_number - номер пина для измерения импульсов
 */
PulseMeter::PulseMeter(uint8_t pin_number) 
    : pin(pin_number), pulse_width_us(0), waiting_for_rising(true), new_pulse_available(false) {
}

/**
 * Деструктор класса PulseMeter
 * Отключает прерывания если данный экземпляр был активным
 */
PulseMeter::~PulseMeter() {
    // Отключить прерывания если этот экземпляр был активным
    if (instance == this) {
        int interrupt_pin = digitalPinToInterrupt(pin);
        if (interrupt_pin != NOT_AN_INTERRUPT) {
            detachInterrupt(interrupt_pin);
        }
        instance = nullptr;
    }
}

/**
 * Инициализация измерения импульсов
 * Настраивает пин и подключает прерывания
 */
void PulseMeter::begin() {
    // Установить этот экземпляр как активный
    instance = this;
    
    // Настроить пин как вход с подтяжкой к земле
    pinMode(pin, INPUT_PULLDOWN);
    
    // Инициализировать состояние
    waiting_for_rising = true;
    pulse_width_us = 0;
    new_pulse_available = false;
    
    // Подключить прерывание на изменение состояния пина
    attachInterrupt(digitalPinToInterrupt(pin), handleInterrupt, CHANGE);
}

/**
 * Статический обработчик прерываний
 * Вызывает метод экземпляра класса
 */
void PulseMeter::handleInterrupt() {
    // Вызвать метод экземпляра
    if (instance != nullptr) {
        instance->handlePulseInterrupt();
    }
}

/**
 * Обработчик прерываний для измерения импульсов
 * Измеряет время между передним и задним фронтами
 */
void PulseMeter::handlePulseInterrupt() {
    static uint32_t last_rising_time = 0;  // Время последнего переднего фронта
    static uint32_t last_interrupt_time = 0;  // Время последнего прерывания
    uint32_t current_time = micros();      // Текущее время в микросекундах
    
    // Защита от дребезга контактов
    if (current_time - last_interrupt_time < DEBOUNCE_US) {
        return;
    }
    last_interrupt_time = current_time;
    
    // Читаем текущее состояние пина
    bool pin_state = digitalRead(pin);
    
    if (pin_state == HIGH && waiting_for_rising) {
        // Обнаружен передний фронт (переход 0->1)
        last_rising_time = current_time;
        waiting_for_rising = false;
    } else if (pin_state == LOW && !waiting_for_rising) {
        // Обнаружен задний фронт (переход 1->0)
        uint32_t width = current_time - last_rising_time;
        
        // Обработка переполнения micros()
        if (current_time < last_rising_time) {
            width = (MICROS_MAX - last_rising_time) + current_time + 1;
        }
        
        // Проверка валидности длины импульса
        if (width >= PULSE_MIN_US && width <= PULSE_MAX_US) {
            pulse_width_us = width;
            new_pulse_available = true;
        }
        waiting_for_rising = true;
    }
}

/**
 * Получить последнюю измеренную длину импульса
 * @return длина импульса в микросекундах
 */
uint32_t PulseMeter::getPulseWidth() const {
    return pulse_width_us;
}

/**
 * Проверить наличие нового измерения импульса
 * @return true если есть новое измерение
 */
bool PulseMeter::isNewPulseAvailable() const {
    return new_pulse_available;
}

/**
 * Получить длину импульса и сбросить флаг за один вызов
 * @return длина импульса в микросекундах
 */
uint32_t PulseMeter::getPulseWidthAndClear() {
    uint32_t width = pulse_width_us;
    new_pulse_available = false;
    return width;
}

/**
 * Получить диагностическую информацию
 * @param pin_state - текущее состояние пина
 * @param waiting_for_rising - флаг ожидания переднего фронта
 * @param new_pulse_available - флаг наличия нового измерения
 */
void PulseMeter::getDiagnostics(bool& pin_state, bool& waiting_for_rising, bool& new_pulse_available) const {
    pin_state = digitalRead(pin);
    waiting_for_rising = this->waiting_for_rising;
    new_pulse_available = this->new_pulse_available;
}
