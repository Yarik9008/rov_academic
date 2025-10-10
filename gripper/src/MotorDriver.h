#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

/**
 * Класс для управления драйвером двигателя L9110s
 * Управление осуществляется двумя пинами с ШИМ сигналами:
 * - Прямое вращение: ШИМ на пин A, 0 на пин B
 * - Обратное вращение: 0 на пин A, ШИМ на пин B
 * - Остановка: 0 на оба пина
 */
class MotorDriver {
private:
    // Константы
    static constexpr int16_t MAX_SPEED = 255;
    static constexpr int16_t MIN_SPEED = -255;
    static constexpr int16_t STOP_SPEED = 0;
    static constexpr uint8_t PWM_OFF = 0;
    
    // Поля класса
    const uint8_t pin_a;         // Пин A (для прямого вращения)
    const uint8_t pin_b;         // Пин B (для обратного вращения)
    int16_t current_speed;       // Текущая скорость (-255 до +255)
    bool is_enabled;             // Включен ли драйвер
    
    // Поля для плавного перехода через таймер
    int16_t target_speed;         // Целевая скорость
    int16_t start_speed;          // Начальная скорость перехода
    int16_t step_count;            // Общее количество шагов
    int16_t current_step;          // Текущий шаг
    unsigned long last_step_time;  // Время последнего шага
    bool smooth_transition_active; // Активен ли плавный переход
    
    // Приватные вспомогательные методы
    void applyPWMSignals(int16_t speed);
    int16_t clampSpeed(int16_t speed) const;
    bool isValidSpeed(int16_t speed) const;
    void startSmoothTransition(int16_t target_speed);

public:
    /**
     * Конструктор для управления двумя пинами с ШИМ
     * @param pin_a - пин A (для прямого вращения)
     * @param pin_b - пин B (для обратного вращения)
     */
    MotorDriver(uint8_t pin_a, uint8_t pin_b);
    
    /**
     * Инициализация драйвера
     */
    void begin();
    
    /**
     * Установить скорость двигателя
     * @param speed - скорость от -255 до +255 (отрицательные значения = обратное вращение)
     */
    void setSpeed(int16_t speed);
    
    /**
     * Остановить двигатель
     */
    void stop();
    
    /**
     * Получить текущую скорость
     * @return текущая скорость от -255 до +255
     */
    int16_t getSpeed() const;
    
    /**
     * Получить диагностическую информацию
     * @param pin_a_value - текущее значение ШИМ на пине A
     * @param pin_b_value - текущее значение ШИМ на пине B
     */
    void getDiagnostics(uint8_t& pin_a_value, uint8_t& pin_b_value) const;
    
    /**
     * Установить скорость с плавным переходом
     * @param speed - целевая скорость от -255 до +255
     */
    void setSpeedSmooth(int16_t speed);
    
    /**
     * Обновить плавный переход (вызывать в loop)
     */
    void update();
    
    /**
     * Проверить, активен ли плавный переход
     * @return true если плавный переход активен
     */
    bool isSmoothTransitionActive() const;
};

#endif // MOTOR_DRIVER_H
