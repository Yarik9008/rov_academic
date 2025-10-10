#include "MotorDriver.h"
#include "Config.h"

/**
 * Конструктор для управления двумя пинами с ШИМ
 * @param pin_a - пин A (для прямого вращения)
 * @param pin_b - пин B (для обратного вращения)
 */
MotorDriver::MotorDriver(uint8_t pin_a, uint8_t pin_b) 
    : pin_a(pin_a), pin_b(pin_b), current_speed(STOP_SPEED), is_enabled(false),
      target_speed(STOP_SPEED), start_speed(STOP_SPEED), step_count(0), 
      current_step(0), last_step_time(0), smooth_transition_active(false) {
}

/**
 * Инициализация драйвера
 */
void MotorDriver::begin() {
    // Настроить пины как выходы
    pinMode(pin_a, OUTPUT);
    pinMode(pin_b, OUTPUT);
    
    // Изначально остановить двигатель
    stop();
    is_enabled = true;
}

/**
 * Установить скорость двигателя
 * @param speed - скорость от -255 до +255 (отрицательные значения = обратное вращение)
 */
void MotorDriver::setSpeed(int16_t speed) {
    if (!is_enabled) {
        return;
    }
    
    // Валидация и ограничение скорости
    speed = clampSpeed(speed);
    
    // Обновить текущую скорость только если она изменилась
    if (current_speed != speed) {
        current_speed = speed;
        applyPWMSignals(speed);
    }
}

/**
 * Остановить двигатель
 */
void MotorDriver::stop() {
    setSpeed(STOP_SPEED);
}

/**
 * Получить текущую скорость
 * @return текущая скорость от -255 до +255
 */
int16_t MotorDriver::getSpeed() const {
    return current_speed;
}

/**
 * Получить диагностическую информацию
 * @param pin_a_value - текущее значение ШИМ на пине A
 * @param pin_b_value - текущее значение ШИМ на пине B
 */
void MotorDriver::getDiagnostics(uint8_t& pin_a_value, uint8_t& pin_b_value) const {
    // Возвращаем текущие значения ШИМ на основе скорости
    pin_a_value = (current_speed > STOP_SPEED) ? static_cast<uint8_t>(current_speed) : PWM_OFF;
    pin_b_value = (current_speed < STOP_SPEED) ? static_cast<uint8_t>(-current_speed) : PWM_OFF;
}

/**
 * Применить ШИМ сигналы к пинам двигателя
 * @param speed - скорость от -255 до +255
 */
void MotorDriver::applyPWMSignals(int16_t speed) {
    if (speed == STOP_SPEED) {
        // Остановка - оба пина в 0
        analogWrite(pin_a, PWM_OFF);
        analogWrite(pin_b, PWM_OFF);
    } else if (speed > STOP_SPEED) {
        // Прямое вращение - ШИМ на пин A, 0 на пин B
        analogWrite(pin_a, static_cast<uint8_t>(speed));
        analogWrite(pin_b, PWM_OFF);
    } else {
        // Обратное вращение - 0 на пин A, ШИМ на пин B
        analogWrite(pin_a, PWM_OFF);
        analogWrite(pin_b, static_cast<uint8_t>(-speed));
    }
}

/**
 * Ограничить скорость в допустимом диапазоне
 * @param speed - исходная скорость
 * @return ограниченная скорость
 */
int16_t MotorDriver::clampSpeed(int16_t speed) const {
    if (speed > MAX_SPEED) return MAX_SPEED;
    if (speed < MIN_SPEED) return MIN_SPEED;
    return speed;
}

/**
 * Проверить валидность скорости
 * @param speed - скорость для проверки
 * @return true если скорость в допустимом диапазоне
 */
bool MotorDriver::isValidSpeed(int16_t speed) const {
    return (speed >= MIN_SPEED && speed <= MAX_SPEED);
}

/**
 * Установить скорость с плавным переходом
 * @param speed - целевая скорость от -255 до +255
 */
void MotorDriver::setSpeedSmooth(int16_t speed) {
    if (!is_enabled) {
        return;
    }
    
    // Валидация и ограничение скорости
    speed = clampSpeed(speed);
    
    // Если скорость не изменилась, ничего не делаем
    if (current_speed == speed) {
        return;
    }
    
    // Если плавный старт отключен или это остановка - мгновенное изменение
    if (!SMOOTH_START_ENABLED || speed == STOP_SPEED) {
        current_speed = speed;
        smooth_transition_active = false;
        applyPWMSignals(speed);
        return;
    }
    
    // Запустить плавный переход через таймер
    startSmoothTransition(speed);
}

/**
 * Обновить плавный переход (вызывать в loop)
 */
void MotorDriver::update() {
    if (!smooth_transition_active || !is_enabled) {
        return;
    }
    
    unsigned long current_time = millis();
    
    // Проверяем, прошло ли достаточно времени для следующего шага
    if (current_time - last_step_time >= SMOOTH_START_STEP_MS) {
        // Вычисляем прогресс перехода (0.0 до 1.0)
        float progress = (float)current_step / step_count;
        
        // Применяем экспоненциальную кривую для более плавного старта
        float eased_progress = progress * progress * (3.0f - 2.0f * progress); // smoothstep
        
        // Вычисляем следующую скорость с учетом кривой
        int16_t speed_diff = target_speed - start_speed;
        int16_t next_speed = start_speed + (int16_t)(speed_diff * eased_progress);
        
        // Ограничиваем скорость
        next_speed = clampSpeed(next_speed);
        
        // Применяем скорость
        current_speed = next_speed;
        applyPWMSignals(next_speed);
        
        current_step++;
        last_step_time = current_time;
        
        // Проверяем, достигли ли целевой скорости
        if (current_step >= step_count) {
            // Завершаем переход
            current_speed = target_speed;
            applyPWMSignals(target_speed);
            smooth_transition_active = false;
        }
    }
}

/**
 * Проверить, активен ли плавный переход
 * @return true если плавный переход активен
 */
bool MotorDriver::isSmoothTransitionActive() const {
    return smooth_transition_active;
}

/**
 * Запустить плавный переход к целевой скорости
 * @param target_speed - целевая скорость
 */
void MotorDriver::startSmoothTransition(int16_t target_speed) {
    this->target_speed = target_speed;
    this->start_speed = current_speed;
    
    // Вычисляем количество шагов
    int16_t speed_diff = abs(target_speed - start_speed);
    step_count = speed_diff / SMOOTH_START_STEP_SIZE;
    
    // Увеличиваем количество шагов для более плавного старта
    step_count = max(step_count * 2, 10); // Минимум 10 шагов
    
    // Максимальное количество шагов для предотвращения слишком медленного старта
    if (step_count > 50) {
        step_count = 50;
    }
    
    // Сбрасываем счетчик шагов
    current_step = 0;
    last_step_time = millis();
    
    // Активируем плавный переход
    smooth_transition_active = true;
}

