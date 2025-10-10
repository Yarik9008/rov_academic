#ifndef JOYSTICK_MODULE_H
#define JOYSTICK_MODULE_H

#include "Config.h"

// JOYSTICK MODULE - Модуль обработки данных джойстиков

class JoystickModule {
private:
    struct Calibration {
        float joy1X_zero = ADC_MAX_VALUE / 2;
        float joy1Y_zero = ADC_MAX_VALUE / 2;
        float joy2X_zero = ADC_MAX_VALUE / 2;
        float joy2Y_zero = ADC_MAX_VALUE / 2;
        bool done = false;
    } calibration;
    
    // Фильтры Калмана
    KalmanFilter kalman_joy1X = {ADC_MAX_VALUE / 2, KALMAN_P_INIT, 0, KALMAN_Q, KALMAN_R};
    KalmanFilter kalman_joy1Y = {ADC_MAX_VALUE / 2, KALMAN_P_INIT, 0, KALMAN_Q, KALMAN_R};
    KalmanFilter kalman_joy2X = {ADC_MAX_VALUE / 2, KALMAN_P_INIT, 0, KALMAN_Q, KALMAN_R};
    KalmanFilter kalman_joy2Y = {ADC_MAX_VALUE / 2, KALMAN_P_INIT, 0, KALMAN_Q, KALMAN_R};
    
    // Настройки фильтрации
    bool enableKalmanFilter = true;
    

public:
    // Конструктор
    JoystickModule() {
        // Инициализация по умолчанию
        calibration.done = false;
        enableKalmanFilter = true;
    }
    
    // Основной метод чтения данных джойстика
    JoystickData readData() {
        // Чтение RAW значений джойстиков
        float raw_joy1X = analogRead(JOYSTICK1_X);
        float raw_joy1Y = analogRead(JOYSTICK1_Y);
        float raw_joy2X = analogRead(JOYSTICK2_X);
        float raw_joy2Y = analogRead(JOYSTICK2_Y);
        
        // Применение фильтра Калмана
        float processed_joy1X = enableKalmanFilter ? applyKalmanFilter(kalman_joy1X, raw_joy1X) : raw_joy1X;
        float processed_joy1Y = enableKalmanFilter ? applyKalmanFilter(kalman_joy1Y, raw_joy1Y) : raw_joy1Y;
        float processed_joy2X = enableKalmanFilter ? applyKalmanFilter(kalman_joy2X, raw_joy2X) : raw_joy2X;
        float processed_joy2Y = enableKalmanFilter ? applyKalmanFilter(kalman_joy2Y, raw_joy2Y) : raw_joy2Y;
        
        // Вычисление отклонений
        int raw_offset_joy1X = round(processed_joy1X - calibration.joy1X_zero);
        int raw_offset_joy1Y = round(processed_joy1Y - calibration.joy1Y_zero);
        int raw_offset_joy2X = round(processed_joy2X - calibration.joy2X_zero);
        int raw_offset_joy2Y = round(processed_joy2Y - calibration.joy2Y_zero);
        
        // Применение мертвой зоны
        int output_joy1X = applyDeadZone(raw_offset_joy1X);
        int output_joy1Y = applyDeadZone(raw_offset_joy1Y);
        int output_joy2X = applyDeadZone(raw_offset_joy2X);
        int output_joy2Y = applyDeadZone(raw_offset_joy2Y);
        
        // Нормализация в диапазон [-1.0, 1.0]
        JoystickData normalized_data = normalizeJoystickData(output_joy1X, output_joy1Y, output_joy2X, output_joy2Y);
        
        // Чтение состояния кнопок
        readButtons(normalized_data);
        
        // Применение коэффициентов вклада
        return applyCoefficients(normalized_data);
    }
    
    // Калибровка джойстиков
    void calibrate() {
        if (calibration.done) return;
        
        // Чтение нулевых значений
        calibration.joy1X_zero = analogRead(JOYSTICK1_X);
        calibration.joy1Y_zero = analogRead(JOYSTICK1_Y);
        calibration.joy2X_zero = analogRead(JOYSTICK2_X);
        calibration.joy2Y_zero = analogRead(JOYSTICK2_Y);
        
        calibration.done = true;
    }
    
    
    // Сброс калибровки
    void resetCalibration() { calibration.done = false; }
    
    // Проверка калибровки
    bool isCalibrated() const { return calibration.done; }
    
    
    // Валидация данных джойстика
    bool isValidJoystickData(const JoystickData& data) const {
        return (data.linear_x >= -1.0 && data.linear_x <= 1.0 &&
                data.linear_y >= -1.0 && data.linear_y <= 1.0 &&
                data.linear_z >= -1.0 && data.linear_z <= 1.0 &&
                data.rotate_x >= -1.0 && data.rotate_x <= 1.0 &&
                data.rotate_y >= -1.0 && data.rotate_y <= 1.0 &&
                data.rotate_z >= -1.0 && data.rotate_z <= 1.0);
    }

private:
    // Реализация приватных методов
    float applyKalmanFilter(KalmanFilter &kf, float measurement) {
        kf.P = kf.P + kf.Q;
        kf.K = kf.P / (kf.P + kf.R);
        kf.x = kf.x + kf.K * (measurement - kf.x);
        kf.P = (1 - kf.K) * kf.P;
        return kf.x;
    }
    
    int applyDeadZone(int value) {
        return (abs(value) <= DEAD_ZONE) ? 0 : value;
    }
    
    JoystickData normalizeJoystickData(int joy1X_offset, int joy1Y_offset, int joy2X_offset, int joy2Y_offset) {
        static const int MAX_JOYSTICK_RANGE = 1500;
        JoystickData joy_data;
        
        // Нормализация в диапазон [-1.0, 1.0]
        joy_data.linear_x = constrain(joy1X_offset, -MAX_JOYSTICK_RANGE, MAX_JOYSTICK_RANGE) / (float)MAX_JOYSTICK_RANGE;
        joy_data.rotate_y = constrain(joy1Y_offset, -MAX_JOYSTICK_RANGE, MAX_JOYSTICK_RANGE) / (float)MAX_JOYSTICK_RANGE;
        joy_data.linear_y = constrain(joy2X_offset, -MAX_JOYSTICK_RANGE, MAX_JOYSTICK_RANGE) / (float)MAX_JOYSTICK_RANGE;
        joy_data.linear_z = constrain(joy2Y_offset, -MAX_JOYSTICK_RANGE, MAX_JOYSTICK_RANGE) / (float)MAX_JOYSTICK_RANGE;
        
        // Остальные поля
        joy_data.rotate_x = 0.0;
        joy_data.rotate_z = 0.0;
        
        return joy_data;
    }
    
    // Чтение состояния кнопок
    void readButtons(JoystickData& joy_data) {
        // Чтение состояния кнопок (инверсия для INPUT_PULLUP)
        joy_data.servo_cam = 0;  // По умолчанию нет команды
        
        bool button_left = !digitalRead(BUTTON0);   // Поворот камеры влево
        bool button_right = !digitalRead(BUTTON1);  // Поворот камеры вправо
        
        if (button_left && !button_right) {
            joy_data.servo_cam = -1;  // Поворот влево
        } else if (button_right && !button_left) {
            joy_data.servo_cam = 1;   // Поворот вправо
        }
        
        // Манипулятор
        joy_data.gripper = 0;  // По умолчанию нет команды
        
        bool button_open = !digitalRead(BUTTON2);    // Открыть манипулятор
        bool button_close = !digitalRead(BUTTON3);   // Закрыть манипулятор
        
        if (button_open && !button_close) {
            joy_data.gripper = -1;  // Открыть
        } else if (button_close && !button_open) {
            joy_data.gripper = 1;   // Закрыть
        }
        
        // Освещение
        joy_data.led = 0;  // По умолчанию нет команды
        
        bool button_light_on = !digitalRead(BUTTON4);   // Включить освещение
        bool button_light_off = !digitalRead(BUTTON5);   // Выключить освещение
        
        if (button_light_on && !button_light_off) {
            joy_data.led = 1;   // Включить
        } else if (button_light_off && !button_light_on) {
            joy_data.led = -1;  // Выключить
        }
    }
    
    // Применение коэффициентов вклада к нормализованным данным
    JoystickData applyCoefficients(const JoystickData& normalized_data) {
        JoystickData joy_data = normalized_data;
        
        // Статические коэффициенты из Config.h
        static const JoystickCoefficients static_coeffs;
        
        // Применение коэффициентов вклада по осям джойстиков
        joy_data.linear_x *= static_coeffs.stick1_x;
        joy_data.rotate_y *= static_coeffs.stick1_y;
        joy_data.linear_y *= static_coeffs.stick2_x;
        joy_data.linear_z *= static_coeffs.stick2_y;
        
        // Ограничение результата в диапазоне [-1.0, 1.0]
        joy_data.linear_x = constrain(joy_data.linear_x, -1.0, 1.0);
        joy_data.rotate_y = constrain(joy_data.rotate_y, -1.0, 1.0);
        joy_data.linear_y = constrain(joy_data.linear_y, -1.0, 1.0);
        joy_data.linear_z = constrain(joy_data.linear_z, -1.0, 1.0);
        
        return joy_data;
    }
};

#endif // JOYSTICK_MODULE_H