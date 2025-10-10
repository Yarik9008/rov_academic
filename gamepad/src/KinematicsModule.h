#ifndef KINEMATICS_MODULE_H
#define KINEMATICS_MODULE_H

#include "Config.h"

// KINEMATICS MODULE - Модуль кинематики для различных конфигураций моторов

class KinematicsModule {
private:
    KinematicsConfig config;

public:
    // Конструктор
    KinematicsModule() {
        // Инициализация по умолчанию
        config.motor_count = DEFAULT_MOTOR_COUNT;
        // reverse_motor будет инициализирован в ROVController
    }
    
    // Основной метод расчета кинематики
    MotorPWMValues calculate(const JoystickData& joy_data, const KinematicsConfig& config) {
        this->config = config;
        
        // Валидация входных данных
        if (!isValidConfig(config)) {
            return createSafePWM();
        }
        
        switch (config.motor_count) {
            case 3: return calculate3Motor(joy_data);
            case 4: return calculate4Motor(joy_data);
            default: return calculate4Motor(joy_data);
        }
    }
    
    // Управление конфигурацией
    const KinematicsConfig& getConfig() const { return config; }
    
    // Валидация конфигурации
    bool isValidConfig(const KinematicsConfig& config) const {
        return (config.motor_count >= getMinMotors() && config.motor_count <= getMaxMotors());
    }
    
    // Получение информации о поддерживаемых схемах
    static const char* getSupportedSchemes() { return "3, 4 motors"; }
    static int getMaxMotors() { return 4; }
    static int getMinMotors() { return 3; }

private:
    // Создание безопасного PWM значения
    MotorPWMValues createSafePWM() const {
        MotorPWMValues safe_pwm;
        for (int i = 0; i < 8; i++) {
            ((int*)&safe_pwm)[i] = PWM_CENTER;
        }
        return safe_pwm;
    }
    
    // Реализация методов кинематик
    MotorPWMValues calculate3Motor(const JoystickData& joy_data) {
        MotorPWMValues pwm;
        
        // Предвычисление коэффициентов для упрощения
        float forward_back = joy_data.linear_x * PWM_RANGE / 2;
        float rotate = joy_data.rotate_y * PWM_RANGE / 2;
        float vertical = joy_data.linear_y * PWM_RANGE / 2;
        
        // Математика управления для 3 моторов (упрощенная)
        pwm.motor0_pwm = PWM_CENTER + forward_back + rotate;
        pwm.motor1_pwm = PWM_CENTER + forward_back - rotate;
        pwm.motor2_pwm = PWM_CENTER + vertical;
        
        // Остальные моторы в нейтральном положении
        pwm.motor3_pwm = PWM_CENTER;
        pwm.motor4_pwm = PWM_CENTER;
        pwm.motor5_pwm = PWM_CENTER;
        pwm.motor6_pwm = PWM_CENTER;
        pwm.motor7_pwm = PWM_CENTER;
        
        applyMotorReverseToAll(pwm);
        constrainPWM(pwm);
        return pwm;
    }
    
    MotorPWMValues calculate4Motor(const JoystickData& joy_data) {
        MotorPWMValues pwm;
        
        // Предвычисление коэффициентов для упрощения
        float forward_back = joy_data.linear_x * PWM_RANGE / 2;
        float rotate = joy_data.rotate_y * PWM_RANGE / 2;
        float vertical = joy_data.linear_y * PWM_RANGE / 2;  // Вертикальное движение
        
        // Математика управления для 4 моторов (2 горизонтальных + 2 вертикальных)
        pwm.motor0_pwm = PWM_CENTER + forward_back + rotate;  // Горизонтальный левый
        pwm.motor1_pwm = PWM_CENTER + forward_back - rotate;   // Горизонтальный правый
        pwm.motor2_pwm = PWM_CENTER + vertical;                // Вертикальный движитель 1
        pwm.motor3_pwm = PWM_CENTER + vertical;                // Вертикальный движитель 2
        
        // Остальные моторы в нейтральном положении
        pwm.motor4_pwm = PWM_CENTER;
        pwm.motor5_pwm = PWM_CENTER;
        pwm.motor6_pwm = PWM_CENTER;
        pwm.motor7_pwm = PWM_CENTER;
        
        applyMotorReverseToAll(pwm);
        constrainPWM(pwm);
        return pwm;
    }
    
    // Вспомогательные методы
    void applyMotorReverseToAll(MotorPWMValues& pwm) {
        if (config.reverse_motor[0]) pwm.motor0_pwm = reverseMotorDirection(pwm.motor0_pwm);
        if (config.reverse_motor[1]) pwm.motor1_pwm = reverseMotorDirection(pwm.motor1_pwm);
        if (config.reverse_motor[2]) pwm.motor2_pwm = reverseMotorDirection(pwm.motor2_pwm);
        if (config.reverse_motor[3]) pwm.motor3_pwm = reverseMotorDirection(pwm.motor3_pwm);
        if (config.reverse_motor[4]) pwm.motor4_pwm = reverseMotorDirection(pwm.motor4_pwm);
        if (config.reverse_motor[5]) pwm.motor5_pwm = reverseMotorDirection(pwm.motor5_pwm);
        if (config.reverse_motor[6]) pwm.motor6_pwm = reverseMotorDirection(pwm.motor6_pwm);
        if (config.reverse_motor[7]) pwm.motor7_pwm = reverseMotorDirection(pwm.motor7_pwm);
    }
    
    void constrainPWM(MotorPWMValues& pwm) {
        pwm.motor0_pwm = constrain(pwm.motor0_pwm, PWM_MIN, PWM_MAX);
        pwm.motor1_pwm = constrain(pwm.motor1_pwm, PWM_MIN, PWM_MAX);
        pwm.motor2_pwm = constrain(pwm.motor2_pwm, PWM_MIN, PWM_MAX);
        pwm.motor3_pwm = constrain(pwm.motor3_pwm, PWM_MIN, PWM_MAX);
        pwm.motor4_pwm = constrain(pwm.motor4_pwm, PWM_MIN, PWM_MAX);
        pwm.motor5_pwm = constrain(pwm.motor5_pwm, PWM_MIN, PWM_MAX);
        pwm.motor6_pwm = constrain(pwm.motor6_pwm, PWM_MIN, PWM_MAX);
        pwm.motor7_pwm = constrain(pwm.motor7_pwm, PWM_MIN, PWM_MAX);
    }
    
    int reverseMotorDirection(int pwm_value) {
        return 3000 - pwm_value;
    }
};

#endif // KINEMATICS_MODULE_H