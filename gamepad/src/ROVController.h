#ifndef ROV_CONTROLLER_H
#define ROV_CONTROLLER_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>
#include "Config.h"
#include "KinematicsModule.h"
#include "JoystickModule.h"

// ROV CONTROLLER CLASS - Основной класс управления ROV

class ROVController {
private:
    // Внутренние состояния
    struct SystemState {
        unsigned long lastDataTime = 0;
        unsigned long lastLedTime = 0;
        bool ledState = false;
    } state;
    
    // Состояние полезной нагрузки
    struct PayloadState {
        int camera_servo = PWM_CENTER;
        int manipulator = PWM_MIN;
        int light = PWM_MIN;  // Начальное состояние - выключено
        unsigned long lastCameraUpdate = 0;
    } payload_state;
    
    // Модули системы
    KinematicsModule kinematics;
    JoystickModule joystick;
    
    // Конфигурация
    KinematicsConfig kinematics_config;
    
    // Приватные методы (реализованы ниже)
    
public:
    // Конструктор
    ROVController() {
        // Инициализация конфигурации из Config.h
        kinematics_config.motor_count = DEFAULT_MOTOR_COUNT;
        MotorReverseConfig default_reverse;
        kinematics_config.reverse_motor[0] = default_reverse.motor0;
        kinematics_config.reverse_motor[1] = default_reverse.motor1;
        kinematics_config.reverse_motor[2] = default_reverse.motor2;
        kinematics_config.reverse_motor[3] = default_reverse.motor3;
        kinematics_config.reverse_motor[4] = default_reverse.motor4;
        kinematics_config.reverse_motor[5] = default_reverse.motor5;
        kinematics_config.reverse_motor[6] = default_reverse.motor6;
        kinematics_config.reverse_motor[7] = default_reverse.motor7;
    }
    
    // Основные методы
    void initialize() {
        // Настройка пинов
        pinMode(LED_PIN, OUTPUT);
        pinMode(BUTTON0, INPUT_PULLUP);
        pinMode(BUTTON1, INPUT_PULLUP);
        pinMode(BUTTON2, INPUT_PULLUP);
        pinMode(BUTTON3, INPUT_PULLUP);
        pinMode(BUTTON4, INPUT_PULLUP);
        pinMode(BUTTON5, INPUT_PULLUP);
        
        // Настройка ADC
        analogReadResolution(12);
        
        // Вывод информации о системе
        Serial2.println("=== ROV GAMEPAD CONTROL SYSTEM ===");
        Serial2.println("ADC Resolution: 12-bit (0-4095)");
        Serial2.println("Joystick1: X=PA0 (forward/backward), Y=PA1 (rotate)");
        Serial2.println("Joystick2: X=PA2 (vertical), Y=PA3 (unused)");
        Serial2.println("PWM Range: 1000-2000 (1500=stop)");
        Serial2.println("Camera: B0(left), B1(right) - Incremental");
        Serial2.println("Manipulator: B2(open), B3(close) - Instant");
        Serial2.println("Light: B4(on), B5(off) - PWM control");
        Serial2.println("Dead zone: ±10 units");
        Serial2.println("Supported kinematics: " + String(KinematicsModule::getSupportedSchemes()));
        Serial2.println("Joystick coefficients: Static configuration from Config.h");
        Serial2.println("================================");
        
        // Калибровка джойстиков
        calibrateJoysticks();
    }
    
    void update() {
        unsigned long currentTime = millis();
        
        // Обновление LED
        updateLED();
        
        // Основной цикл обработки данных
        if (currentTime - state.lastDataTime >= DATA_INTERVAL) {
            // Чтение данных джойстика
            JoystickData joy_data = joystick.readData();
            
            // Валидация данных джойстика
            if (!joystick.isValidJoystickData(joy_data)) {
                // Пропускаем невалидные данные
            }
            
            // Вычисление кинематики
            MotorPWMValues motor_pwm = kinematics.calculate(joy_data, kinematics_config);
            
            // Обновление полезной нагрузки с данными от джойстика
            updateCamera(joy_data);
            updateManipulator(joy_data);
            updateLighting(joy_data);
            
            // Формирование объединенных команд ROV
            ROVCommands commands;
            commands.motor0_pwm = motor_pwm.motor0_pwm;
            commands.motor1_pwm = motor_pwm.motor1_pwm;
            commands.motor2_pwm = motor_pwm.motor2_pwm;
            commands.motor3_pwm = motor_pwm.motor3_pwm;
            commands.motor4_pwm = motor_pwm.motor4_pwm;
            commands.motor5_pwm = motor_pwm.motor5_pwm;
            commands.motor6_pwm = motor_pwm.motor6_pwm;
            commands.motor7_pwm = motor_pwm.motor7_pwm;
            commands.camera_servo = payload_state.camera_servo;
            commands.manipulator = payload_state.manipulator;
            commands.light = payload_state.light;
            
            // Вывод управляющих команд
            outputCommands(commands);
            
            // Дополнительная телеметрия при включенном debug
            #ifdef DEBUG_TELEMETRY
            if (DEBUG_TELEMETRY) {
                outputTelemetry(joy_data, commands);
            }
            #endif
            
            state.lastDataTime = currentTime;
        }
    }
    
    void calibrateJoysticks() {
        joystick.calibrate();
    }
    
    // Геттеры для состояния системы
    bool isCalibrated() const { return joystick.isCalibrated(); }
    int getCameraPosition() const { return payload_state.camera_servo; }
    int getManipulatorPosition() const { return payload_state.manipulator; }
    int getLightLevel() const { return payload_state.light; }

private:
    // Реализация приватных методов
    void updateLED() {
        unsigned long currentTime = millis();
        if (currentTime - state.lastLedTime >= LED_INTERVAL) {
            state.ledState = !state.ledState;
            digitalWrite(LED_PIN, state.ledState ? LOW : HIGH);
            state.lastLedTime = currentTime;
        }
    }
    
    void outputCommands(const ROVCommands& commands) {
        Serial1.print(commands.motor0_pwm);
        Serial1.print(" ");
        Serial1.print(commands.motor1_pwm);
        Serial1.print(" ");
        Serial1.print(commands.motor2_pwm);
        Serial1.print(" ");
        Serial1.print(commands.motor3_pwm);
        Serial1.print(" ");
        Serial1.print(commands.motor4_pwm);
        Serial1.print(" ");
        Serial1.print(commands.motor5_pwm);
        Serial1.print(" ");
        Serial1.print(commands.motor6_pwm);
        Serial1.print(" ");
        Serial1.print(commands.motor7_pwm);
        Serial1.print(" ");
        Serial1.print(commands.camera_servo);
        Serial1.print(" ");
        Serial1.print(commands.manipulator);
        Serial1.print(" ");
        Serial1.println(commands.light);
    }
    
    void outputTelemetry(const JoystickData& joy_data, const ROVCommands& commands) {
        // Первая строка: показания джойстиков и кнопок
        Serial2.print("JOYSTICKS: ");
        Serial2.print("J1X:");
        Serial2.print(joy_data.linear_x, 2);
        Serial2.print(" J1Y:");
        Serial2.print(joy_data.rotate_y, 2);
        Serial2.print(" J2X:");
        Serial2.print(joy_data.linear_y, 2);
        Serial2.print(" J2Y:");
        Serial2.print(joy_data.linear_z, 2);
        Serial2.print(" | BUTTONS: ");
        Serial2.print("B0:");
        Serial2.print(!digitalRead(BUTTON0));
        Serial2.print(" B1:");
        Serial2.print(!digitalRead(BUTTON1));
        Serial2.print(" B2:");
        Serial2.print(!digitalRead(BUTTON2));
        Serial2.print(" B3:");
        Serial2.print(!digitalRead(BUTTON3));
        Serial2.print(" B4:");
        Serial2.print(!digitalRead(BUTTON4));
        Serial2.print(" B5:");
        Serial2.println(!digitalRead(BUTTON5));
        
        // Вторая строка: отправляемый пакет команд
        Serial2.print("COMMANDS: ");
        Serial2.print("M0:");
        Serial2.print(commands.motor0_pwm);
        Serial2.print(" M1:");
        Serial2.print(commands.motor1_pwm);
        Serial2.print(" M2:");
        Serial2.print(commands.motor2_pwm);
        Serial2.print(" M3:");
        Serial2.print(commands.motor3_pwm);
        Serial2.print(" M4:");
        Serial2.print(commands.motor4_pwm);
        Serial2.print(" M5:");
        Serial2.print(commands.motor5_pwm);
        Serial2.print(" M6:");
        Serial2.print(commands.motor6_pwm);
        Serial2.print(" M7:");
        Serial2.print(commands.motor7_pwm);
        Serial2.print(" CAM:");
        Serial2.print(commands.camera_servo);
        Serial2.print(" MAN:");
        Serial2.print(commands.manipulator);
        Serial2.print(" LIGHT:");
        Serial2.println(commands.light);
    }
    
    void updateCamera(const JoystickData& joy_data) {
        unsigned long current_time = millis();
        if (current_time - payload_state.lastCameraUpdate >= CAMERA_UPDATE_INTERVAL) {
            int camera_direction = joy_data.servo_cam;
            
            // Применяем инверсию если включена
            if (CAMERA_INVERT) {
                camera_direction = -camera_direction;
            }
            
            if (camera_direction == -1) {
                // Поворот вниз (или влево при инверсии)
                payload_state.camera_servo -= CAMERA_SPEED;
                payload_state.camera_servo = constrain(payload_state.camera_servo, CAMERA_SAFE_MIN, CAMERA_SAFE_MAX);
            } else if (camera_direction == 1) {
                // Поворот вверх (или вправо при инверсии)
                payload_state.camera_servo += CAMERA_SPEED;
                payload_state.camera_servo = constrain(payload_state.camera_servo, CAMERA_SAFE_MIN, CAMERA_SAFE_MAX);
            }
            
            payload_state.lastCameraUpdate = current_time;
        }
    }
    
    void updateManipulator(const JoystickData& joy_data) {
        if (joy_data.gripper == -1) {
            // Открыть манипулятор
            payload_state.manipulator = PWM_MIN;
        } else if (joy_data.gripper == 1) {
            // Закрыть манипулятор
            payload_state.manipulator = PWM_MAX;
        }
    }
    
    void updateLighting(const JoystickData& joy_data) {
        if (joy_data.led == -1) {
            // Выключить освещение
            payload_state.light = LIGHT_OFF;
        } else if (joy_data.led == 1) {
            // Включить освещение
            payload_state.light = LIGHT_ON;
        }
    }
};

#endif // ROV_CONTROLLER_H