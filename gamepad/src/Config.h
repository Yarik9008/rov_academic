#ifndef CONFIG_H
#define CONFIG_H

// НАСТРОЙКИ ПРОЕКТА - ROV GAMEPAD CONTROL SYSTEM

// Пины UART1 (Hardware Serial)
#define SERIAL1_RX PA10
#define SERIAL1_TX PA9

// Пины UART2 (Debug Serial)
#define SERIAL2_RX PB11
#define SERIAL2_TX PB10

// Объявления Serial портов
// Serial1 объявлен в фреймворке Arduino для STM32
extern HardwareSerial Serial2;

// Пины джойстиков (12-bit ADC: 0-4095)
#define JOYSTICK1_X PA0    // Джойстик 1 - X ось (вперед/назад)
#define JOYSTICK1_Y PA1    // Джойстик 1 - Y ось (поворот влево/вправо)
#define JOYSTICK2_X PA2    // Джойстик 2 - X ось (движение вверх/вниз)
#define JOYSTICK2_Y PA3    // Джойстик 2 - Y ось (не используется)

// Пины кнопок (INPUT_PULLUP)
#define BUTTON0 PA4        // B0 - Поворот камеры вниз
#define BUTTON1 PA5        // B1 - Поворот камеры вверх
#define BUTTON2 PA6        // B2 - Открыть манипулятор
#define BUTTON3 PA7        // B3 - Закрыть манипулятор
#define BUTTON4 PB0        // B4 - Включить освещение
#define BUTTON5 PB1        // B5 - Выключить освещение

// Светодиод
#define LED_PIN PC13       // Встроенный светодиод (индикация работы)

// Настройки ADC
#define ADC_RESOLUTION 12          // Наше разрешение ADC (бит)
#define ADC_MAX_VALUE 4095         // Максимальное значение ADC

// Коэффициенты вклада джойстиков по осям стиков (0.0 - 1.0, где 1.0 = 100%)
struct JoystickCoefficients {
    float stick1_x = 0.5;    // Джойстик 1 - ось X (вперед/назад)
    float stick1_y = 0.3;    // Джойстик 1 - ось Y (поворот влево/вправо)
    float stick2_x = 0.75;    // Джойстик 2 - ось X (движение вверх/вниз)
    float stick2_y = 1.0;    // Джойстик 2 - ось Y (не используется)
};

// Настройки серийного порта
#define SERIAL_BAUD 115200           // Скорость Serial1
#define DEBUG_BAUD 115200           // Скорость Serial2 (debug)

// Настройки кинематики
#define DEFAULT_MOTOR_COUNT 4      // Количество моторов по умолчанию (3, 4)

// Настройки фильтра Калмана
#define KALMAN_Q 25                  // Шум процесса (чувствительность к изменениям)
#define KALMAN_R 20                  // Шум измерений (сглаживание шума)
#define KALMAN_P_INIT 1000           // Начальное значение ковариации

// Валидация конфигурации
#define MIN_KALMAN_Q 1
#define MAX_KALMAN_Q 100
#define MIN_KALMAN_R 1
#define MAX_KALMAN_R 100
#define MIN_KALMAN_P_INIT 100
#define MAX_KALMAN_P_INIT 10000

// Настройки таймеров (миллисекунды)
#define DATA_INTERVAL 50            // Интервал отправки данных
#define LED_INTERVAL 200             // Интервал мигания светодиода
#define CAMERA_UPDATE_INTERVAL 100    // Интервал обновления камеры

// Настройки сервопривода камеры
#define CAMERA_SPEED 25              // Скорость изменения положения камеры
#define CAMERA_SAFE_MIN (PWM_MIN + 100)  // Безопасный минимум камеры
#define CAMERA_SAFE_MAX (PWM_MAX - 100)  // Безопасный максимум камеры
#define CAMERA_INVERT false          // Инвертировать направление поворота камеры

// Настройки манипулятора
#define MANIPULATOR_SAFE_MIN (PWM_MIN + 250)  // Безопасный минимум манипулятора
#define MANIPULATOR_SAFE_MAX (PWM_MAX - 250)  // Безопасный максимум манипулятора

// Настройки реверса моторов (по умолчанию все моторы не реверсированы)
struct MotorReverseConfig {
    bool motor0 = false;
    bool motor1 = false;
    bool motor2 = false;
    bool motor3 = false;
    bool motor4 = false;
    bool motor5 = false;
    bool motor6 = false;
    bool motor7 = false;
};


// Настройки освещения
#define LIGHT_OFF PWM_MIN   // Свет выключен (1000)
#define LIGHT_ON PWM_MAX    // Максимальная яркость (2000)

// Настройки управления выводом
#define DEBUG_TELEMETRY false     // Включить дополнительную телеметрию при отладке

// Мертвая зона джойстиков
#define DEAD_ZONE 25                 // Зона нечувствительности (±25 единиц)

// PWM константы для моторов и сервоприводов
#define PWM_MIN 1000                 // Минимальное значение PWM (полная остановка)
#define PWM_CENTER 1500              // Центральное значение PWM (нейтраль/покой)
#define PWM_MAX 2000                 // Максимальное значение PWM (полная мощность)
#define PWM_RANGE (PWM_MAX - PWM_MIN) // Диапазон PWM (1000)


// Структуры данных
// Фильтр Калмана
struct KalmanFilter {
  float x;    // Текущее состояние
  float P;    // Ковариация ошибки
  float K;    // Коэффициент Калмана
  float Q;    // Шум процесса
  float R;    // Шум измерений
};

// PWM значения для моторов (8 моторов)
struct MotorPWMValues {
  int motor0_pwm;    // Мотор 0
  int motor1_pwm;    // Мотор 1
  int motor2_pwm;    // Мотор 2
  int motor3_pwm;    // Мотор 3
  int motor4_pwm;    // Мотор 4
  int motor5_pwm;    // Мотор 5
  int motor6_pwm;    // Мотор 6
  int motor7_pwm;    // Мотор 7
};

// Объединенные команды ROV (моторы + полезная нагрузка)
struct ROVCommands {
  // PWM значения моторов
  int motor0_pwm;    // Мотор 0
  int motor1_pwm;    // Мотор 1
  int motor2_pwm;    // Мотор 2
  int motor3_pwm;    // Мотор 3
  int motor4_pwm;    // Мотор 4
  int motor5_pwm;    // Мотор 5
  int motor6_pwm;    // Мотор 6
  int motor7_pwm;    // Мотор 7
  
  // Полезная нагрузка
  int camera_servo;  // Позиция камеры
  int manipulator;    // Позиция манипулятора
  int light;          // Уровень освещения
};

// Структура для входных данных джойстика
struct JoystickData {
  float linear_x;    // -1.0 до 1.0
  float linear_y;    // -1.0 до 1.0
  float linear_z;    // -1.0 до 1.0
  float rotate_x;    // -1.0 до 1.0
  float rotate_y;    // -1.0 до 1.0
  float rotate_z;    // -1.0 до 1.0
  int servo_cam;     // -1, 0, 1
  int gripper;       // -1, 0, 1
  int led;           // 0, 1
};

// Конфигурация кинематики
struct KinematicsConfig {
  int motor_count;           // Количество моторов (3, 4)
  bool reverse_motor[8];     // Реверс для каждого мотора
};

// ВАЛИДАЦИОННЫЕ ФУНКЦИИ

// Валидация конфигурации фильтра Калмана
inline bool isValidKalmanConfig(float q, float r, float p_init) {
    return (q >= MIN_KALMAN_Q && q <= MAX_KALMAN_Q &&
            r >= MIN_KALMAN_R && r <= MAX_KALMAN_R &&
            p_init >= MIN_KALMAN_P_INIT && p_init <= MAX_KALMAN_P_INIT);
}

// Валидация конфигурации кинематики
inline bool isValidKinematicsConfig(const KinematicsConfig& config) {
    return (config.motor_count >= 3 && config.motor_count <= 4);
}

// Валидация PWM значений
inline bool isValidPWMValue(int pwm) {
    return (pwm >= PWM_MIN && pwm <= PWM_MAX);
}

// Валидация ADC значений
inline bool isValidADCValue(int adc) {
    return (adc >= 0 && adc <= ADC_MAX_VALUE);
}

// Валидация конфигурации фильтра Калмана
inline bool isValidKalmanConfig(const KalmanFilter& config) {
    return (config.Q >= MIN_KALMAN_Q && config.Q <= MAX_KALMAN_Q &&
            config.R >= MIN_KALMAN_R && config.R <= MAX_KALMAN_R);
}


#endif // CONFIG_H
