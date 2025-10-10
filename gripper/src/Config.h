#ifndef CONFIG_H
#define CONFIG_H

// Пины для измерения импульсов и тока
#define PULSE_INPUT_PIN PA2
#define I2C_SDA_PIN PB7
#define I2C_SCL_PIN PB6

// Пины для драйвера двигателя L9110s (только 2 пина управления)
#define MOTOR_IA_PIN PA0
#define MOTOR_IB_PIN PA1

// Диапазон PWM сигнала
#define PWM_MIN_US 900
#define PWM_MAX_US 2400

// Мертвая зона вокруг нейтрали
#define PWM_DEADZONE_MIN_US 1400
#define PWM_DEADZONE_MAX_US 1600

// Нейтральное значение PWM
#define PWM_NEUTRAL_US 1500

// Скорости двигателя
#define MOTOR_SPEED_FORWARD 255
#define MOTOR_SPEED_REVERSE -255
#define MOTOR_SPEED_STOP 0

// Настройки плавного старта
#define SMOOTH_START_ENABLED true
#define SMOOTH_START_STEP_MS 10        // Интервал между шагами (мс) - уменьшен для более плавного старта
#define SMOOTH_START_STEP_SIZE 8       // Размер шага скорости - уменьшен для более плавного старта
#define SMOOTH_START_MIN_SPEED 10      // Минимальная скорость для плавного старта - уменьшена

// Интервал измерения тока (мс)
#define CURRENT_MEASUREMENT_INTERVAL 50

// Мертвая зона для тока (мА)
#define CURRENT_DEADZONE_MA 0.5

// Количество образцов для калибровки холостого тока
#define IDLE_CURRENT_SAMPLES 20

// Интервал между сэмплами холостого тока (мс)
#define IDLE_CURRENT_SAMPLE_INTERVAL_MS 100

// Порог защиты от перегрузки (мА)
#define CURRENT_PROTECTION_THRESHOLD_MA 10

// Задержка измерения тока после старта двигателя (мс)
#define MOTOR_START_DELAY_MS 1000  // 2 секунды для избежания пусковых токов

// Диапазон валидных импульсов (мкс)
#define PULSE_MIN_US 500
#define PULSE_MAX_US 3000

// Порог для обнаружения слишком больших невалидных импульсов (мкс)
#define PULSE_MAX_INVALID_US 100000

// Интервал обновления двигателя (мс)
#define MOTOR_UPDATE_INTERVAL_MS 50

// Интервал вывода данных (мс)
#define DATA_PRINT_INTERVAL_MS 100

// Скорость последовательного порта
#define SERIAL_BAUD_RATE 115200

#endif // CONFIG_H
