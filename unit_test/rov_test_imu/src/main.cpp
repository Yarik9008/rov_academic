#include <Arduino.h>
#include <Adafruit_BNO08x.h>

// Создаем объект датчика BNO085
Adafruit_BNO08x bno08x;

// Структура для хранения данных IMU
struct IMUData {
  float quatI, quatJ, quatK, quatReal;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float magX, magY, magZ;
  float roll, pitch, yaw; // Углы Эйлера в градусах
};

IMUData imuData;

// Объявление функций
void setReports();
bool readIMUData();
void printIMUData();
void quaternionToEuler(float qw, float qx, float qy, float qz, float &roll, float &pitch, float &yaw);

// Функция для сканирования I2C устройств
void scanI2C() {
  Serial.println("Сканирование I2C устройств...");
  Wire.begin();
  
  int deviceCount = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Найдено устройство по адресу 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("Устройства I2C не найдены!");
  } else {
    Serial.print("Найдено устройств: ");
    Serial.println(deviceCount);
  }
}

// Функция для инициализации датчика
bool initBNO085() {
  Serial.println("Попытка инициализации BNO085 по адресу 0x4B...");
  
  // Сначала сканируем I2C шину
  scanI2C();
  
  // Пробуем подключиться к адресу 0x4B несколько раз
  const int maxAttempts = 5;
  const int delayMs = 500;
  
  for (int attempt = 1; attempt <= maxAttempts; attempt++) {
    Serial.print("Попытка ");
    Serial.print(attempt);
    Serial.print(" из ");
    Serial.print(maxAttempts);
    Serial.println(" подключения к адресу 0x4B...");
    
    if (bno08x.begin_I2C(0x4B)) {
      Serial.println("BNO085 инициализирован успешно по адресу 0x4B!");
      
      // Настройка отчетов датчика
      setReports();
      
      return true;
    }
    
    if (attempt < maxAttempts) {
      Serial.print("Ошибка подключения. Повтор через ");
      Serial.print(delayMs);
      Serial.println("мс...");
      delay(delayMs);
    }
  }
  
  Serial.println("Ошибка инициализации BNO085 по адресу 0x4B после всех попыток!");
  return false;
}

// Функция для настройки отчетов датчика
void setReports() {
  Serial.println("Настройка отчетов BNO085...");
  
  // Включаем отчеты для кватернионов (ориентация)
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) {
    Serial.println("Ошибка включения отчета кватернионов");
  }
  
  // Включаем отчеты для акселерометра
  if (!bno08x.enableReport(SH2_ACCELEROMETER, 10000)) {
    Serial.println("Ошибка включения отчета акселерометра");
  }
  
  // Включаем отчеты для гироскопа
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)) {
    Serial.println("Ошибка включения отчета гироскопа");
  }
  
  // Включаем отчеты для магнитометра
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 10000)) {
    Serial.println("Ошибка включения отчета магнитометра");
  }
  
  
  Serial.println("Отчеты настроены успешно!");
}

// Функция для преобразования кватернионов в углы Эйлера
void quaternionToEuler(float qw, float qx, float qy, float qz, float &roll, float &pitch, float &yaw) {
  // Roll (x-axis rotation)
  float sinr_cosp = 2 * (qw * qx + qy * qz);
  float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
  roll = atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  float sinp = 2 * (qw * qy - qz * qx);
  if (abs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // Yaw (z-axis rotation)
  float siny_cosp = 2 * (qw * qz + qx * qy);
  float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
  yaw = atan2(siny_cosp, cosy_cosp);

  // Преобразуем в градусы
  roll = roll * 180.0 / M_PI;
  pitch = pitch * 180.0 / M_PI;
  yaw = yaw * 180.0 / M_PI;
}

// Функция для чтения данных с датчика
bool readIMUData() {
  sh2_SensorValue_t sensorValue;
  
  // Проверяем наличие новых данных
  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ROTATION_VECTOR:
        imuData.quatI = sensorValue.un.rotationVector.i;
        imuData.quatJ = sensorValue.un.rotationVector.j;
        imuData.quatK = sensorValue.un.rotationVector.k;
        imuData.quatReal = sensorValue.un.rotationVector.real;
        
        // Вычисляем углы Эйлера из кватернионов
        quaternionToEuler(imuData.quatReal, imuData.quatI, imuData.quatJ, imuData.quatK, 
                         imuData.roll, imuData.pitch, imuData.yaw);
        break;
        
      case SH2_ACCELEROMETER:
        imuData.accelX = sensorValue.un.accelerometer.x;
        imuData.accelY = sensorValue.un.accelerometer.y;
        imuData.accelZ = sensorValue.un.accelerometer.z;
        break;
        
      case SH2_GYROSCOPE_CALIBRATED:
        imuData.gyroX = sensorValue.un.gyroscope.x;
        imuData.gyroY = sensorValue.un.gyroscope.y;
        imuData.gyroZ = sensorValue.un.gyroscope.z;
        break;
        
      case SH2_MAGNETIC_FIELD_CALIBRATED:
        imuData.magX = sensorValue.un.magneticField.x;
        imuData.magY = sensorValue.un.magneticField.y;
        imuData.magZ = sensorValue.un.magneticField.z;
        break;
        
    }
    return true;
  }
  return false;
}

// Функция для вывода данных в Serial (одна строчка)
void printIMUData() {
  Serial.print("E:");
  Serial.print(imuData.roll, 1);
  Serial.print(",");
  Serial.print(imuData.pitch, 1);
  Serial.print(",");
  Serial.print(imuData.yaw, 1);
  
  Serial.print(" A:");
  Serial.print(imuData.accelX, 2);
  Serial.print(",");
  Serial.print(imuData.accelY, 2);
  Serial.print(",");
  Serial.print(imuData.accelZ, 2);
  
  Serial.print(" G:");
  Serial.print(imuData.gyroX, 2);
  Serial.print(",");
  Serial.print(imuData.gyroY, 2);
  Serial.print(",");
  Serial.print(imuData.gyroZ, 2);
  
  Serial.print(" M:");
  Serial.print(imuData.magX, 1);
  Serial.print(",");
  Serial.print(imuData.magY, 1);
  Serial.print(",");
  Serial.print(imuData.magZ, 1);
  
  Serial.println();
}

void setup() {
  // Инициализация Serial порта
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("Инициализация системы ROV IMU...");
  
  // Инициализация I2C шины
  Wire.begin();
  Wire.setClock(400000); // Устанавливаем частоту I2C на 400kHz
  
  Serial.println("I2C шина инициализирована");
  
  // Инициализация датчика BNO085
  if (!initBNO085()) {
    Serial.println("Критическая ошибка: не удалось инициализировать BNO085!");
    Serial.println("Проверьте подключение датчика:");
    Serial.println("- VCC -> 3.3V");
    Serial.println("- GND -> GND");
    Serial.println("- SDA -> PB7");
    Serial.println("- SCL -> PB6");
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println("Система готова к работе!");
  delay(1000);
}

void loop() {
  // Читаем данные с датчика
  readIMUData();
  
  // Выводим данные 10 раз в секунду (каждые 100мс)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 50) {
    printIMUData();
    lastPrint = millis();
  }
  
}