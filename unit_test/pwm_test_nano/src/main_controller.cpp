#include <Arduino.h>
#include <Servo.h>

Servo servo;
const int SERVO_PIN = 3;

// Параметры для агрессивного движения
int MIN_POSITION = 1200;    // Минимальная позиция
int MAX_POSITION = 1800;    // Максимальная позиция
int CENTER_POSITION = 1500; // Центральная позиция
int STEP_DELAY = 15;        // Задержка между шагами (мс)
int STEP_SIZE = 20;         // Размер шага для движения

int currentPosition = CENTER_POSITION;
int targetPosition = CENTER_POSITION;
bool isMoving = false;

void setup() {
  Serial.begin(115200);
  
  // Подключение сервопривода (1000-2000 мкс)
  servo.attach(SERVO_PIN, 1000, 2000);
  
  servo.writeMicroseconds(CENTER_POSITION);  // Центр
  delay(2000);
  
  Serial.println("Servo Controller Ready");
  Serial.println("Commands:");
  Serial.println("POS:XXXX - Set position (1200-1800)");
  Serial.println("SETTINGS:SPEED:DELAY - Set movement parameters");
  Serial.println("STATUS - Get current status");
}

void loop() {
  // Обработка команд от Python GUI
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
  }
  
  // Плавное движение к целевой позиции
  if (isMoving && currentPosition != targetPosition) {
    if (currentPosition < targetPosition) {
      currentPosition += STEP_SIZE;
      if (currentPosition > targetPosition) {
        currentPosition = targetPosition;
      }
    } else {
      currentPosition -= STEP_SIZE;
      if (currentPosition < targetPosition) {
        currentPosition = targetPosition;
      }
    }
    
    servo.writeMicroseconds(currentPosition);
    delay(STEP_DELAY);
    
    // Проверяем, достигли ли целевой позиции
    if (currentPosition == targetPosition) {
      isMoving = false;
      Serial.println("POSITION_REACHED:" + String(currentPosition));
    }
  }
}

void processCommand(String command) {
  if (command.startsWith("POS:")) {
    int pos = command.substring(4).toInt();
    if (pos >= MIN_POSITION && pos <= MAX_POSITION) {
      targetPosition = pos;
      isMoving = true;
      Serial.println("MOVING_TO:" + String(pos));
    } else {
      Serial.println("ERROR:Invalid position " + String(pos));
    }
  }
  else if (command.startsWith("SETTINGS:")) {
    int firstColon = command.indexOf(':', 9);
    if (firstColon > 0) {
      int speed = command.substring(9, firstColon).toInt();
      int delay = command.substring(firstColon + 1).toInt();
      
      if (speed >= 1 && speed <= 50) {
        STEP_SIZE = speed;
      }
      if (delay >= 1 && delay <= 100) {
        STEP_DELAY = delay;
      }
      
      Serial.println("SETTINGS_APPLIED:Speed=" + String(STEP_SIZE) + ",Delay=" + String(STEP_DELAY));
    }
  }
  else if (command == "STATUS") {
    Serial.println("STATUS:Position=" + String(currentPosition) + 
                   ",Target=" + String(targetPosition) + 
                   ",Moving=" + String(isMoving ? "true" : "false") +
                   ",Speed=" + String(STEP_SIZE) +
                   ",Delay=" + String(STEP_DELAY));
  }
  else if (command == "CENTER") {
    targetPosition = CENTER_POSITION;
    isMoving = true;
    Serial.println("MOVING_TO_CENTER");
  }
  else if (command == "MIN") {
    targetPosition = MIN_POSITION;
    isMoving = true;
    Serial.println("MOVING_TO_MIN");
  }
  else if (command == "MAX") {
    targetPosition = MAX_POSITION;
    isMoving = true;
    Serial.println("MOVING_TO_MAX");
  }
  else {
    Serial.println("ERROR:Unknown command " + command);
  }
}
