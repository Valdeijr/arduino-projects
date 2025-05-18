#include <Wire.h>
#include <MPU6050_light.h>

// MPU6050 no barramento I2C
MPU6050 mpu(Wire);

void setup() {
  Wire.begin();
  Serial.begin(115200);

  // Inicia o MPU6050
  byte status = mpu.begin();
  if (status != 0) {
    while (true) {
      Serial.println("MPU6050 não inicializado");
      delay(2000);
    }
  }

  // Calibra o sensor
  mpu.calcOffsets(true, true);
  delay(100);
}

void loop() {
  mpu.update();

  // Captura valores e converte para formato "bruto"
  int16_t ax = mpu.getAccX() * 16384;
  int16_t ay = mpu.getAccY() * 16384;
  int16_t az = mpu.getAccZ() * 16384;

  // Envia 6 bytes: ax, ay, az (2 bytes cada, little endian)
  Serial.write((uint8_t *)&ax, 2);
  Serial.write((uint8_t *)&ay, 2);
  Serial.write((uint8_t *)&az, 2);

  delayMicroseconds(3500);  // Taxa de ~285 Hz
}
