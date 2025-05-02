// The PID 2WD Car
#include <Wire.h>
#include <LedControl.h>

int addressMPU = 0x68; //Endereço do sensor
int accCalibrationValue = 1000; //Valor de calibração do sensor

/*** Valores do PID ***/
float kp = 15; //Ganho do P (15)
float ki = 1.5; //Ganho do I (1.5)
float kd = 30; //Ganho do D (30)
float turningSpeed = 30; //Velocidade de rotação (20)
float maxTargetSpeed = 150; //Velocidade maxima (100)

/*** Variaveis globais ***/
byte start, receivedByte/*, low_bat*/;
int leftMotor, throttleLeftMotor, throttleCounterLeftMotor, throttleLeftMotorMemory;
int rightMotor, throttleRightMotor, throttleCounterRightMotor, throttleRightMotorMemory;
//int battery_voltage;
int receiveCounter;
int pitchDataRaw, yawDataRaw, accelerometerDataRaw;
long pitchCalibrationValue, yawCalibrationValue;
unsigned long loopTimer;
float angleGyro, angleAcc, angle, selfBalancePidSetPoint;
float errorTemp, errorSum, setPoint, gyroInput, pidOutput, lastDerivateError;
float pidOutputLeft, pidOutputRight;
LedControl lc = LedControl(12,11,10,1);

/*** Inicio e configuração ***/
void setup(){
  //Serial.begin(9600);//Start the serial port at 9600 kbps
  lc.shutdown(0,false); //Colocar o módulo em power-seving mode
  lc.setIntensity(0,8); //Setar o brilho - brilho médio
  Wire.begin();
  TWBR = 12;//I2C clock 400kHz

  /*** Para criar um pulso variável para controlar
   *** os motores de passo é criado um timer que
   *** irá executar um trecho de código (sub-rotina)
   *** a cada 70us called TIMER2_COMPA_vect ***/
  TCCR2A = 0;//Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;//Make sure that the TCCR2A register is set to zero
  TIMSK2 |= (1 << OCIE2A);//Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);//Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 139;//The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);//Set counter 2 to CTC (clear timer on compare) mode
  
  //Ligar/acordar MPU-6050
  Wire.beginTransmission(addressMPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  //Define a escala completa do giroscópio para +/- 250 graus por segundo
  Wire.beginTransmission(addressMPU);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();
  //Define a escala completa do acelerômetro para +/- 4g
  Wire.beginTransmission(addressMPU)
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission();
  //Definir alguma filtragem para melhorar os dados brutos
  Wire.beginTransmission(addressMPU);
  Wire.write(0x1A);
  Wire.write(0x03)
  Wire.endTransmission();

  pinMode(2, OUTPUT);//Porta do stepDriver do Motor1
  pinMode(3, OUTPUT);//Porta de direção do stepDriver do Motor1
  pinMode(4, OUTPUT);//Porta do stepDriver do Motor2
  pinMode(5, OUTPUT);//Porta de direção do stepDriver do Motor2
  pinMode(13, OUTPUT);//Porta do Led

  CalibrationPID();

  loopTimer = micros() + 4000;
}

/*** Update loop ***/
void loop(){
  if(Serial.available()){
    receivedByte = Serial.read();
    receiveCounter = 0;
  }
  if(receiveCounter <= 25)receiveCounter ++;
  else receivedByte = 0x00;
 
  /*** Angle calculations ***/
  Wire.beginTransmission(addressMPU);
  Wire.write(0x3F);
  Wire.endTransmission();
  Wire.requestFrom(addressMPU, 2);
  accelerometerDataRaw = Wire.read()<<8|Wire.read();
  accelerometerDataRaw += accCalibrationValue;
  if(accelerometerDataRaw > 8200)accelerometerDataRaw = 8200;
  if(accelerometerDataRaw < -8200)accelerometerDataRaw = -8200;

  angleAcc = asin((float)accelerometerDataRaw/8200.0)* 57.296;

  if(start == 0 && angleAcc > -0.5 && angleAcc < 0.5){
    angleGyro = angleAcc;
    start = 1;
  }
  
  Wire.beginTransmission(addressMPU);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(addressMPU, 4);
  yawDataRaw = Wire.read()<<8|Wire.read();
  pitchDataRaw = Wire.read()<<8|Wire.read();
  
  pitchDataRaw -= pitchCalibrationValue;
  angleGyro += pitchDataRaw * 0.000031;
  
  /*** MPU-6050 offset compensation ***/
  yawDataRaw -= yawCalibrationValue;
  //angleGyro -= yawDataRaw * 0.0000003;
  angleGyro = angleGyro * 0.9996 + angleAcc * 0.0004;
  
  /*** Print do angulo na matrix de led ***/  
  int ledY = map(angleGyro, -10, 10, 0, 7);
  Serial.println(ledY);
  lc.clearDisplay(0); // limpar display
  for (int i = 0; i < 8; i++){
    lc.setLed(0, i, ledY, true);
  }

  /*** PID ***/
  errorTemp = angleGyro - selfBalancePidSetPoint - setPoint;
  if(pidOutput > 10 || pidOutput < -10)errorTemp += pidOutput * 0.015 ;
  errorSum += ki * errorTemp;
  if(errorSum > 400)errorSum = 400;
  else if(errorSum < -400)errorSum = -400;
  pidOutput = kp * errorTemp + errorSum + kd * (errorTemp - lastDerivateError);
  //Evitar overflow do pid
  if(pidOutput > 400)pidOutput = 400;
  else if(pidOutput < -400)pidOutput = -400;
  lastDerivateError = errorTemp;
  if(pidOutput < 5 && pidOutput > -5)pidOutput = 0;
  if(angleGyro > 30 || angleGyro < -30 || start == 0){
    pidOutput = 0;
    errorSum = 0;
    start = 0;
    selfBalancePidSetPoint = 0;
    CalibrationPID();
  }

  /*** Control calculations ***/
  pidOutputLeft = pidOutput;
  pidOutputRight = pidOutput;

  if(receivedByte & B00000001){
    pidOutputLeft += turningSpeed;
    pidOutputRight -= turningSpeed;
  }
  if(receivedByte & B00000010){
    pidOutputLeft -= turningSpeed;
    pidOutputRight += turningSpeed;
  }
  if(receivedByte & B00000100){
    if(setPoint > -2.5)setPoint -= 0.05;
    if(pidOutput > maxTargetSpeed * -1)setPoint -= 0.005;
  }
  if(receivedByte & B00001000){
    if(setPoint < 2.5)setPoint += 0.05;
    if(pidOutput < maxTargetSpeed)setPoint += 0.005;
  }   
  if(!(receivedByte & B00001100)){
    if(setPoint > 0.5)setPoint -=0.05;
    else if(setPoint < -0.5)setPoint +=0.05;
    else setPoint = 0;
  }
  
  //O ponto de auto balanceamento é ajustado quando não há movimento para frente ou para trás do transmissor. Desta forma, o robô sempre encontrará seu ponto de equilíbrio
  if(setPoint == 0){
    if(pidOutput < 0)selfBalancePidSetPoint += 0.0015;
    if(pidOutput > 0)selfBalancePidSetPoint -= 0.0015;
  }

  /*** Motor pulse calculations ***/
  //Para compensar o comportamento não linear dos motores de passo, são necessários os seguintes cálculos para obter um comportamento de velocidade linear.
  if(pidOutputLeft > 0)pidOutputLeft = 405 - (1/(pidOutputLeft + 9)) * 5500;
  else if(pidOutputLeft < 0)pidOutputLeft = -405 - (1/(pidOutputLeft - 9)) * 5500;
  if(pidOutputRight > 0)pidOutputRight = 405 - (1/(pidOutputRight + 9)) * 5500;
  else if(pidOutputRight < 0)pidOutputRight = -405 - (1/(pidOutputRight - 9)) * 5500;
  //Calcule o tempo de pulso necessário para os controladores de motor de passo esquerdo e direito
  if(pidOutputLeft > 0)leftMotor = 400 - pidOutputLeft;
  else if(pidOutputLeft < 0)leftMotor = -400 - pidOutputLeft;
  else leftMotor = 0;
  if(pidOutputRight > 0)rightMotor = 400 - pidOutputRight;
  else if(pidOutputRight < 0)rightMotor = -400 - pidOutputRight;
  else rightMotor = 0;
  //Copie o tempo de pulso para as variáveis ​​do acelerador para que a sub-rotina de interrupção possa usá-las
  throttleLeftMotor = leftMotor;
  throttleRightMotor = rightMotor;
  
  while(loopTimer > micros());
  loopTimer += 4000;
}

void CalibrationPID(){
    for(receiveCounter = 0; receiveCounter < 500; receiveCounter++){
    if(receiveCounter % 15 == 0)digitalWrite(13, !digitalRead(13));//Change the state of the LED every 15 loops to make the LED blink fast
    Wire.beginTransmission(addressMPU);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(addressMPU, 4);
    /*** Faz um "pedido" para ler até 14 registradores
     *** 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
     *** 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
     *** 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
     *** 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L) -> (dividir por / 340.00 + 36.53)
     *** 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
     *** 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
     *** 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L) ***/
    yawCalibrationValue += Wire.read()<<8|Wire.read();
    pitchCalibrationValue += Wire.read()<<8|Wire.read();
    delayMicroseconds(3700);
  }
  pitchCalibrationValue /= 500;
  yawCalibrationValue /= 500;
}

//Interrupt routine  TIMER2_COMPA_vect
ISR(TIMER2_COMPA_vect){
  //Esquerdo
  throttleCounterLeftMotor ++;
  if(throttleCounterLeftMotor > throttleLeftMotorMemory){
    throttleCounterLeftMotor = 0;
    throttleLeftMotorMemory = throttleLeftMotor;
    if(throttleLeftMotorMemory < 0){
      PORTD &= 0b11110111;
      throttleLeftMotorMemory *= -1;
    }
    else PORTD |= 0b00001000;
  }
  else if(throttleCounterLeftMotor == 1)PORTD |= 0b00000100;
  else if(throttleCounterLeftMotor == 2)PORTD &= 0b11111011;
  
  //Direito
  throttleCounterRightMotor ++;
  if(throttleCounterRightMotor > throttleRightMotorMemory){
    throttleCounterRightMotor = 0;
    throttleRightMotorMemory = throttleRightMotor;
    if(throttleRightMotorMemory < 0){
      PORTD |= 0b00100000;
      throttleRightMotorMemory *= -1;
    }
    else PORTD &= 0b11011111;
  }
  else if(throttleCounterRightMotor == 1)PORTD |= 0b00010000;
  else if(throttleCounterRightMotor == 2)PORTD &= 0b11101111;
}