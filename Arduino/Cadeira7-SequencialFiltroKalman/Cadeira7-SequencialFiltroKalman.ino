double angleDistance(int alpha, int beta);
void readEncSpeed(void);
void stop1(void);
void leitura(void);
void goright(int velocity, int angle=NULL);
void goleft(int velocity, int angle=NULL);
void gobackward(int velocity, double distance=NULL);
void goforward(int velocity, double distance=NULL);
void setSpeedMotors(double velocidade1, double velocidade2, double Accel=NULL);

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop);
uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes); 
//Arduino Mega and Leonardo chips only support some pins for receiving data back from the RoboClaw
//This is because only some pins of these boards support PCINT interrupts or are UART receivers.
//Mega: 0,10,11,12,13,14,15,17,19,50,51,52,53,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15
//Leonardo: 0,8,9,10,11

//Arduino Due currently does not support SoftwareSerial. Only hardware uarts can be used, pins 0/1, 14/15, 16/17 or 18/19.

//Note: Most Arduinos do not support higher baudrates rates than 115200.  Also the arduino hardware uarts generate 57600 and 115200 with a
//relatively large error which can cause communications problems.

//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include "RoboClaw.h"
//#include "BMSerial.h"
#include <SoftwareSerial.h>

//
//Roboclaw
//

//Roboclaw Address
#define address 0x80

//Setup communcaitions with roboclaw Use pins 10 and 11 with 10ms timeout
SoftwareSerial serial(10,11);  
RoboClaw roboclaw(&serial,10000);
int16_t erro;
int32_t enc1;
int32_t enc2;
int32_t speed1;
int32_t speed2;

double enc1metro=75941;
//double enc1volta=119227; //1 volta roda
//double enc1metro=119227; //1 volta roda

//
//PID
//

double angulo;

class PID {
  public:

    double error;
    double sample;
    double lastSample;
    double kP, kI, kD;
    double P, I, D;
    double pid;

    double setPoint;
    long lastProcess;

    PID(double _kP, double _kI, double _kD) {
      kP = _kP;
      kI = _kI;
      kD = _kD;
    }

    void addNewSample(double _sample) {
      sample = _sample;
    }

    void setSetPoint(double _setPoint) {
      setPoint = _setPoint;
      P = 0;
      I = 0;
      D = 0;
      lastSample = 0;
    }
    double returnError() {
      return angleDistance(setPoint, sample);
    }

    double process() {
      // Implementação P ID
      error = angleDistance(setPoint, sample);
      Serial.print(" erro  ");
      Serial.print(error);
      float deltaTime = (millis() - lastProcess) / 1000.0;
      lastProcess = millis();

      //P
      P = error * kP;

      //I
      I = I + (error * kI) * deltaTime;

      //D
      D = (lastSample - sample) * kD / deltaTime;
      lastSample = sample;

      // Soma tudo
      pid = P + I + D;
      Serial.print(" Controle  ");
      Serial.println(pid);
      return pid;
    }
};
PID meuPid(1.0, 0.01, 0.02);

//
//Bluetooth
//

char cmd;
char param;
char param1;
String param2;
int distanceBT;
int angleBT;
int slideBT; //varia de 0 a 50
#define max_char 10
char message[max_char];    // stores you message
char r_char;               // reads each character
byte index = 0;            // defines the position into your array
int i;
bool messageEnd = false;
double responceTime = 0;
double inicialResponceTime = 0;
bool flagTime = true;


//
//Variáveis
//
int accelGlobal = 10000;
int velocityGlobal = 30000;


//
// MPU6050 com Kalman
//
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data



void setup() {
  Serial1.begin(9600);//Bluetooth
  
  roboclaw.begin(38400); //Open  Serial1 and roboclaw serial ports

  Serial.begin(115200); // SERIAL
  Wire.begin(); //MPU6050
  #if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif
  //initSensors();
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();

  //Set PID Coefficients
  float P1 = 14.27100;
  float I1 = 3.56775;
  float D1 = 0.00000;
  float qpps1 = 115796;
  float P2 = 18.41226;
  float I2 = 5.38075;
  float D2 = 0.00000;
  float qpps2 = 115972;
  roboclaw.SetM1VelocityPID(address, P1, I1, D1, qpps1);
  roboclaw.SetM2VelocityPID(address, P2, I2, D2, qpps2);

  // Aguarda a resposta dos sensores
  delay(1500);
  //Serial1.println("Cadeira started");

  stop1();

  double tempo = millis(); //Tempo para estabilizar leituras
  while (millis() - tempo < 10000) {
    //Serial.println(millis()-tempo); // Imprime o tempo passado
    leitura();
  }
  //goforward(10000, 30000); delay(5000);
  //goleft(10000, 90); delay(5000);
  //goleft(10000, 90); delay(5000);
  //goforward(10000, 30000); delay(5000);
  //goright(10000, 90); delay(5000);
  //goright(10000, 90);
  //goforward(12000,76000); //76000 é um quadrado da sala
  //delay(100);
  //gobackward(12000,76000); //76000 é um quadrado da sala
  
}
// Main loop
void loop(){

}

void serialEvent1(){
  Serial.println("Recebeu");
  //while is reading the message
  inicialResponceTime = millis();
  flagTime = true;
  while (Serial1.available() > 0) {
    //the message can have up to 100 characters
    if (index < (max_char - 1))
    {
      r_char = Serial1.read();      // Reads a character
      message[index] = r_char;     // Stores the character in message array
      index++;                     // Increment position
      message[index] = '\0';       // Delete the last position
      if (r_char == 'z') {
        messageEnd = true;
      }
    }
  }
  if (messageEnd) {
    Serial.println(message);
    switch (message[0]) {
      //COMANDO POR BOTOES
      case '1':
        {
          Serial.println("Comando1");
          // read the parameter byte
          param = message[1];
          Serial.print("Param = ");
          Serial.println(param);
          switch (param)
          {
            case 'f':
              Serial.println("Frente");
              //goforward(30000,100);
              goforward(velocityGlobal );
              break;
            case 'd':
              Serial.println("Direita");
              //goright(30000, 90);
              goright(velocityGlobal/2 );
              break;
            case 'e':
              Serial.println("Esquerda");
              //goleft(30000, 90);
              goleft(velocityGlobal/2 );
              break;
            case 'a':
              Serial.println("Trás");
              //gobackward(30000, 100);
			         gobackward(velocityGlobal );
              break;
            case 'p':
              Serial.println("Parar");
              stop1();
              break;
            default: break; // do nothing
          }// switch (param)
        } break; // switch (cmd) case 1
      //COMANDO DE VOZ
      case '2':
        {
          param = message[1];
          Serial.print("Param = ");
          Serial.println(param);
          switch (param)
          {
            case 'f':
              Serial.println("Frente");
              for (int a = 2; a < (index - 1); a++) {
                param2.concat(message[a]);
              }
                distanceBT = param2.toInt();
                Serial.print("Param2 = ");
                Serial.println(distanceBT);
                if (distanceBT ==0){
                  goforward(velocityGlobal);
                }else{
                  goforward(velocityGlobal , distanceBT);
                }
              break;
            case 'd':
              Serial.println("Direita");
              for (int a = 2; a < (index - 1); a++) {
                param2.concat(message[a]);
              }
                angleBT = param2.toInt();
                Serial.print("Param2 = ");
                Serial.println(angleBT);
                if (distanceBT ==0){
                  goright(velocityGlobal/2 );
                }
                else{
                  goright(velocityGlobal/2 , angleBT);
                }
                
              }
              break;
            case 'e':
              Serial.println("Esquerda");
              for (int a = 2; a < (index - 1); a++) {
                param2.concat(message[a]);
              }
                angleBT = param2.toInt();
                Serial.print("Param2 = ");
                Serial.println(angleBT);
                if (distanceBT ==0){
                  goleft(velocityGlobal/2);
                }
                else{
                  goleft(velocityGlobal/2 , angleBT);
                }
                
              
              break;
            case 'a':
              Serial.println("Tras");
              for (int a = 2; a < (index - 1); a++) {
                param2.concat(message[a]);
              }
                distanceBT = param2.toInt();
                Serial.print("Param2 = ");
                Serial.println(distanceBT);
                if (distanceBT ==0){
                  gobackward(velocityGlobal);
                }
                else{
                  gobackward(velocityGlobal , distanceBT);
                }
              
              break;
              case 'p':
              Serial.println("Parar");
              stop1();
              break;
            default: break; // do nothing
          } 
        }break; // switch (cmd) case 2
       //COMANDO POR SLIDER
       case '3':
       {
          param = message[1]; // d ou e
          Serial.print("Param = ");
          Serial.println(param);
          switch (param){
           case 'd':
              param1 = message[2]; // F ou T ou 0
              Serial.print("Param2 = ");
              Serial.println(param1);
              if(param1=='0'){
                  Serial.println("Direita");
                  for (int a = 2; a < (index - 1); a++) {
                    param2.concat(message[a]);
                  }
                  slideBT = param2.toInt();
                  Serial.print("Param2 = ");
                  Serial.println(slideBT);
                  if (slideBT < 5){
                    //int velocidade = (velocityGlobal/4);
                    //setSpeedMotors( velocidade, -velocidade);
                    stop1();
                  }else{
                     int velocidade = 100*slideBT;
                    setSpeedMotors( (velocityGlobal/4)+velocidade, -(velocityGlobal/4));
                  }
              }else if(param1=='F'){
                 Serial.println("Direita Frente");
                  for (int a = 3; a < (index - 1); a++) {
                    param2.concat(message[a]);
                  }
                  slideBT = param2.toInt();
                  Serial.print("Param2 = ");
                  Serial.println(slideBT);
                  if (slideBT < 5){
                    int velocidade = (velocityGlobal/4);
                    setSpeedMotors( velocidade, velocidade);
                  }else{
                    int velocidade = 100*slideBT;
                    setSpeedMotors( (velocityGlobal/4)+velocidade, (velocityGlobal/4)-velocidade);
                  }
              }else if(param1=='T'){
                  Serial.println("Direita Tras");
                  for (int a = 3; a < (index - 1); a++) {
                    param2.concat(message[a]);
                  }
                  slideBT = param2.toInt();
                  Serial.print("Param2 = ");
                  Serial.println(slideBT);
                  if (slideBT < 5){
                    int velocidade = (velocityGlobal/4);
                    setSpeedMotors( -velocidade, -velocidade);
                  }else{
                    int velocidade = 100*slideBT;
                    setSpeedMotors( -(velocityGlobal/4)-velocidade, -(velocityGlobal/4)+velocidade);
                  }
              }
            break;
            case 'e':
              param1 = message[2]; // F ou T ou 0
              Serial.print("Param2 = ");
              Serial.println(param1);
              if(param1=='0'){
                  Serial.println("Direita");
                  for (int a = 3; a < (index - 1); a++) {
                    param2.concat(message[a]);
                  }
                  slideBT = param2.toInt();
                  Serial.print("Param2 = ");
                  Serial.println(slideBT);
                  if (slideBT < 5){
                    //int velocidade = (velocityGlobal/4);
                    //setSpeedMotors( velocidade, -velocidade);
                    stop1();
                  }else{
                     int velocidade = 100*slideBT;
                    setSpeedMotors( -(velocityGlobal/4)-velocidade, (velocityGlobal/4));
                  }
              }else if(param1=='F'){
                 Serial.println("Direita Frente");
                  for (int a = 3; a < (index - 1); a++) {
                    param2.concat(message[a]);
                  }
                  slideBT = param2.toInt();
                  Serial.print("Param2 = ");
                  Serial.println(slideBT);
                  if (slideBT < 5){
                    int velocidade = (velocityGlobal/4);
                    setSpeedMotors( velocidade, velocidade);
                  }else{
                    int velocidade = 100*slideBT;
                    setSpeedMotors( (velocityGlobal/4)-velocidade, (velocityGlobal/4)+velocidade);
                  }
              }else if(param1=='T'){
                  Serial.println("Direita Tras");
                  for (int a = 3; a < (index - 1); a++) {
                    param2.concat(message[a]);
                  }
                  slideBT = param2.toInt();
                  Serial.print("Param2 = ");
                  Serial.println(slideBT);
                  if (slideBT < 50){
                    int velocidade = (velocityGlobal/4);
                    setSpeedMotors(-velocidade, -velocidade);
                  }else{
                    int velocidade = 100*slideBT;
                    setSpeedMotors( -(velocityGlobal/4)+velocidade, -(velocityGlobal/4)-velocidade);
                  }
              }
              break;
            default: break; // do nothing
          }
       }break; // switch (cmd) case 3
      default: break; // do nothing
    }
    messageEnd = false;
    param2 = "";
    for (i = 0; i < max_char; i++) {
      message[i] = '\0';
    }
    //resests the index
    index = 0;
  }
}

//Leitura Angulo
void leitura(void){
  //
  bool valid;
  erro = roboclaw.ReadError(address, &valid);
  if (valid && erro != 0){
    Serial.println(erro, HEX);
  }
  


  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

  #ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  #else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  #endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
  #if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
  #endif

  Serial.print(roll); Serial.print("\t");
  Serial.print(gyroXangle); Serial.print("\t");
  Serial.print(compAngleX); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

  Serial.print("\t");

  Serial.print(pitch); Serial.print("\t");
  Serial.print(gyroYangle); Serial.print("\t");
  Serial.print(compAngleY); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");

  Serial.print("\r\n");

    angulo = kalAngleX * 180 / M_PI;
    if (angulo < 0) {
      angulo = -angulo;
      angulo = map(angulo, 0, 180, 180, 0);
      angulo = angulo + 180;
    }
    //Serial.print(" Angulo Atual ");
    //Serial.println(angulo);
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

void goright(int velocity, int angle=NULL) {
  if(angle == NULL) {
      //setSpeedMotors(velocity,-velocity);
      setSpeedMotors(velocity,-velocity, accelGlobal);
  }else{
    //displayspeed();
    leitura();
    double anguloInicial = angulo;
    //Serial1.print("Angulo Inicial");
    //Serial1.println(anguloInicial);
    double angulofinal = anguloInicial + angle;
    if (angulofinal > 360) {
      angulofinal = angulofinal - 360;
    }
    //Serial1.print("AnguloFinal");
    //Serial1.println(angulofinal);
    meuPid.setSetPoint(angulofinal);
    meuPid.addNewSample(angulo);
    int controle = meuPid.process();
    double erro = abs(meuPid.returnError());
    double tempo = millis();
    while ((erro >= 1) && !(Serial1.available() > 0)) {
      int velocidade = abs(controle * 100);
      if (velocidade > velocity) {
        velocidade = velocity;
      }
      //Serial1.print("Virando direita Velocidade =");
      //Serial1.println(velocidade);
      setSpeedMotors( velocidade, -velocidade);
      leitura();
      meuPid.addNewSample(angulo);
      controle = meuPid.process();
      erro = abs(meuPid.returnError());
    }
    //Serial1.print("Angulo Atual");
    //Serial1.println(angulo);
    //Serial1.print("Angulo Girado");
    //Serial1.println(anguloInicial-angulo);
    stop1();
  }
}
void goleft(int velocity, int angle=NULL) {
  if(angle == NULL) {
      //setSpeedMotors(-velocity,velocity);
      setSpeedMotors(-velocity,velocity, accelGlobal/2);
  }else{
    //displayspeed();
    leitura();
    double anguloInicial = angulo;
    //Serial1.print("Angulo Inicial");
    //Serial1.println(anguloInicial);
    double angulofinal = anguloInicial - angle;
    if (angulofinal < 0) {
      angulofinal = angulofinal + 360;
    }
    //Serial1.print("AnguloFinal");
    //Serial1.println(angulofinal);
    meuPid.setSetPoint(angulofinal);
    meuPid.addNewSample(angulo);
    int controle = meuPid.process();
    double erro = abs(meuPid.returnError());

    double tempo = millis();
    while ((erro >= 1) && !(Serial1.available() > 0)) {
      int velocidade = abs(controle * 100); 
      if (velocidade > velocity) {
        velocidade = velocity;
      }
      //Serial1.print("Virando esquerda Velocidade =");
      //Serial1.println(velocidade);
      setSpeedMotors(-velocidade, velocidade);
      leitura();
      meuPid.addNewSample(angulo);
      controle = meuPid.process();
      erro = abs(meuPid.returnError());
    }
    //Serial1.print("Angulo Atual");
    //Serial1.println(angulo);
    //Serial1.print("Angulo Girado");
    //Serial1.println(anguloInicial-angulo);
    stop1();
  }
}

void goforward(int velocity, double distance = NULL){
  if(distance == NULL) {
      //setSpeedMotors( velocity, velocity);
      setSpeedMotors( velocity, velocity, accelGlobal/2);
  }else{
      leitura();
      double anguloInicial = angulo;
      //Serial1.print("Angulo Inicial");
      //Serial1.println(anguloInicial);
      //double erro = angleDistance(anguloInicial,angulo);
      meuPid.setSetPoint(anguloInicial);
      roboclaw.SetEncM1(address, 0);
      roboclaw.SetEncM2(address, 0);
      readEncSpeed();
      double enc1Inicial= enc1;
      double enc2Inicial= enc2;
      double distanciaInicial = ((enc1+enc2)/2)/(enc1metro/100);//cm
      
      //Serial1.print("Distancia Inicial ");
      //Serial1.println(distanciaInicial);
      
      double distanciaAtual = distanciaInicial;
      
      while ((distanciaAtual-distanciaInicial)<distance && !(Serial1.available() > 0)) {
        readEncSpeed();
        distanciaAtual = ((enc1+enc2)/2)/(enc1metro/100);//cm
        Serial.print(" Inicial");
        Serial.print(distanciaInicial);
        Serial.print(" ");
        Serial.print(" Atual");
        Serial.print(distanciaAtual);
        //erial1.print(" ");
        Serial.print("GoForward");
        //displayspeed();
        /*//Serial1.print("Angulo Inicial");
        //Serial1.println(anguloInicial);
        //Serial1.print("Angulo Passado");
        //Serial1.println(anguloAnterior);
        */
        leitura();
        meuPid.addNewSample(angulo);
        //erro = angleDistance(anguloInicial,angulo);
        // //Serial1.print("erro  ");
        // //Serial1.println(erro);
        int controle = meuPid.process();
        int velocidade = velocity - (abs(controle) * 100);
        //está pra esquerda
        if (controle > 0) {
          /*
          //Serial1.print("motor2 mais lento (ir para direita) V1 = ");
          //Serial1.print(velocity);
          //Serial1.print(" V2 = ");
          //Serial1.print(velocidade);
          */
          setSpeedMotors( velocity, velocidade);
          
        } //está pra direita
        else if (controle < 0) {
          /*
          //Serial1.print("motor1 mais lento (ir para esquerda) V1=");
          //Serial1.print(velocity);
          //Serial1.print(" V2 = ");
          //Serial1.print(velocidade);
          */
          setSpeedMotors( velocidade, velocity);
        } else {
          setSpeedMotors( velocity, velocity);
        }
      }
      if (meuPid.returnError() > 0) {
        //goright(10000, meuPid.returnError());
      } else if (meuPid.returnError() < 0) {
        //goleft(10000, abs(meuPid.returnError()));
      }
      stop1();
      //Serial1.print("EncInicial ");
      //Serial1.println(enc1Inicial);
      //Serial1.print("EncFinal ");
      //Serial1.println(enc1Inicial+distance*(enc1metro/100));
      //Serial1.print("Enc1Atual ");
      //Serial1.println(enc1);
      //Serial1.print("Enc2Atual ");
      //Serial1.println(enc2);
      //Serial1.print("Distancia Percorrida");
      //Serial1.println(distanciaAtual-distanciaInicial);
      //Serial1.print("Variacao Angular ");
      //Serial1.println(angulo-anguloInicial);
  }
}
void gobackward(int velocity, double distance = NULL){
  if(distance== NULL) {
      //setSpeedMotors(-velocity,-velocity);
      setSpeedMotors(-velocity,-velocity, accelGlobal);
  }else{
    leitura();
    double anguloInicial = angulo;
    //Serial1.print("Angulo Inicial");
    //Serial1.println(anguloInicial);
    //double erro = angleDistance(anguloInicial,angulo);
    meuPid.setSetPoint(anguloInicial);
    roboclaw.SetEncM1(address, 0);
    roboclaw.SetEncM2(address, 0);
    readEncSpeed();
    double enc1Inicial= enc1;
    double enc2Inicial= enc2;
    double distanciaInicial = ((enc1+enc2)/2)/(enc1metro/100);//cm
    //Serial1.print("Distancia Inicial ");
    //Serial1.println(distanciaInicial);
    double distanciaAtual = distanciaInicial;
      while ((distanciaInicial-distanciaAtual)<distance && !(Serial1.available() > 0)) {
      readEncSpeed();
      distanciaAtual = ((enc1+enc2)/2)/(enc1metro/100);//cm
      Serial.print(" Inicial");
      Serial.print(distanciaInicial);
      Serial.print(" ");
      Serial.print(" Atual");
      Serial.print(distanciaAtual);
      Serial.print(" ");
      Serial.print("GoForward");
      //displayspeed();
      /*//Serial1.print("Angulo Inicial");
      //Serial1.println(anguloInicial);
      //Serial1.print("Angulo Passado");
      //Serial1.println(anguloAnterior);
      */
      leitura();
      meuPid.addNewSample(angulo);
      //erro = angleDistance(anguloInicial,angulo);
      // //Serial1.print("erro  ");
      // //Serial1.println(erro);
      int controle = meuPid.process();
      int velocidade = velocity - (abs(controle) * 100);
      //está pra esquerda
      if (controle > 0) {
        Serial.print("motor2 mais lento (ir para direita) V1 = ");
        Serial.print(velocity);
        Serial.print(" V2 = ");
        Serial.print(velocidade);
        setSpeedMotors( -velocidade, -velocity);
      } //está pra direita
      else if (controle < 0) {
        Serial.print("motor1 mais lento (ir para esquerda) V1=");
        Serial.print(velocity);
        Serial.print(" V2 = ");
        Serial.print(velocidade);
        setSpeedMotors(-velocity, -velocidade);
      } else {
        setSpeedMotors(-velocity, -velocity);
      }
    }
    if (meuPid.returnError() > 0) {
      //goleft(10000, abs(meuPid.returnError()));
    } else if (meuPid.returnError() < 0) {
      //pgoright(10000, meuPid.returnError());
    }
    stop1();
    //Serial1.print("EncInicial ");
    //Serial1.println(enc1Inicial);
    //Serial1.print("EncFinal ");
    //Serial1.println(enc1Inicial+distance*(enc1metro/100));
    //Serial1.print("Enc1Atual ");
    //Serial1.println(enc1);
    //Serial1.print("Enc2Atual ");
    //Serial1.println(enc2);
    //Serial1.print("Distancia Percorrida");
    //Serial1.println(distanciaInicial-distanciaAtual);
    //Serial1.print("Variacao Angular ");
    //Serial1.println(angulo-anguloInicial);
  }
}
void setSpeedMotors(double velocidade1, double velocidade2,double accel = NULL){
  if (flagTime){
    responceTime = millis()-inicialResponceTime;
    //Serial1.print("Tempo de Resposta = ");
    //Serial1.println(responceTime);
    flagTime = false;
  }
  if (accel == NULL){
  	roboclaw.SpeedM1M2(address,velocidade1, velocidade2);
  }else{
  	roboclaw.SpeedAccelM1M2(address, accel, velocidade1, velocidade2);
  }
}
void stop1() {
  roboclaw.DutyM1M2(address, 0, 0);
  Serial1.print("done");
  Serial1.print("z");
}
double angleDistance(int alpha, int beta) {
  double phi = abs(beta - alpha) % 360;       // This is either the distance or 360 - distance
  double distance = phi > 180 ? 360 - phi : phi;
  int sign = (alpha - beta >= 0 && alpha - beta <= 180) || (alpha - beta <= -180 && alpha - beta >= -360) ? 1 : -1;
  distance *= sign;
  return distance;
}


void readEncSpeed(void)
{
  uint8_t status1, status2, status3, status4;
  bool valid1, valid2, valid3, valid4;

  enc1 = roboclaw.ReadEncM1(address, &status1, &valid1);
  enc2 = roboclaw.ReadEncM2(address, &status2, &valid2);
  speed1 = roboclaw.ReadSpeedM1(address, &status3, &valid3);
  speed2 = roboclaw.ReadSpeedM2(address, &status4, &valid4);
  /*
  //Serial1.print("Encoder1:");
  
  if (valid1) {
    //Serial1.print(enc1, DEC);
    //Serial1.print(" ");
    //Serial1.print(status1, HEX);
    //Serial1.print(" ");
  }
  else {
    //Serial1.print("failed ");
  }
  //Serial1.print("Encoder2:");
  if (valid2) {
    //Serial1.print(enc2, DEC);
    //Serial1.print(" ");
    //Serial1.print(status2, HEX);
    //Serial1.print(" ");
  }
  else {
    //Serial1.print("failed ");
  }
  //Serial1.print("Speed1:");
  if (valid3) {
    //Serial1.print(speed1, DEC);
    //Serial1.print(" ");
  }
  else {
    //Serial1.print("failed ");
  }
  //Serial1.print("Speed2:");
  if (valid4) {
    //Serial1.print(speed2, DEC);
    //Serial1.print(" ");
  }
  else {
    //Serial1.print("failed ");
  }
  //Serial1.println();
  */
}

