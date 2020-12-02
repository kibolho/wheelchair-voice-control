double angleDistance(int alpha, int beta);
void readEncSpeed(void);
void stop(void);
void leitura(void);
void goright(int velocity, int angle);
void goleft(int velocity, int angle);
void gobackward(int velocity, double distance);
void goforward(int velocity, double distance);
void setSpeedMotors(double velocidade1, double velocidade2);
//Arduino Mega and Leonardo chips only support some pins for receiving data back from the RoboClaw
//This is because only some pins of these boards support PCINT interrupts or are UART receivers.
//Mega: 0,10,11,12,13,14,15,17,19,50,51,52,53,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15
//Leonardo: 0,8,9,10,11

//Arduino Due currently does not support SoftwareSerial. Only hardware uarts can be used, pins 0/1, 14/15, 16/17 or 18/19.

//Note: Most Arduinos do not support higher baudrates rates than 115200.  Also the arduino hardware uarts generate 57600 and 115200 with a
//relatively large error which can cause communications problems.

//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

#include "BMSerial.h"
#include <SoftwareSerial.h>
#include "RoboClaw.h"

//
//Roboclaw
//

//Roboclaw Address
#define address 0x80

//Definte terminal for display. Use hardware serial pins 0 and 1
BMSerial terminal(0, 1);

//Setup communcaitions with roboclaw. Use pins 10 and 11 with 10ms timeout
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
      terminal.print(" erro  ");
      terminal.print(error);
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
      terminal.print(" Controle  ");
      terminal.println(pid);
      return pid;
    }
};
PID meuPid(1.0, 0.01, 0);

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

void setup() {
  Serial1.begin(9600);//Bluetooth
  //Open terminal and roboclaw serial ports
  terminal.begin(115200);
  roboclaw.begin(38400);

  //Serial.begin(9600);
  //Serial1.begin(9600);

  //initSensors();

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
  terminal.println("Cadeira started");

  stop();
  //goforward(12000,76000); //76000 é um quadrado da sala
  //delay(100);
  //gobackward(12000,76000); //76000 é um quadrado da sala
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  terminal.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  terminal.println(F("Testing device connections..."));
  terminal.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  /*Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
  */
  // load and configure the DMP
  terminal.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  double tempo = millis(); //Tempo para estabilizar leituras
  while (millis() - tempo < 15000) {
    //Serial.println(millis()-tempo); // Imprime o tempo passado
    leitura();
  }
  //goforward(10000, 30000); delay(5000);
  //goleft(10000, 90); delay(5000);
  //goleft(10000, 90); delay(5000);
  //goforward(10000, 30000); delay(5000);
  //goright(10000, 90); delay(5000);
  //goright(10000, 90);
}
// Main loop
void loop()
{

}
void serialEvent1() {
  //Serial.println("Recebeu");
  //while is reading the message
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
      // First byte contains a generic "command" byte. We arbitrarily defined '1' as the command to then check the 2nd parameter byte
      // User can additional commands by adding case 2, 3, 4, etc
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
              goforward(20000, 3000);
              break;
            case 'd':
              Serial.println("Direita");
              goright(20000, 90);
              break;
            case 'e':
              Serial.println("Esquerda");
              goleft(20000, 90);
              break;
            case 'a':
              Serial.println("Trás");
              gobackward(20000, 3000);
              break;
            case 'p':
              Serial.println("Parar");
              stop();
              break;
            default: break; // do nothing
          }// switch (param)
      }break; // switch (cmd) case 1
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
              goforward(30000, distanceBT);
              break;
            case 'd':
              Serial.println("Direita");
              for (int a = 2; a < (index - 1); a++) {
                param2.concat(message[a]);
              }
              angleBT = param2.toInt();
              Serial.print("Param2 = ");
              Serial.println(angleBT);
              goright(30000, angleBT);
              break;
            case 'e':
              Serial.println("Esquerda");
              for (int a = 2; a < (index - 1); a++) {
                param2.concat(message[a]);
              }
              angleBT = param2.toInt();
              Serial.print("Param2 = ");
              Serial.println(angleBT);
              goleft(30000, angleBT);
              break;
            case 'a':
              Serial.println("Tras");
              for (int a = 2; a < (index - 1); a++) {
                param2.concat(message[a]);
              }
              distanceBT = param2.toInt();
              Serial.print("Param2 = ");
              Serial.println(distanceBT);
              gobackward(30000, distanceBT);
              break;
            default: break; // do nothing
          } // switch (param)
      }break;
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
                  if (slideBT == 50){
                    int velocidade = 5000;
                    roboclaw.SpeedM1M2(address, velocidade, -velocidade);
                  }else{
                     int velocidade = 100*slideBT;
                    roboclaw.SpeedM1M2(address, 4000+velocidade, -4000);
                  }
              }else if(param1=='F'){
                 Serial.println("Direita Frente");
                  for (int a = 2; a < (index - 1); a++) {
                    param2.concat(message[a]);
                  }
                  slideBT = param2.toInt();
                  Serial.print("Param2 = ");
                  Serial.println(slideBT);
                  if (slideBT == 50){
                    int velocidade = 5000;
                    roboclaw.SpeedM1M2(address, velocidade, -velocidade);
                  }else{
                    int velocidade = 100*slideBT;
                    roboclaw.SpeedM1M2(address, 5000+velocidade, 5000-velocidade);
                  }
              }else if(param1=='T'){
                  Serial.println("Direita Tras");
                  for (int a = 2; a < (index - 1); a++) {
                    param2.concat(message[a]);
                  }
                  slideBT = param2.toInt();
                  Serial.print("Param2 = ");
                  Serial.println(slideBT);
                  if (slideBT == 50){
                    int velocidade = 5000;
                    roboclaw.SpeedM1M2(address, velocidade, -velocidade);
                  }else{
                    int velocidade = 100*slideBT;
                    roboclaw.SpeedM1M2(address, -5000-velocidade, -5000+velocidade);
                  }
              }
            break;
            case 'e':
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
                  if (slideBT == 50){
                    int velocidade = 5000;
                    roboclaw.SpeedM1M2(address, -velocidade, +velocidade);
                  }else{
                     int velocidade = 100*slideBT;
                    roboclaw.SpeedM1M2(address, -5000-velocidade, 5000);
                  }
              }else if(param1=='F'){
                 Serial.println("Direita Frente");
                  for (int a = 2; a < (index - 1); a++) {
                    param2.concat(message[a]);
                  }
                  slideBT = param2.toInt();
                  Serial.print("Param2 = ");
                  Serial.println(slideBT);
                  if (slideBT == 50){
                    int velocidade = 5000;
                    roboclaw.SpeedM1M2(address, -velocidade, +velocidade);
                  }else{
                    int velocidade = 100*slideBT;
                    roboclaw.SpeedM1M2(address, 5000-velocidade, 5000+velocidade);
                  }
              }else if(param1=='T'){
                  Serial.println("Direita Tras");
                  for (int a = 2; a < (index - 1); a++) {
                    param2.concat(message[a]);
                  }
                  slideBT = param2.toInt();
                  Serial.print("Param2 = ");
                  Serial.println(slideBT);
                  if (slideBT == 50){
                    int velocidade = 5000;
                    roboclaw.SpeedM1M2(address, velocidade, -velocidade);
                  }else{
                    int velocidade = 100*slideBT;
                    roboclaw.SpeedM1M2(address, -5000+velocidade, -5000-velocidade);
                  }
            default: break; // do nothing
          }
      }break;
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
void leitura()
{
  bool valid;
  erro = roboclaw.ReadError(address, &valid);
  if (valid && erro != 0) {
    terminal.println(erro, HEX);
  }
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //Serial.print("ypr\t");
    //Serial.println();

    angulo = ypr[0] * 180 / M_PI;
    if (angulo < 0) {
      angulo = -angulo;
      angulo = map(angulo, 0, 180, 180, 0);
      angulo = angulo + 180;
    }
    Serial.print(" Angulo Atual ");
    Serial.print(angulo);
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

void goright(int velocity, int angle) {
  //displayspeed();
  double anguloInicial = angulo;
  Serial.print("Angulo Inicial");
  Serial.println(anguloInicial);
  double angulofinal = anguloInicial + angle;
  if (angulofinal > 360) {
    angulofinal = angulofinal - 360;
  }
  terminal.print("AnguloFinal");
  terminal.println(angulofinal);
  meuPid.setSetPoint(angulofinal);
  meuPid.addNewSample(angulo);
  int controle = meuPid.process();
  double erro = abs(meuPid.returnError());
  double tempo = millis();
  while ((erro >= 2 && (millis() - tempo) < 10000) && !(Serial1.available() > 0)) {
    int velocidade = abs(controle * 100);
    if (velocidade > velocity) {
      velocidade = velocity;
    }
    terminal.print("Virando direita Velocidade =");
    terminal.println(velocidade);
    roboclaw.SpeedM1M2(address, velocidade, -velocidade);
    leitura();
    meuPid.addNewSample(angulo);
    controle = meuPid.process();
    erro = abs(meuPid.returnError());
  }
  stop();
}
void goleft(int velocity, int angle) {
  //displayspeed();
  double anguloInicial = angulo;
  Serial.print("Angulo Inicial");
  Serial.println(anguloInicial);
  double angulofinal = anguloInicial - angle;
  if (angulofinal < 0) {
    angulofinal = angulofinal + 360;
  }
  terminal.print("AnguloFinal");
  terminal.println(angulofinal);
  meuPid.setSetPoint(angulofinal);
  meuPid.addNewSample(angulo);
  int controle = meuPid.process();
  double erro = abs(meuPid.returnError());

  double tempo = millis();
  while ((erro >= 2 && (millis() - tempo) < 10000) && !(Serial1.available() > 0)) {
    int velocidade = controle * 100; 
    if (velocidade < -velocity) {
      velocidade = -velocity;
    }
    terminal.print("Virando esquerda Velocidade =");
    terminal.println(velocidade);
    roboclaw.SpeedM1M2(address, velocidade, -velocidade);
    leitura();
    meuPid.addNewSample(angulo);
    controle = meuPid.process();
    erro = abs(meuPid.returnError());
  }
  stop();
}

void goforward(int velocity, double distance) {
  double anguloInicial = angulo;
  Serial.print("Angulo Inicial");
  Serial.println(anguloInicial);
  //double erro = angleDistance(anguloInicial,angulo);
  double tempo = millis();
  meuPid.setSetPoint(anguloInicial);
  readEncSpeed();
  double distanciaInicial = ((enc1+enc2)/2)/(enc1metro/100);//cm
  Serial.print("Distancia Inicial ");
  Serial.println(distanciaInicial);
  double distanciaAtual = distanciaInicial;
  while ((distanciaAtual-distanciaInicial)<distance && !(Serial1.available() > 0)) {
    readEncSpeed();
    distanciaAtual = ((enc1+enc2)/2)/(enc1metro/100);//cm
    Serial.print(" Inicial");
    Serial.print(distanciaInicial);
    Serial.print(" ");
    Serial.print(" Atual");
    Serial.print(distanciaAtual);
    Serial.print(" ");
    //Serial.print("GoForward");
    //displayspeed();
    /*Serial.print("Angulo Inicial");
    Serial.println(anguloInicial);
    Serial.print("Angulo Passado");
    Serial.println(anguloAnterior);
    */
    leitura();
    meuPid.addNewSample(angulo);
    //erro = angleDistance(anguloInicial,angulo);
    //terminal.print("erro  ");
    //terminal.println(erro);
    int controle = meuPid.process();
    int velocidade = velocity - (abs(controle) * 100);
    //está pra esquerda
    if (controle > 0) {
      /*
      terminal.print("motor2 mais lento (ir para direita) V1 = ");
      terminal.print(velocity);
      terminal.print(" V2 = ");
      terminal.print(velocidade);
      */
      roboclaw.SpeedM1M2(address, velocity, velocidade);
      
    } //está pra direita
    else if (controle < 0) {
      /*
      terminal.print("motor1 mais lento (ir para esquerda) V1=");
      terminal.print(velocity);
      terminal.print(" V2 = ");
      terminal.print(velocidade);
      */
      roboclaw.SpeedM1M2(address, velocidade, velocity);
    } else {
      roboclaw.SpeedM1M2(address, velocity, velocity);
    }
  }
  if (meuPid.returnError() > 0) {
    //goright(10000, meuPid.returnError());
  } else if (meuPid.returnError() < 0) {
    //goleft(10000, abs(meuPid.returnError()));
  }
  stop();
}
void gobackward(int velocity, int distance) {
  double anguloInicial = angulo;
  Serial.print("Angulo Inicial");
  Serial.println(anguloInicial);
  //double erro = angleDistance(anguloInicial,angulo);
  double tempo = millis();
  meuPid.setSetPoint(anguloInicial);
  readEncSpeed();
  double distanciaInicial = ((enc1+enc2)/2)/(enc1metro/100);//cm
  Serial.print("Distancia Inicial ");
  Serial.println(distanciaInicial);
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
    //Serial.print("GoForward");
    //displayspeed();
    /*Serial.print("Angulo Inicial");
    Serial.println(anguloInicial);
    Serial.print("Angulo Passado");
    Serial.println(anguloAnterior);
    */
    leitura();
    meuPid.addNewSample(angulo);
    //erro = angleDistance(anguloInicial,angulo);
    //terminal.print("erro  ");
    //terminal.println(erro);
    int controle = meuPid.process();
    int velocidade = velocity - (abs(controle) * 100);
    //está pra esquerda
    if (controle > 0) {
      terminal.print("motor2 mais lento (ir para direita) V1 = ");
      terminal.print(velocity);
      terminal.print(" V2 = ");
      terminal.print(velocidade);
      roboclaw.SpeedM1M2(address, -velocidade, -velocity);
    } //está pra direita
    else if (controle < 0) {
      terminal.print("motor1 mais lento (ir para esquerda) V1=");
      terminal.print(velocity);
      terminal.print(" V2 = ");
      terminal.print(velocidade);
      roboclaw.SpeedM1M2(address, -velocity, -velocidade);
    } else {
      roboclaw.SpeedM1M2(address, -velocity, -velocity);
    }
  }
  if (meuPid.returnError() > 0) {
    //goleft(10000, abs(meuPid.returnError()));
  } else if (meuPid.returnError() < 0) {
    //pgoright(10000, meuPid.returnError());
  }
  stop();
}
void stop() {
  roboclaw.DutyM1M2(address, 0, 0);
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
  terminal.print("Encoder1:");

  if (valid1) {
    terminal.print(enc1, DEC);
    terminal.print(" ");
    terminal.print(status1, HEX);
    terminal.print(" ");
  }
  else {
    terminal.print("failed ");
  }
  terminal.print("Encoder2:");
  if (valid2) {
    terminal.print(enc2, DEC);
    terminal.print(" ");
    terminal.print(status2, HEX);
    terminal.print(" ");
  }
  else {
    terminal.print("failed ");
  }
  terminal.print("Speed1:");
  if (valid3) {
    terminal.print(speed1, DEC);
    terminal.print(" ");
  }
  else {
    terminal.print("failed ");
  }
  terminal.print("Speed2:");
  if (valid4) {
    terminal.print(speed2, DEC);
    terminal.print(" ");
  }
  else {
    terminal.print("failed ");
  }
  //terminal.println();
}

