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
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

#include "BMSerial.h"
#include "RoboClaw.h"

//
//Roboclaw 
//

//Roboclaw Address
#define address 0x80

//Definte terminal for display. Use hardware serial pins 0 and 1
BMSerial terminal(0,1);

//Setup communcaitions with roboclaw. Use pins 10 and 11 with 10ms timeout
RoboClaw roboclaw(10,11,10000);

int32_t enc1;
int32_t enc2;
int32_t speed1;
int32_t speed2;

double angulo;

class PID{
public:
  
  double error;
  double sample;
  double lastSample;
  double kP, kI, kD;      
  double P, I, D;
  double pid;
  
  double setPoint;
  long lastProcess;
  
  PID(double _kP, double _kI, double _kD){
    kP = _kP;
    kI = _kI;
    kD = _kD;
  }
  
  void addNewSample(double _sample){
    sample = _sample;
  }
  
  void setSetPoint(double _setPoint){
    setPoint = _setPoint;
  }
  double returnError(){
    return angleDistance(setPoint,sample);
  }
  
  double process(){
    // Implementação P ID
    error = angleDistance(setPoint,sample);
    terminal.print("erro  ");
    terminal.println(error);
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
    
    return pid;
  }
};
PID meuPid(1.0,0.1, 0.2);

void setup(){

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
  roboclaw.SetM1VelocityPID(address,D1,P1,I1,qpps1);
  roboclaw.SetM2VelocityPID(address,D2,P2,I2,qpps2);  

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
    while(millis()-tempo<15000){
      //Serial.println(millis()-tempo); // Imprime o tempo passado
      leitura();
    }

    //goleft(10000,90);
    //goleft(10000,90);
    goforward(10000,30000);
    //goright(10000,90);
    //goright(10000,90);
}
// Main loop
void loop()
{
  
}
void leitura()
{
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
        
        angulo = ypr[0] * 180/M_PI;
        if (angulo<0){
          angulo=-angulo;
          angulo=map(angulo,0,180,180,0);
          angulo=angulo+180;
        }
        Serial.print("Angulo Atual");
        Serial.println(angulo);
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}

void goright(int velocity,int angle){
       //displayspeed();
    double anguloInicial=angulo;
    Serial.print("Angulo Inicial");
    Serial.println(anguloInicial);
    double angulofinal=anguloInicial+angle;
    if (angulofinal>360){
      angulofinal=angulofinal-360;
    }
    terminal.print("AnguloFinal");
    terminal.println(angulofinal);
    double erro = angulofinal-angulo;
    erro=abs(erro);
    if (erro>(angle+2)){
      erro=angulofinal-angulo+180;
    }
    while(abs(erro)>=1){
      if (erro<10){
        roboclaw.SpeedM1M2(address,velocity-(erro*200),-velocity+(erro*200));
      }else{
        roboclaw.SpeedM1M2(address,velocity,-velocity);
      }
      if (erro>(angle+2)){
        erro=angulofinal-angulo+180;
      }else{
        erro = angulofinal-angulo;
      }
      leitura();
      terminal.print("erro");
      terminal.println(erro);
    }
    stop();
}
void goleft(int velocity,int angle){
     //displayspeed();
    double anguloInicial=angulo;
    Serial.print("Angulo Inicial");
    Serial.println(anguloInicial);
    double angulofinal=anguloInicial-angle;
    if (angulofinal<0){
      angulofinal=angulofinal+360;
    }
    terminal.print("AnguloFinal");
    terminal.println(angulofinal);
    double erro = angulofinal-angulo;
    erro=abs(erro);
    if (erro>(angle+2)){
      erro=(360-angulofinal)+angulo;
    }
    while(abs(erro)>=1){
      if (erro<10){
        roboclaw.SpeedM1M2(address,-velocity+erro*200,velocity-erro*200);
      }else{
        roboclaw.SpeedM1M2(address,-velocity,velocity);
      }
      leitura();
      if (angulofinal-angulo>(angle+2)){
        erro=(360-angulofinal)+angulo;
      }else{
        erro = angulofinal-angulo;
      }
      erro=abs(erro);
      terminal.print("erro");
      terminal.println(erro);
    }
    stop();
}
void goforward(int velocity, int distance){
    double anguloInicial=angulo;
    Serial.print("Angulo Inicial");
    Serial.println(anguloInicial);
    //double erro = angleDistance(anguloInicial,angulo);
    double tempo = millis();
    meuPid.setSetPoint(anguloInicial);
    while((millis()-tempo)<distance){
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
      //está pra esquerda
      if(controle>0){
        terminal.print("motor2 mais lento controle =");
        terminal.println(controle);
        roboclaw.SpeedM1M2(address,velocity,velocity-controle*100);
      } //está pra direita
      else if(controle<0){
        terminal.print("motor1 mais lento controle =");
        terminal.println(controle);
        roboclaw.SpeedM1M2(address,velocity-(abs(controle)*100),velocity);
      }else{
        roboclaw.SpeedM1M2(address,velocity,velocity);
      }
    }
    if (meuPid.returnError()>0){
      goright(10000,meuPid.returnError());
    }else if (meuPid.returnError()<0){
      goleft(10000,abs(meuPid.returnError()));
    }
    stop();
}
void gobackward(int velocity, int distance){
  terminal.println("gobackward");
  uint8_t depth1,depth2;
  roboclaw.SpeedDistanceM1M2(address,-velocity,distance,-velocity,distance,1);
  do{
    displayspeed();
    roboclaw.ReadBuffers(address,depth1,depth2);
  }while(depth1!=0x80 && depth2!=0x80);  //Loop until distance command has completed
  stop();
}
void stop(){
  roboclaw.DutyM1M2(address, 0, 0);
}
double angleDistance(int alpha, int beta) {
  double phi = abs(beta - alpha) % 360;       // This is either the distance or 360 - distance
  double distance = phi > 180 ? 360 - phi : phi;
  int sign = (alpha - beta >= 0 && alpha - beta <= 180) || (alpha - beta <= -180 && alpha - beta>= -360) ? 1 : -1; 
  distance *= sign;
  return distance;
}
    

void displayspeed(void)
{
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;
  
  enc1= roboclaw.ReadEncM1(address, &status1, &valid1);
  enc2 = roboclaw.ReadEncM2(address, &status2, &valid2);
  speed1 = roboclaw.ReadSpeedM1(address, &status3, &valid3);
  speed2 = roboclaw.ReadSpeedM2(address, &status4, &valid4);
  terminal.print("Encoder1:");

  if(valid1){
    terminal.print(enc1,DEC);
    terminal.print(" ");
    terminal.print(status1,HEX);
    terminal.print(" ");
  }
  else{
  terminal.print("failed ");
  }
  terminal.print("Encoder2:");
  if(valid2){
    terminal.print(enc2,DEC);
    terminal.print(" ");
    terminal.print(status2,HEX);
    terminal.print(" ");
  }
  else{
  terminal.print("failed ");
  }
  terminal.print("Speed1:");
  if(valid3){
    terminal.print(speed1,DEC);
    terminal.print(" ");
  }
  else{
  terminal.print("failed ");
  }
  terminal.print("Speed2:");
  if(valid4){
    terminal.print(speed2,DEC);
    terminal.print(" ");
  }
  else{
  terminal.print("failed ");
  }
  terminal.println();
}

