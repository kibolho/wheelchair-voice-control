//Arduino Mega and Leonardo chips only support some pins for receiving data back from the RoboClaw
//This is because only some pins of these boards support PCINT interrupts or are UART receivers.
//Mega: 0,10,11,12,13,14,15,17,19,50,51,52,53,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15
//Leonardo: 0,8,9,10,11

//Arduino Due currently does not support SoftwareSerial. Only hardware uarts can be used, pins 0/1, 14/15, 16/17 or 18/19.

//Note: Most Arduinos do not support higher baudrates rates than 115200.  Also the arduino hardware uarts generate 57600 and 115200 with a
//relatively large error which can cause communications problems.

//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include "BMSerial.h"
#include "RoboClaw.h"
#include <Wire.h>
#include <VirtualWire.h>
/*
class PID {
  public:
  double error;
  double sample;
  double lastSample;

  double P;
  double kP;

  double I;
  double kI;

  double D;
  double kD;

  double PIDR = 0;

  double setPoint;
  long lastProcess;

  PID(double _kP,  double _kI, double _kD){
    kP = _kP;
    kI = _kI;
    kD = _kD;
  }
  void addNewSample(double _sample){
    sample = _sample;
  }
  void setSetPoint (double _setPoint){
    setPoint = _setPoint;
  }
  double process(){
    //Implementação P I D
    error = setPoint - sample;
    float deltaTime = (millis() - lastProcess) /1000.0;
    lastProcess = millis();
    //P
    P = error * kP;
    //I
    I += (error * kI) *deltaTime;
    //D
    D = (lastSample- sample) * kD * deltaTime; 
    lastSample = sample;
    //PID
    PIDR = P + I + D;
    return PIDR;
  }
};
*/
int flag=0;

bool manualMode = false;
//
//RF
//
#define RFData 12

// Sensors 
int Sensor1Data;

// RF Transmission container
char StringReceived[16]; 

int x = 512;
int y = 512;
int btn1 = 0;
int btn2 = 0;
int btnJoy = 0;

int buttonState1;             // the current reading from the input pin
int lastButtonState1 = LOW;   // the previous reading from the input pin

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime1 = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers

//
//Roboclaw 
//

//Roboclaw Address
#define address 0x80

//Definte terminal for display. Use hardware serial pins 0 and 1
BMSerial terminal(0,1);

//Setup communcaitions with roboclaw. Use pins 10 and 11 with 10ms timeout
RoboClaw roboclaw(10,11,10000);

//
//IMU
//


#define OUTPUT_DATA_INTERVAL 20  // in milliseconds



#define COMMAND_INTERVAL 200
long timeRF;

// SENSOR CALIBRATION
/*****************************************************************/
// How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
// Put MIN/MAX and OFFSET readings for your board here
// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define ACCEL_X_MIN (-250.0f)
#define ACCEL_X_MAX (250.0f)
#define ACCEL_Y_MIN (-250.0f)
#define ACCEL_Y_MAX (250.0f)
#define ACCEL_Z_MIN (-250.0f)
#define ACCEL_Z_MAX (250.0f)

// Magnetometer
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define MAGN_X_MIN (-600.0f)
#define MAGN_X_MAX (600.0f)
#define MAGN_Y_MIN (-600.0f)
#define MAGN_Y_MAX (600.0f)
#define MAGN_Z_MIN (-600.0f)
#define MAGN_Z_MAX (600.0f)

// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
#define GYRO_X_OFFSET (0.0f)
#define GYRO_Y_OFFSET (0.0f)
#define GYRO_Z_OFFSET (0.0f)


// Altymeter
#define ALT_SEA_LEVEL_PRESSURE 102133

/*
// Calibration example:
// "accel x,y,z (min/max) = -278.00/270.00  -254.00/284.00  -294.00/235.00"
#define ACCEL_X_MIN ((float) -278)
#define ACCEL_X_MAX ((float) 270)
#define ACCEL_Y_MIN ((float) -254)
#define ACCEL_Y_MAX ((float) 284)
#define ACCEL_Z_MIN ((float) -294)
#define ACCEL_Z_MAX ((float) 235)

// "magn x,y,z (min/max) = -511.00/581.00  -516.00/568.00  -489.00/486.00"
#define MAGN_X_MIN ((float) -511)
#define MAGN_X_MAX ((float) 581)
#define MAGN_Y_MIN ((float) -516)
#define MAGN_Y_MAX ((float) 568)
#define MAGN_Z_MIN ((float) -489)
#define MAGN_Z_MAX ((float) 486)

//"gyro x,y,z (current/average) = -32.00/-34.82  102.00/100.41  -16.00/-16.38"
#define GYRO_AVERAGE_OFFSET_X ((float) -34.82)
#define GYRO_AVERAGE_OFFSET_Y ((float) 100.41)
#define GYRO_AVERAGE_OFFSET_Z ((float) -16.38)
*/

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))

// Gain for gyroscope
#define GYRO_GAIN_X (0.06957f)
#define GYRO_GAIN_Y (0.06957f)
#define GYRO_GAIN_Z (0.06957f)

#define GYRO_X_SCALE (TO_RAD(GYRO_GAIN_X))
#define GYRO_Y_SCALE (TO_RAD(GYRO_GAIN_Y))
#define GYRO_Z_SCALE (TO_RAD(GYRO_GAIN_Z))

// DCM parameters
#define Kp_ROLLPITCH (0.02f)
#define Ki_ROLLPITCH (0.00002f)
#define Kp_YAW (1.2f)
#define Ki_YAW (0.00002f)

// Stuff
#define GRAVITY (256.0f) // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

// RAW sensor data
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
//float accel_min[3];
//float accel_max[3];

float magnetom[3];
//float magnetom_min[3];
//float magnetom_max[3];

float gyro[3];
//float gyro_average[3];
//int gyro_num_samples = 0;

float temperature;
float pressure;
float altitude;

// DCM variables
float MAG_Heading;
float Magn_Vector[3]= {0, 0, 0}; // Store the magnetometer turn rate in a vector
float Accel_Vector[3]= {0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3]= {0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3]= {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3]= {0, 0, 0}; // Omega Proportional correction
float Omega_I[3]= {0, 0, 0}; // Omega Integrator
float Omega[3]= {0, 0, 0};
float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Euler angles
float yaw, pitch, roll;

// DCM timing in the main loop
long timestamp;
long timestamp_old;
float G_Dt; // Integration time for DCM algorithm

// More output-state variables
int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;

void ReadSensors() {
  Read_Pressure();
  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
  ApplySensorMapping();  
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void reset_sensor_fusion()
{
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  ReadSensors();
  
  timestamp = millis();
  
  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  pitch = -atan2(Accel_Vector[0], sqrt(Accel_Vector[1] * Accel_Vector[1] + Accel_Vector[2] * Accel_Vector[2]));
  
  // GET ROLL
  // Compensate pitch of gravity vector 
  Vector_Cross_Product(temp1, Accel_Vector, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  // Normally using x-z-plane-component/y-component of compensated gravity vector
  // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  // Since we compensated for pitch, x-z-plane-component equals z-component:
  roll = atan2(temp2[1], temp2[2]);
  
  // GET YAW
  Compass_Heading();
  yaw = MAG_Heading;
  
  // Init rotation matrix
  init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

// Apply calibration to raw sensor readings
void ApplySensorMapping()
{
    // Magnetometer axis mapping
    Magn_Vector[1] = -magnetom[0];
    Magn_Vector[0] = -magnetom[1];
    Magn_Vector[2] = -magnetom[2];

    // Magnetometer values mapping
    Magn_Vector[0] -= MAGN_X_OFFSET;
    Magn_Vector[0] *= MAGN_X_SCALE;
    Magn_Vector[1] -= MAGN_Y_OFFSET;
    Magn_Vector[1] *= MAGN_Y_SCALE;
    Magn_Vector[2] -= MAGN_Z_OFFSET;
    Magn_Vector[2] *= MAGN_Z_SCALE;
  
    // Accelerometer axis mapping
    Accel_Vector[1] = accel[0];
    Accel_Vector[0] = accel[1];
    Accel_Vector[2] = accel[2];

    // Accelerometer values mapping
    Accel_Vector[0] -= ACCEL_X_OFFSET;
    Accel_Vector[0] *= ACCEL_X_SCALE;
    Accel_Vector[1] -= ACCEL_Y_OFFSET;
    Accel_Vector[1] *= ACCEL_Y_SCALE;
    Accel_Vector[2] -= ACCEL_Z_OFFSET;
    Accel_Vector[2] *= ACCEL_Z_SCALE;
    
    // Gyroscope axis mapping
    Gyro_Vector[1] = -gyro[0];
    Gyro_Vector[0] = -gyro[1];
    Gyro_Vector[2] = -gyro[2];

    // Gyroscope values mapping
    Gyro_Vector[0] -= GYRO_X_OFFSET;
    Gyro_Vector[0] *= GYRO_X_SCALE;
    Gyro_Vector[1] -= GYRO_Y_OFFSET;
    Gyro_Vector[1] *= GYRO_Y_SCALE;
    Gyro_Vector[2] -= GYRO_Z_OFFSET;
    Gyro_Vector[2] *= GYRO_Z_SCALE;
}

//PID meuPID(1.0,0,0);
void initSensors(){
  // Init sensors
  delay(50);  // Give sensors enough time to start
  terminal.println("Cadeira started");
  I2C_Init();
  Accel_Init();
  Magn_Init();
  Gyro_Init();
  Pressure_Init();
  
  // Read sensors, init DCM algorithm
  delay(20);  // Give sensors enough time to collect data
  reset_sensor_fusion();
}
void setup(){

  //Open terminal and roboclaw serial ports
  terminal.begin(57600);
  roboclaw.begin(38400);
  
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
  
  // VirtualWire 
  // Initialise the IO and ISR
  // Required for DR3100
  vw_set_ptt_inverted(true); 
  vw_set_rx_pin(RFData);
  // Bits per sec
  vw_setup(2000);     
  
  // Start the receiver PLL running
  vw_rx_start();  
  terminal.println("Cadeira started");
  timeRF = millis();
  stop();
  //turnright(90);
  //turnright(180);
  //terminal.println("Go Forward ");
  
  goforward(12000,76000); //76000 é um quadrado da sala
  delay(100);
  gobackward(12000,76000); //76000 é um quadrado da sala
}
// Main loop
void loop()
{
  
  /*
  //Controle RF
  uint8_t buf[VW_MAX_MESSAGE_LEN];
  uint8_t buflen = VW_MAX_MESSAGE_LEN;
  // Non-blocking
  if (vw_get_message(buf, &buflen)) 
  {    
      // Message with a good checksum received, dump it. 
      for (int i = 0; i < buflen; i++)
      {            
        // Fill StringReceived Char array with corresponding 
        // chars from buffer.   
        StringReceived[i] = char(buf[i]);
      }
      
      sscanf(StringReceived, "%d,%d,%d,%d,%d",&x, &y,&btnJoy,&btn2, &btn1);
      terminal.println(StringReceived);
      manualMode = !btn2;
      if(manualMode){
          terminal.println("Manual mode ON");
      }
      timeRF = millis();
  }else{
    readSensors();
  }
  //Modo Manual Ou Sequencial: 
  //terminal.println(manualMode);
  if ((millis() - timeRF) >= COMMAND_INTERVAL) {
    if(manualMode){
        if(y>700){ //forward
         terminal.println("Forward");
          goforward(10000);
        }else if (y<350){ //back
          terminal.println("Backwards");
          gobackwards(10000);
        }else if(x<400){ //left
          terminal.println("Left");
          goleft(10000);
          //turnleft(90);
          //roboclaw.SpeedM1M2(address, 1000, 10000);
        }else if (x>600){ //right
          terminal.println("Right");
          goright(10000);
          //turnright(90);
          //roboclaw.SpeedM1M2(address, 10000, 1000);
        }else{
         stop();
        }
    }else{
      goforward(12000);
      terminal.println("GoForward");
    }
  }
  readSensors();
  if(flag==0){
    roboclaw.SpeedDistanceM1M2(address,-12000,70000,12000,70000,1);
  }
  flag++;
  */
  
}
void readSensors(){
  // Time to read the sensors again?
  if ((millis() - timestamp) >= OUTPUT_DATA_INTERVAL) {
    timestamp_old = timestamp;
    timestamp = millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;
    ReadSensors();
    
    // Run DCM algorithm
    Compass_Heading(); // Calculate magnetic heading
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();
    /*
    Serial.println();
    terminal.print(" YAW ");
    terminal.print(TO_DEG(yaw));    Serial.print(";");
    terminal.print(TO_DEG(pitch));  Serial.print(";");
    terminal.print(TO_DEG(roll));   Serial.print(";");
    
    terminal.print(temperature);    Serial.print(";");
    terminal.print(pressure);       Serial.print(";");
    terminal.print(altitude);       
    */
  }
}
void goright(int velocity){
  roboclaw.SpeedM1M2(address, velocity, -velocity);
}
void goleft(int velocity){
  roboclaw.SpeedM1M2(address, -velocity, velocity);
}
void goforward(int velocity, int distance){
  uint8_t depth1,depth2;
  //roboclaw.SpeedDistanceM1M2(address,velocity,distance,velocity,distance,1);
  do{
    displayspeed();
    roboclaw.ReadBuffers(address,depth1,depth2);
  }while(depth1!=0x80 && depth2!=0x80);  //Loop until distance command has completed

}
void gobackward(int velocity, int distance){
  uint8_t depth1,depth2;
  //roboclaw.SpeedDistanceM1M2(address,-velocity,distance,-velocity,distance,1);
  do{
    displayspeed();
    roboclaw.ReadBuffers(address,depth1,depth2);
  }while(depth1!=0x80 && depth2!=0x80);  //Loop until distance command has completed

}
void stop(){
  roboclaw.DutyM1M2(address, 0, 0);
}
float convertAngle(float angle){
  //0 <-> 180 e -180 <-> 0 para 0 - 360
  if (angle<0){ //-180
    angle = 360 + angle; 
  }
  return angle;
}
void turnright(float angle){ //90
  readSensors();
  float angleInitial = convertAngle(TO_DEG(yaw));
  terminal.print("Angulo Recebido ");
  terminal.println(angle);
  terminal.print("Angulo Inicial ");
  terminal.println(angleInitial);
  float angleFinal;
  if ((angleInitial + angle)<0){
    angleFinal = (angleInitial + angle)+360;
  }else if((angleInitial + angle)>360){
    angleFinal = (angleInitial + angle)-360;
  }else{
    angleFinal = angleInitial + angle;
  }
  //terminal.print("Angulo Final ");
  //terminal.println(angleFinal);
  float angleAtual;
  float erro = 2;
  //meuPID.setSetPoint(angleFinal);
  while(abs(erro)>1){
  //while(true){
    readSensors();
    angleAtual = convertAngle(TO_DEG(yaw));
    //meuPID.addNewSample(angleAtual);    
    //erro = meuPID.process();
    erro= abs(angleAtual-angleFinal);
    //Angle Final = 90 e angulo atual 350
    if(erro>angle){
      erro=360-angleAtual+angleFinal;
    }
    //erro=50;
    roboclaw.SpeedM1M2(address, 50*abs(erro), -50*abs(erro));
    terminal.println(angleAtual);
    /*terminal.print(" Angulo Final ");
    terminal.print(angleFinal);
    terminal.print(" Angulo Atual ");
    terminal.print(angleAtual);
    terminal.print(" Erro ");
    terminal.println(erro);
    */
  }
  stop();
}
void turnleft(float angle){ //80
 readSensors();
  float angleInitial = convertAngle(TO_DEG(yaw));
  terminal.print("Angulo Recebido ");
  terminal.println(angle);
  terminal.print("Angulo Inicial ");
  terminal.println(angleInitial);
  float angleFinal;
  if ((angleInitial - angle)<0){
    angleFinal = (angleInitial - angle)+360;
  }else if((angleInitial - angle)>360){
    angleFinal = (angleInitial - angle)-360;
  }else{
    angleFinal = angleInitial - angle;
  }
  terminal.print("Angulo Final ");
  terminal.println(angleFinal);
  float angleAtual;
  float erro = 2.00;
  //meuPID.setSetPoint(angleFinal);
  while(abs(erro)>1){
  //while(true){
    readSensors();
    angleAtual = convertAngle(TO_DEG(yaw));
    //meuPID.addNewSample(angleAtual);    
    //erro = meuPID.process();
    erro= abs(angleAtual-angleFinal);
    //Angle Final = 90 e angulo atual 350
    if(erro>angle){
      erro=360-angleAtual+angleFinal;
    }
    //erro=50;
    roboclaw.SpeedM1M2(address, -50*abs(erro), 50*abs(erro));
    terminal.print(" Angulo Final ");
    terminal.print(angleFinal);
    terminal.print(" Angulo Atual ");
    terminal.print(angleAtual);
    terminal.print(" Erro ");
    terminal.println(erro);
  }
  stop();
}

void displayspeed(void)
{
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;
  
  int32_t enc1= roboclaw.ReadEncM1(address, &status1, &valid1);
  int32_t enc2 = roboclaw.ReadEncM2(address, &status2, &valid2);
  int32_t speed1 = roboclaw.ReadSpeedM1(address, &status3, &valid3);
  int32_t speed2 = roboclaw.ReadSpeedM2(address, &status4, &valid4);
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

