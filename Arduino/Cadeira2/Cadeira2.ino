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
  /*RF  
  // VirtualWire 
  // Initialise the IO and ISR
  // Required for DR3100
  vw_set_ptt_inverted(true); 
  vw_set_rx_pin(RFData);
  // Bits per sec
  vw_setup(2000);     
  
  // Start the receiver PLL running
  vw_rx_start();  
  timeRF = millis();
  */
  
  terminal.println("Cadeira started");
  
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

void goright(int velocity){
  roboclaw.SpeedM1M2(address, velocity, -velocity);
}
void goleft(int velocity){
  roboclaw.SpeedM1M2(address, -velocity, velocity);
}
void goforward(int velocity, int distance){
  terminal.println("goforward");
  uint8_t depth1,depth2;
  roboclaw.SpeedDistanceM1M2(address,velocity,distance,velocity,distance,1);
  do{
    displayspeed();
    roboclaw.ReadBuffers(address,depth1,depth2);
  }while(depth1!=0x80 && depth2!=0x80);  //Loop until distance command has completed
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
float convertAngle(float angle){
  //0 <-> 180 e -180 <-> 0 para 0 - 360
  if (angle<0){ //-180
    angle = 360 + angle; 
  }
  return angle;
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

