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

char str[4];
 
void setup(){

  //Open terminal and roboclaw serial ports
  terminal.begin(57600);
  roboclaw.begin(38400);
  Serial.begin(9600);
  Serial1.begin(9600);
  
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
}
// Main loop
void loop()
{
  int i=0;

  if (Serial1.available()) {
    delay(100); //allows all serial sent to be received together
    while(Serial1.available() && i<4) {
      str[i++] = Serial1.read();
    }
    str[i++]='\0';
  }
  if(i>0) {
    angulo = str;
  }
}
void spin(float angle,int velocityLeft,int velocityRight){
  float angleInicial = anglePhone;
  while((anglePhone-angle)>4){
    roboclaw.SpeedM1M2(address, velocityLeft, -velocityRight);
  }
}

void goright(int velocityLeft,int velocityRight){
  roboclaw.SpeedM1M2(address, velocityLeft, -velocityRight);
}
void goleft(int velocityLeft,int velocityRight)){
  roboclaw.SpeedM1M2(address, -velocityRight, velocityLeft);
}
void goforward(int velocity, int distance){
  terminal.println("goforward");
  uint8_t depth1,depth2;
  int erroInicial=enc1-enc2;
  roboclaw.SpeedDistanceM1M2(address,velocity,distance,velocity,distance,1);

  do{
    displayspeed();
    roboclaw.ReadBuffers(address,depth1,depth2);
    int erro = (enc1-erroInicial)-enc2;
    if(erro>0){ //Motor 1 está mais na frente
      roboclaw.SpeedDistanceM1M2(address,velocity,distance,velocity+(erro*100),distance,1);
    }else if(erro<0){
      roboclaw.SpeedDistanceM1M2(address,velocity+(erro*100)<!DOCTYPE html>
<html>
  <head>
    <title>Capture Photo</title>
    <meta name="viewport" content="width=device-width,height=device-height,initial-scale=1"/>
    <script type="text/javascript" charset="utf-8" src="js/phonegap.js"></script>
    <script type="text/javascript" charset="utf-8">
    var pictureSource;   // picture source
    var destinationType; // sets the format of returned value 
    // Wait for PhoneGap to connect with the device
    //
    document.addEventListener("deviceready",onDeviceReady,false);
    // PhoneGap is ready to be used!
    //
    function onDeviceReady() {
        pictureSource=navigator.camera.PictureSourceType;
        destinationType=navigator.camera.DestinationType;
    }
    // Called when a photo is successfully retrieved
    //
    function onPhotoDataSuccess(imageData) {
      // Get image handle
      //
      var smallImage = document.getElementById('smallImage');
      // Unhide image elements
      //
      smallImage.style.display = 'block';
      // Show the captured photo
      // The inline CSS rules are used to resize the image
      //
      smallImage.src = "data:image/jpeg;base64," + imageData;
    }
    
  // Called when a photo is successfully retrieved
    //
    function onPhotoFileSuccess(imageData) {
      // Get image handle
      console.log(JSON.stringify(imageData));
      
      // Get image handle
      //
      var smallImage = document.getElementById('smallImage');
      // Unhide image elements
      //
      smallImage.style.display = 'block';
      // Show the captured photo
      // The inline CSS rules are used to resize the image
      //
      smallImage.src = imageData;
    }
    // Called when a photo is successfully retrieved
    //
    function onPhotoURISuccess(imageURI) {
      // Uncomment to view the image file URI 
      // console.log(imageURI);
      // Get image handle
      //
      var largeImage = document.getElementById('largeImage');
      // Unhide image elements
      //
      largeImage.style.display = 'block';
      // Show the captured photo
      // The inline CSS rules are used to resize the image
      //
      largeImage.src = imageURI;
    }
    // A button will call this function
    //
    function capturePhotoWithData() {
      // Take picture using device camera and retrieve image as base64-encoded string
      navigator.camera.getPicture(onPhotoDataSuccess, onFail, { quality: 50 });
    }
    function capturePhotoWithFile() {
        navigator.camera.getPicture(onPhotoFileSuccess, onFail, { quality: 50, destinationType: Camera.DestinationType.FILE_URI });
    }
    
    // A button will call this function
    //
    function getPhoto(source) {
      // Retrieve image file location from specified source
      navigator.camera.getPicture(onPhotoURISuccess, onFail, { quality: 50, 
        destinationType: destinationType.FILE_URI,
        sourceType: source });
    }
    // Called if something bad happens.
    // 
    function onFail(message) {
      alert('Failed because: ' + message);
    }
    </script>
  </head>
  <body>
    <button onclick="capturePhotoWithData();">Capture Photo With Image Data</button> <br>
    <button onclick="capturePhotoWithFile();">Capture Photo With Image File URI</button> <br>
    <button onclick="getPhoto(pictureSource.PHOTOLIBRARY);">From Photo Library</button><br>
    <button onclick="getPhoto(pictureSource.SAVEDPHOTOALBUM);">From Photo Album</button><br>
    <img style="display:none;width:60px;height:60px;" id="smallImage" src="" />
    <img style="display:none;" id="largeImage" src="" />
  </body>
</html>,distance,velocity,distance,1);
    }
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

