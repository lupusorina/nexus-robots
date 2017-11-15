#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#define printByte(args) Serial.write(args)
#else
#include "WProgram.h"
#define printByte(args) Serial.print(args,BYTE)
#endif
 
#define SerialPort Serial
#define CommMAXRetry 40
#define TriggerPin 13
 
#define urmAdd 0x11
 
// Command list
 
#define AddComm 0xab
#define DisComm 0x02
#define TempComm 0x03
#define MaxDisComm 0x04
 
/******************** Variables ****************/
 
byte readingStep;
byte cmdst[10];
unsigned int urmData;
unsigned long managerTimer = 20;
 
/******************** Functions ****************/
 
void initAddr(); 
void urmInit();
void runUrm05();
void urmTrigger(int id);
void decodeURM05();
void transmitCommands();
void ValidateCmd(byte cmd[]);
 
/****************** Init sensor ****************/
 
void urmInit(){
   
  pinMode(TriggerPin,OUTPUT);  // TTL -> RS485 chip driver pin on Arduino IO expansion shield v5
  digitalWrite(TriggerPin,LOW);// Turn the driver pin to LOW  -> Turn on reading mode for the RS485 interface
                               // Turn the drvier pin to HIGH -> Turn on code transmitting mode for the RS485 interface
  readingStep = 0;
  managerTimer = millis();
   
  SerialPort.begin(19200);               // Init the RS485 interface
                                         // Also when you are driving the URM04, you could open serial monitor to 
                                         // tracing the steps and data feedback from URM04
//  SerialPort.println(" ");
//  SerialPort.println("The default baudrate: 19200");
//  SerialPort.println("Start setting the 485 sensors Address!");
  for(int i = 0 ;i < 10; i++)  cmdst[i] = 0;  //init the URM04 protocol
  cmdst[0]=0x55;  
  cmdst[1]=0xaa;
   
  initAddr();
//  SerialPort.print("Address:");
//  SerialPort.println(urmAdd);
  delay(100);
   
}
 
void initAddr(){
   
  cmdst[2] = AddComm;
  cmdst[3] = 0x01;
  cmdst[4] = 0x55;
  cmdst[5] = urmAdd;
  cmdst[6] = cmdst[0]+cmdst[1]+cmdst[2]+cmdst[3]+cmdst[4]+cmdst[5];
//  delay(1);
  for(int j = 0; j < 7; j++){
    printByte(cmdst[j]);
//    delayMicroseconds(50);
  }
  Serial.flush();
//  delay(10);
   
}
 
/********************* Drive URM05 and get the data code *************************/
 
void runUrm05(){                 // You could adjust the sensor measuring rate by changing the managerTimer value
 
  static unsigned long timer = 0;
  static int num = 0;           // Set the URM05 id to be drived
   
  if(millis() - timer > managerTimer){
    digitalWrite(TriggerPin, HIGH);  // Turn on transmitting mode for the RS485 interface
    switch(readingStep){
 
    case 0:
      urmTrigger(urmAdd);
      managerTimer = 0;            // set a interval after trigger the measuring
       
      break;
 
    case 1:
      digitalWrite(TriggerPin, LOW); // Turn on reading mode for the RS485 interface
      managerTimer = 300;            // control the flash rate for reading sensor value
      break;
       
    default:
      readingStep = 0;               // Finish reading the distance and start a new measuring for the sensor
      break;
    }
     
    if(readingStep < 1)  readingStep++;  //step manager
    else readingStep = 0;
 
    timer = millis();
//    digitalWrite(TriggerPin, LOW); //Turn on reading mode for the RS485 interface
  }
 
}
 
/********************* Transmit Command via the RS485 interface ***************/
 
void urmTrigger(int id){  // The function is used to trigger the measuring
  cmdst[2] = id;
  cmdst[3] = 0x00;
  cmdst[4] = DisComm;
  transmitCommands();
  //  SerialPort.println("Trigger!");
}
 
 
void transmitCommands(){  // Send protocol via RS485 interface
  cmdst[5]=cmdst[0]+cmdst[1]+cmdst[2]+cmdst[3]+cmdst[4];
  delay(1);
  for(int j = 0; j < 6; j++){
    printByte(cmdst[j]);
//    delayMicroseconds(10);
  }
//  delay(3);
  Serial.flush();
}
 
 
/********************* Receive the data and get the distance value from the RS485 interface ***************/
 
void decodeURM05(){
   
  if(SerialPort.available()){
     
    unsigned long timerPoint = millis();
     
    int RetryCounter = 0;
    byte cmdrd[10];
    for(int i = 0 ;i < 10; i++)  cmdrd[i] = 0;
    int i=0;
//    SerialPort.println("OK");
 
    boolean flag = true;
    boolean valid = false;
    byte headerNo = 0;
 
    while(RetryCounter < CommMAXRetry && flag)
    {
       
      if(SerialPort.available()){
        cmdrd[i]= SerialPort.read();
//        printByte(cmdrd[i]);
         
        if(i > 7){
          flag=false;
          break;
        }
        if(cmdrd[i] == 0xAA){
          headerNo = i;
          valid = true;
        }
        if(valid && i == headerNo + 6){
          flag = false;
          break;
        }
        i ++;
        RetryCounter = 0;
      }
      else{
        RetryCounter++;
        delayMicroseconds(15);
      }
    }
     
//    printByte(millis() - timerPoint);
 
    if(valid)  ValidateCmd(cmdrd);
    else printByte(0xEC);  //Get an invalid error command
     
  }
   
}
 
void ValidateCmd(byte cmd[]){
   
  byte sumCheck = 0;
  for(int h = 0;h < 7; h ++)  sumCheck += cmd[h];
   
  if(sumCheck == cmd[7] && cmd[3] == 2 && cmd[4] == 2){
     
    byte id = cmd[2] - urmAdd;
    urmData = cmd[5] * 256 + cmd[6];
     
//    SerialPort.print(id);
//    SerialPort.print(":");
//    SerialPort.println(urmData);
     
  }
  else if(cmd[3] == 2 && cmd[4] == 2){
    SerialPort.print("Sum error");
  }
   
}