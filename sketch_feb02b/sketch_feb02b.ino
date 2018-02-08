/* Copy of Glove code 
 *  [fixed] No response when sending AT commands 
 *  
 *  One problem is the start char '<' 
 *      It is transmitted as a byte. Its decimal value is 60. 
 *      The program cannot differentiate between a start char and a legit FSR value of 60.
 *      Solution: the mapped FSR value is constrained to a max of 99, so you can use anything larger than that as the start marker
 *      
 *  The -1 problem: The robot also periodically sends a value of -1, which seems to be interpreted as 255 in the glove side. (2's complement)
 *      either look into why the robot is sending -1. Is the -1 from the map() function? or from analogueread()?
 *      OR... ignore it on the glove side (current solution)
 *  
 *  On the glove side, Why is -1 sometimes printed as -1, and sometimes as 255?
 *  
 *  You don't need that much data resolution for the FSR values. If you have 8 levels of pressure, you only need 3 bits per finger.  Total = 15 bits (2 bytes!)
 *  Right now, you are sending 8 bits per finger. Total: 40 bits (5 bytes)
 *  
 *  Do you realize the similarities between the two sides? the FSR and the flex sensors. You cound just write one master function to sample and transmit. 
 *  bearing in mind that FSR needs lower resolution. 
 *  
 *  
 *  On ROBOT code: 
 *    use byte unstead of int for FSR[i] ? 
 *    FSR is currently int, so can be negative. 
 *    
 *  Well, FSR is int in the glove code as well... so why are the -1's displayed as 255?   
 *  
 *  What's the BTserial buffer size? 64KB?
 *    As the robot transmits data, this is stored in a buffer. 
 *    When BTserial.read() is called, if reads from that buffer, emptying one char at a time. 
 *    If you are filling the buffer faster than you are emptying it, then you will lose data...right?
 *    https://internetofhomethings.com/homethings/?p=927
 *  
 */

#include <Wire.h>
#include <Adafruit_DRV2605.h>
#define TCAADDR 0x70 //Depends on how A0-A2 is hardwired. 
#include <SoftwareSerial.h>

SoftwareSerial BTserial(10,11);
Adafruit_DRV2605 drv;

unsigned long previousMillis[5] = {0};
int i=0;
int i2 = 0;
int data = 0;
int Channel[5]={0, 1, 2, 3, 4};
int FSR[6]={0};
int delayValue[5]={0};
unsigned long currentMillis[5]={0};
int Pin[5]={A0, A1, A2, A3, A6};
int Mpos[5]={0};

void tcaselect(uint8_t j) {
  if (j > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

int sensor2delay(int sensorValue){ // sensor1Value must be in range 0-100
  int delayValue;
  if      (sensorValue < 10)  {delayValue = 400;  drv.setWaveform(0, 10);} // Low pressure, long delay
  else if (sensorValue < 20)  {delayValue = 350;  drv.setWaveform(0, 10);}
  else if (sensorValue < 30)  {delayValue = 300;  drv.setWaveform(0, 10);}
  else if (sensorValue < 40)  {delayValue = 250;  drv.setWaveform(0, 10);}
  else if (sensorValue < 50)  {delayValue = 200;  drv.setWaveform(0, 10);}
  else if (sensorValue < 60)  {delayValue = 150;  drv.setWaveform(0, 10);}
  else if (sensorValue < 70)  {delayValue = 200;  drv.setWaveform(0, 10);}
  else if (sensorValue < 100) {delayValue = 100;  drv.setWaveform(0, 13);} // Large pressure, small delay
  return delayValue;
}


void setup() {
  Serial.begin(9600);
  BTserial.begin(9600);
  for(int k=0;k<5;k++){
    tcaselect(Channel[k]);
    if(!drv.begin()){
      Serial.print("Failed to initialize driver ");  Serial.println(k);
      while(1);
    }
  }
  Serial.println("Drivers initialized.");
  for(int m=0;m<5;m++){
    tcaselect(Channel[m]);
    //Select Libraries
    drv.selectLibrary(Channel[m]);
    //Select Effects
    drv.setWaveform(0, 10);   // first digit: index, from 0 to 7. Second digit: waveform number, choose 0 to denote end. 
    drv.setWaveform(1, 0);    // end waveform
  }
  Serial.println("Libraries and Effects selected.");
}


//void loop(){
//  while(i<5){
//    int temp = BTserial.read();
//    if(temp==255||temp==-1){}
//    else if(temp == 222){data = 1;}
//    else{
//      if(data){
//        FSR[i]=temp;
//        Serial.print(FSR[i]);
//        Serial.print('\t');
//        i++;
//      }
//    }
////
////      if (FSR[i] > 5){ // if FSR pressed
//////        tcaselect(Channel[i]);
//////        delayValue[i]=sensor2delay(FSR[i]);
//////        currentMillis[i] = millis();
////        if(currentMillis[i] - previousMillis[i] > delayValue[i]){
//////          tcaselect(Channel[i]);
//////          drv.go();
//////          previousMillis[i] = currentMillis[i];
////        }    // end if
////      }
//  }
//  data = 0;
//  i = 0;
//  Serial.println("");  
//
//  for(int n=0;n<5;n++){
////    Mpos[n]=analogRead(Pin[n]);
////    Mpos[n]=constrain(Mpos[n],200,580);
////    Mpos[n]=map(Mpos[n],200,580, 0, 127);
//   
//    if(n==0){BTserial.write(100);}
////    BTserial.write(Mpos[n]);
//  }
//}

void loop(){
  receiveFSRData();
  printData();
  }
  
// function to receive blocks of 5 valid bytes
void receiveFSRData(){
  static byte ndx = 0;
  char endMarker = '>';
  char startMarker = '<';
  char rc;
  boolean messageInProgress = false;
  
  while(BTserial.available()>0 && messageInProgress  == false){
    rc = BTserial.read();
    dataReceived = 1;
    
    
    }
  
  }

// function to print data received
void printData(){
  if(dataReceived == 1){
    Serial.print(rc);
    dataReceived = 0;
    }
  
  }

