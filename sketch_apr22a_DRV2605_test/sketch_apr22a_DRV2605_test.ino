/* 
 *  You don't need that much data resolution for the FSR values. If you have 8 levels of pressure, you only need 3 bits per finger.  Total = 15 bits (2 bytes!)
 *  Right now, you are sending 8 bits per finger. Total: 40 bits (5 bytes)
 *  
 *  Do you realize the similarities between the two sides? the FSR and the flex sensors. You cound just write one master function to sample and transmit. 
 *  bearing in mind that FSR needs lower resolution. 
 *  
 *    
 *  What's the BTserial buffer size? 64KB?
 *    As the robot transmits data, this is stored in a buffer. 
 *    When BTserial.read() is called, if reads from that buffer, emptying one char at a time. 
 *    If you are filling the buffer faster than you are emptying it, then you will lose data...right?
 *    https://internetofhomethings.com/homethings/?p=927
 *  
 *  
 *  Instead of constantly transmitting the FSR values, why not Tx only when the FSRs are pressed?
 *  Instead of relying on the sequence of the received bytes to determine which number corresponds to which finger, 
 *  why not encode the finger number within the transmitted byte itself? eg. 
 *    Thumb:  10~19 
 *    Index:  20~29 
 *    Middle: 30~39 etc..
 *  It will be necessary to reduce the resolution of the haptic information (0-9, i.e., ten levels of force) 
 *  but I think that should be an acceptable trade-off. Then write a function on the Glove side to decode the received bytes. 
 *  UPDATE:  
 *    Yes, this seems very promising!
 *    When an FSR is pressed, the BT buffer fills up VERY quickly, based on the number of times the loop has executed. 
 *    Implement a function to only check the FSRs every xxx milliseconds, so the buffer fills up with one single set of values 
 *    every xxx ms. Use the millis() funstion. 
 *    Use an array to keep track of time elapsed since each FSR was read. 
 *    
 *  Max value that can be written by BT is 8 bits (int 255)  
 */

// Haptics
  #include <Wire.h>
  #include <Adafruit_DRV2605.h>
  #define TCAADDR 0x70 //Depends on how A0-A2 is hardwired. 
  #define FSR_MAX      24
  #define FSR_TRESHOLD 8
  #define ERM_MAX      450
  #define ERM_MIN      100
  
// Bluetooth
  #include <SoftwareSerial.h>
  #define BT_MAX 125
  #define BT_MIN 0
  #define BT_START 126
  #define BT_END 127

// Flex
  #define CALIBRATION_PERIOD 3500
  #define FLEX_DELAY    200      // FSR checking interval (milliseconds). This gives a max Tx rate of 5Bytes every 200ms, if all FSRs are triggered simultaneously = 25 Bytes per second.  

// Misc
  #define SERIAL_DEBUG 3 // 1: Loop Timing, 2: Flex, 3: FSR

//================================================================================
// Haptics
  Adafruit_DRV2605  drv;
  uint8_t FSR_index = 0;
  uint8_t FSR_val = 0;
  unsigned long previousFSRMillis[5] = {0}; // for FSR delay
  unsigned long currentMillis[5]  = {0}; // Replace this by millis() to free SRAM?
  int FSR[5] = {0};
  int delayValue[5]={0};

// Bluetooth 
  SoftwareSerial    BTserial(10,11);

// Flex



//================================================================================
void setup() {
  Serial.begin(9600);
  BTserial.begin(9600);
  Serial.println("Pre drv");
  setupDRV2605();
  Serial.println("Post drv");
}

//================================================================================
void loop(){
  for (uint8_t i=0; i<2; i++){
    Serial.print(".");
    tcaselect(i);
    drv.go();
    delay(500);
  }
  Serial.println();  
}

//================================================================================

void writeERM(){
  for (uint8_t i=0; i<5; i++){
    delayValue[i]=sensor2delay(FSR[i]);
    currentMillis[i] = millis();
    if(currentMillis[i] - previousFSRMillis[i] > delayValue[i]){
      tcaselect(i);
      drv.go();
      previousFSRMillis[i] = currentMillis[i];
    }
  }
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++

int sensor2delay(int sensorval){ // sensorValue must be <= FSR_MAX
  int delayval;
  if(sensorval<(FSR_MAX-FSR_TRESHOLD)){
    delayval = ERM_MAX-sensorval*20; // This is an equation describing how delayValue changes with respect to sensorValue. 
    drv.setWaveform(0, 10);
  } else if (sensorval<=FSR_MAX){
    delayval = ERM_MIN;               // For large sensorValues, use a minimum delay of 100ms.  
    drv.setWaveform(0, 13);
  }
  return delayval; // in milliseconds
}
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setupDRV2605(){
  for(uint8_t i=0;i<5;i++){
    Serial.println("Pre tcaselect(i)");
    tcaselect(i);
    Serial.println("Post tcaselect(i)");
    if(!drv.begin()){
      Serial.print(F("Failed to initialize driver "));  Serial.println(i);
      Serial.println("..."); 
      Serial.print(F("Please restart program."));
      while(1);
    } else     Serial.println("Post if");

  }
  Serial.println(F("Drivers initialized."));
  for(uint8_t m=0;m<5;m++){
    tcaselect(m);
    drv.selectLibrary(1);     // Select Libraries and effects ############### DOES THIS NEED TO BE EXECUTED FOR EACH DRV? #######
    drv.setWaveform(0, 47);   // first digit: index, from 0 to 7. Second digit: waveform number, choose 0 to denote end. 
    drv.setWaveform(1, 0);    // end waveform
  }
  Serial.println(F("Libraries and Effects selected."));
}

