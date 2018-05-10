uint8_t c = 0;

//===========================================================     Libraries & Definitions
// Haptics
  #include <Wire.h>
  #include <Adafruit_DRV2605.h>
  #define FSR_MAX           24
  #define FSR_TRESHOLD       4    // upper range of values of FSR_MAX for which the DRV waveform changes (to stronger waveform). 
  #define ERM_RATE          50
  #define ERM_MAX         2000    // y-intercept of line for lower range of FSR values (below FSR_MAX-FSR_TRESHOLD). 
  #define ERM_MIN          100  
  #define ERM_MULTIPLIER    75
  #define ERM_MIN_TRESHOLD   1
  #define TCAADDR         0x70    // Depends on how A0-A2 is hardwired. 
  #define MUX_SHIFT          3

// Bluetooth
  #include <SoftwareSerial.h>
  #define BT_MAX           125
  #define BT_MIN             0
  #define BT_START         126
  #define BT_END           127
  #define BT_TX_RATE        50 
  #define BT_RX_RATE       100

// Flex
  #define NS                           5 // Number of Servos
  #define FLEX_CALIBRATION_PERIOD   4000
  #define SERVO_CALIBRATION_PERIOD 10000
  #define CALIBRATION_BARS            50  // Used by both Flex and Servo calibration
  #define MINMAX_FLEX_TRESHOLD       680
  #define MINMAX_FLEX_TYPICAL_LIMIT 1700
  #define ANALOGREAD_DEFAULT_MAX     850
  #define ANALOGREAD_DEFAULT_MIN     250

// Statistics
  #include "Statistic.h"
  #define STATS_MEAN_PRECISION 1    // Number of decimal places to print for the mean
  #define STATS_SD_PRECISION   1    // Number of decimal places to print for the Std. Dev.
  #define AVERAGING_WINDOW   100    // Number of points over which to average   

// debugging
  #define SERIAL_DEBUG         2 // 1: Calibration, 2: Rx'd FSR, 3: Loop Timing, 4: Tx'd Flex
  #define SETUP_DELAY        200
  #define LED_HIGH           120
  #define LED_LOW            120  
  #define CURRENT_BUILD   "07 May 2018"
  
//===========================================================     Declarations
// Bluetooth
  SoftwareSerial BTSerial(10, 11); // RX | TX

// Flex
  int millis_FLEX_CALIBRATION;          // Calibrate Servo
  int millis_SERVO_CALIBRATION;         // Calibrate Flex
  unsigned long previousFlexMillis = 0; // For timing of Flex sensor readings
  int Pin[NS]={A0, A1, A2, A3, A6};      // A4 and A5 are used for I2C
  int analogval[NS]={0}; 
  int analogvalMax[NS]={0}; 
  int analogvalMin[NS]={1023, 1023, 1023, 1023, 1023}; // temp
  int ServoMax[NS]={0}; 
  int Mpos[NS]={0};
//  int i = 0;      // used by bluetoothTX()

// ERMs, Mux
  Adafruit_DRV2605  drv;
  int BTread[NS] = {0};
  int FSR_index = 0;
  uint8_t ERMcounter = 0;
  int delayValue[NS]={0};
  unsigned long previousFSRMillis[NS] = {0}; // for FSR delay
  unsigned long currentMillis[NS]  = {0}; // Replace this by millis() to free SRAM?

// Statistics
  Statistic myStat1, myStat2, myStat3, myStat4, myStat5;

// Misc
  unsigned long millis_BT_TX = 0; // Used by bluetoothTX0()
  unsigned long millis_BT_RX = 0; // Used by printBluetoothRx()
  unsigned long millis_LOOP = 0; //
  unsigned long timestamp_FLEX_CALIBRATION =0;
  unsigned long timestamp_SERVO_CALIBRATION =0;
  uint8_t setup_delay_count = 0; 
  
//===========================================================     SETUP
void setup(){
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println();
  Serial.println(F("iGrasp project - Glove code"));
  Serial.print(F("Current Build: "));
  Serial.println(CURRENT_BUILD);
  Serial.println();
  delay(SETUP_DELAY); 
  
  Serial.print(F("Starting Bluetooth communications... "));
  BTSerial.begin(9600);
  Serial.println(F("ok"));
  delay(SETUP_DELAY);

  double_blink(3);    // Signal user that haptic drivers will be tested next
  setupDRV2605();  
  delay(SETUP_DELAY);
  testDRV2605();

  Serial.print(F("Clearing stats... "));
  stats_clear();
  Serial.println(F("ok"));
  Serial.println();
  delay(SETUP_DELAY);

  double_blink(3);    // Signal user
  calibrateFlex();
  delay(SETUP_DELAY);

  double_blink(3);    // Signal user that haptic drivers will be tested next
  calibrateServo();
  delay(SETUP_DELAY); 

  Serial.println(F("Setup complete."));  
  delay(SETUP_DELAY); 
}

//===========================================================     LOOP
void loop(){
  #if (SERIAL_DEBUG==3)
    millis_LOOP = millis();
  #endif
  
  bluetoothRX0();
  bluetoothTX0();
  writeERM();

  #if (SERIAL_DEBUG==2)
    if (millis()-millis_BT_RX > BT_RX_RATE){
      printBluetoothRx();
      millis_BT_RX = millis();
    }
  #endif

  #if (SERIAL_DEBUG==3)
    Serial.println(millis()-millis_LOOP);
  #endif  
}

//=========================================================== SETUP functions: DRV2605
void double_blink(int k){
  for (uint8_t i=0; i<2; i++){
    for (uint8_t j=0; j<k; j++){
      digitalWrite(LED_BUILTIN, HIGH); delay(LED_HIGH);
      digitalWrite(LED_BUILTIN, LOW); delay(LED_LOW);
    }
    delay(500);  
  }
}

void setupDRV2605(){
  for(uint8_t i=0;i<NS;i++){
    tcaselect(i);
    if(!drv.begin()){
      Serial.print(F("Failed to initialize driver "));  Serial.println(i);
      Serial.println("..."); 
      Serial.print(F("Please restart program."));
      while(1);
    }
  }
  Serial.println(F("Drivers initialized."));
  for(uint8_t m=0;m<NS;m++){
    tcaselect(m);
    drv.selectLibrary(1);     // Library #1 is for ERMs.
    drv.setWaveform(0, 47);   // first digit: index, from 0 to 7. Second digit: waveform number, choose 0 to denote end. 

/*  drv.setWaveform(1, 42);   // For example: 
    drv.setWaveform(2, 51);   // uncomment this section (and comment out the line after this block)
    drv.setWaveform(3, 90);   // to setup 4 waveforms to be played sequentially every time the drv.go() command is executed.
    drv.setWaveform(4, 0);    // End waveform
*/
    drv.setWaveform(1, 0);    // End waveform
  }
  Serial.println(F("Libraries and Effects selected."));
}

void testDRV2605(){
  Serial.println(F("Testing each DRV2605 in turn..."));
  for (uint8_t i=0; i<NS; i++){
    Serial.print(F("Testing #"));
    Serial.print(i);
    Serial.print(F(": "));
    tcaselect(i);
    drv.go();
    Serial.println(F("ok"));
    delay(1000);
  }
  Serial.println(F("DRV2605 testing complete."));
  Serial.println();
}

//========================================================== SETUP functions: Calibration

void calibrateFlex(){ // add feature: let user know that calibration is in progress by blinking LED (the serial monitor won't always be available)
  float MINMAX_FLEX_DIFFERENCE=0;
  float percentageVal=0;
  timestamp_FLEX_CALIBRATION= millis();
  millis_FLEX_CALIBRATION = millis();
  
  #if (SERIAL_DEBUG>=1)
    Serial.println(F("Starting flex sensor calibration..."));
    Serial.println(F("Please open and close your gloved hand repeatedly for about 5 seconds."));
    Serial.print(F("|"));
    for (uint8_t i=0; i<(CALIBRATION_BARS-3); i++)
      Serial.print(F("-"));  
    Serial.println(F("|"));
  #endif  

  while(millis() < FLEX_CALIBRATION_PERIOD+timestamp_FLEX_CALIBRATION){
    for(uint8_t i=0;i<NS;i++){
      analogval[i]=analogRead(Pin[i]);
      if (analogval[i]>analogvalMax[i]){ analogvalMax[i] = analogval[i]; }
      if (analogval[i]<analogvalMin[i]){ analogvalMin[i] = analogval[i]; }
    }
    if((millis()-millis_FLEX_CALIBRATION)>(FLEX_CALIBRATION_PERIOD/CALIBRATION_BARS)){
      #if (SERIAL_DEBUG>=1)
        Serial.print("|");
      #endif  
      millis_FLEX_CALIBRATION = millis();
    }
  }

  #if (SERIAL_DEBUG>=1)  
    Serial.println();
    Serial.println();
    Serial.print(F("Maximum ADC values:\t"));
    for(uint8_t i=0; i<NS; i++){
      Serial.print(analogvalMax[i]);
      if (i<(NS-1)){Serial.print('\t');}
      else {Serial.println("");}
    }
    Serial.print(F("Minimum ADC values:\t"));
    for(uint8_t i=0; i<NS; i++){
      Serial.print(analogvalMin[i]);
      if (i<(NS-1)){Serial.print('\t');}
      else {Serial.println("");}
    }
  #endif
  
  for (uint8_t i = 0; i<NS; i++){
    MINMAX_FLEX_DIFFERENCE +=  (analogvalMax[i]-analogvalMin[i]);
  }
  percentageVal = (MINMAX_FLEX_DIFFERENCE/MINMAX_FLEX_TYPICAL_LIMIT)*100; 

  #if (SERIAL_DEBUG>=1)  
    Serial.println();
    Serial.print(F("Sum of MinMax_Difference: "));
    Serial.print(MINMAX_FLEX_DIFFERENCE,0);
    Serial.println();
    Serial.print(F("Typical Maximum: "));
    Serial.println(MINMAX_FLEX_TYPICAL_LIMIT);
    Serial.println();
  
    if (MINMAX_FLEX_DIFFERENCE < MINMAX_FLEX_TRESHOLD){
      Serial.println(F("Calibration not conducted properly. "));
      Serial.print(F("You have only moved about "));
      Serial.print(percentageVal,0);
      Serial.println(F("% of the typical user's maximum range."));
      Serial.println(F("Move your fingers through a wider range next time."));
      Serial.println(F("Restart program to continue."));
      while(1);
    } else {
      Serial.println(F("Flex sensor calibration complete."));    
      Serial.println("");    
    }
  #endif  
}

void calibrateServo(){
  millis_SERVO_CALIBRATION = millis();
  timestamp_SERVO_CALIBRATION = millis();  
  
#if (SERIAL_DEBUG>=1)
  Serial.println(F("Starting Servo sensor calibration... "));
  Serial.println(F("Actuate each finger of the robot until they are at "));
  Serial.println(F("their maximum flexion, but do NOT flex more than that."));
  Serial.println(F("This will set the maximum range of motion of the servos."));
  Serial.print(F("|"));
  for (uint8_t i=0; i<(CALIBRATION_BARS-3); i++){
    Serial.print(F("-"));  
  }
  Serial.println(F("|"));
#endif  

  while(millis() < SERVO_CALIBRATION_PERIOD+timestamp_SERVO_CALIBRATION){
    for (uint8_t i=0; i<NS; i++){
      analogval[i] = analogRead(Pin[i]);
      analogval[i] = constrain(analogval[i],analogvalMin[i],analogvalMax[i]);
      Mpos[i]      = map(analogval[i],analogvalMin[i],analogvalMax[i], BT_MIN, BT_MAX);
      stats_update(i,Mpos[i]);
      if(Mpos[i]>ServoMax[i]) ServoMax[i] = Mpos[i]; // keep track of max servo position
    }

    if((millis()-millis_SERVO_CALIBRATION)>((SERVO_CALIBRATION_PERIOD)/CALIBRATION_BARS)){
    #if (SERIAL_DEBUG>=1)
      Serial.print("|");
    #endif  
      millis_SERVO_CALIBRATION = millis();
    }

    if (millis()-millis_BT_TX > BT_TX_RATE){          // Limit rate of BT transmission 
      for (uint8_t i=0; i<NS; i++){                   // 5 bytes transmitted at a time
        if(i==0)    BTSerial.write(BT_START);         // add a start byte at the beginning of each transmission
                    BTSerial.write(stats_getMean(i)); // Transmit the average of the past X values (as defined by AVERAGING_WINDOW). 
      }
      millis_BT_TX = millis();
    }
      
    if(myStat1.count()==AVERAGING_WINDOW){          // Reset stats once the window is full
      stats_clear();
    }
  }
  Serial.println('\n');
  Serial.println(F("Servo calibration complete."));
}

//=========================================================== LOOP functions: TX/RX
void bluetoothTX(){               // for simple bluetooth Tx troubleshooting
  static int i=0;
  BTSerial.write(i++);
  if (i==100) i=0;
}
void bluetoothRX(){               // for simple bluetooth Rx troubleshooting
  if (BTSerial.available()) 
    Serial.println(BTSerial.read());
}

void bluetoothRX0(){              // Read one char from BT buffer
  uint8_t char1, index, val;
  if (BTSerial.available()){
    char1 = BTSerial.read();
    if (char1<BT_START){
      val = char1%(FSR_MAX+1);          // Decode byte
      index = (char1-val)/(FSR_MAX+1);  // index = 0 (thumb), 1 (index), 2 (middle), 3 (ring), 4 (pinky)
      BTread[index] = val;              // Store haptic measurement in array
    }  
  }
}

void printBluetoothRx(){              // Print data received over bluetooth, in rows of 5 bytes. 
  Serial.print(F("Rx:   "));
  Serial.print("\t");
  for (uint8_t i=0; i<NS; i++){
    Serial.print(BTread[i]);
    if(i<(NS-1)) Serial.print("\t");
    else Serial.println("");
  }
}

void bluetoothTX0(){                  // Read and format flex sensor vaues. Transmit 5 bytes at a time.  
  for (uint8_t i=0; i<NS; i++){
    analogval[i] = analogRead(Pin[i]);
    analogval[i] = constrain(analogval[i],analogvalMin[i],analogvalMax[i]);
    Mpos[i]      = map(analogval[i],analogvalMin[i],analogvalMax[i], BT_MIN, ServoMax[i]); // use ServoMax as maximum to avoid overactuating the servos. 
    stats_update(i,Mpos[i]);    // add current reading to data pool (to calculate average and std. deviation)
  }
  
  if (millis()-millis_BT_TX > BT_TX_RATE){          // Limit rate of BT transmission 
    #if (SERIAL_DEBUG == 4)
      Serial.print("Tx:    ");
    #endif
    for (uint8_t i=0; i<NS; i++){                   // 5 bytes transmitted at a time
      if(i==0)    BTSerial.write(BT_START);         // add a start byte at the beginning of each transmission
                  BTSerial.write(round(stats_getMean(i))); // Transmit the average of the past X values (as defined by AVERAGING_WINDOW). 
      #if (SERIAL_DEBUG == 4)
        Serial.print(round(stats_getMean(i)));
        if(i<(NS-1)) Serial.print("\t");
        else Serial.println("");
      #endif
      millis_BT_TX = millis();
    }
  }
    
  if(myStat1.count()==AVERAGING_WINDOW){          // Reset stats once the window is full
    stats_clear();
  }
}

//=========================================================== LOOP functions: Haptics
void writeERM(){                        // Triggers the ERMs
  for (uint8_t i=0; i<NS; i++){         // Each function call updates all the haptic drivers sequentialy 
    tcaselect(i);
    delayValue[i] = sensor2delay(BTread[i]);              // Firing rate depends on FSR readings. 
//    if(millis() - previousFSRMillis[i] > delayValue[i]){  // More pressure = more frequent firings. 
    if(millis() - previousFSRMillis[i] > 2000){  // More pressure = more frequent firings. 
      drv.go();
      previousFSRMillis[i] = millis();
    }
  }
}

int sensor2delay(int sensorval){            // sensorval must be <= FSR_MAX (currently set at 24)
  int delayval;                             // As sensorval increases, select stronger waveforms from the DRV2605 library AND decrease delay between firings.
  if      (sensorval < 4){                  // i.e., Stronger FSR readings = stronger, more frequent ERM action.
          drv.setWaveform(0, 3); delayval=4000;}
  else if (sensorval < 8){
          drv.setWaveform(0, 11); delayval=850;}
  else if (sensorval < 12){
          drv.setWaveform(0, 49); delayval=700;}
  else if (sensorval < 16){
          drv.setWaveform(0, 49); delayval=550;}
  else if (sensorval < 20){
          drv.setWaveform(0, 48); delayval=400;}
  else if (sensorval < 24){
          drv.setWaveform(0, 48); delayval=300;}
  else if (sensorval = 24){
          drv.setWaveform(0, 58); delayval=200;}
  else
          drv.setWaveform(0, 58); delayval=200;
  return delayval;  
}
  
void tcaselect(uint8_t i) {   // for selecting the active channel (0 through 7) of the TCA9548A i2c Multiplexer
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << (i+MUX_SHIFT));
  Wire.endTransmission();
}

//===========================================================     LOOP functions: Statistics
void stats_update(int i, int val){      // Add one data point to the ith stats instance
  switch(i){
    case 0:
      myStat1.add(val);
      break;
    case 1:
      myStat2.add(val);
      break;
    case 2:
      myStat3.add(val);
      break;
    case 3:
      myStat4.add(val);
      break;
    case 4:
      myStat5.add(val);
      break;
    default:
      break;   
  }
}

float stats_getMean(int i){     // return the average of the ith instance
  switch(i){
    case 0:
      return(myStat1.average());
      break;
    case 1:
      return(myStat2.average());
      break;
    case 2:
      return(myStat3.average());
      break;
    case 3:
      return(myStat4.average());
      break;
    case 4:
      return(myStat5.average());
      break;
    default:
      break;   
  }
}

float stats_getStdDev(int i){     // return the Std. Dev. of the ith instance
  switch(i){
    case 0:
      return(myStat1.pop_stdev());
      break;
    case 1:
      return(myStat2.pop_stdev());
      break;
    case 2:
      return(myStat3.pop_stdev());
      break;
    case 3:
      return(myStat4.pop_stdev());
      break;
    case 4:
      return(myStat5.pop_stdev());
      break;
    default:
      break;   
  }
}

void stats_clear(){         // Clear all stats. Reset all means and Std. Devs. to zero
  myStat1.clear();
  myStat2.clear();  
  myStat3.clear();  
  myStat4.clear();  
  myStat5.clear();  
}
