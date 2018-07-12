/*
 * Project name   : iGrasp (Glove/master)
 * Description    : This code implements a bilateral teleoperation system which consists of a 3D printed, actuated mechatronic hand (i.e., the slave) fitted with Force Sensitive Resistors (FSRs) at the fingertips
 *                  and a smart glove (i.e., the master) fitted with flex sensors and haptic feedback Eccentric Rotation Mass (ERM) motors at each fingertip. 
 *                  Wireless communication is achieved by means of a Bluetooth 2.0 link, using off-the-shelf HC-05 modules. 
 *                  The STL files for the hand are based on the work by Sonia Verdu, which are available online. 
 *                  The STL of the fingertips were extensively modified to accommodate the FSRs and to allow the mechatronic hand to detect touch from a range of angles. 
 * 
 * Platform       : Arduino Nano
 * Compile date   : 24 May 2018
 * Author         : Kevin Too
*/

// ==================================================================================================================================  
//                                              Libraries & Definitions
// ==================================================================================================================================  

// Haptics
  #include <Wire.h>
  #include <Adafruit_DRV2605.h>

// Bluetooth
  #include <SoftwareSerial.h>
  #define BT_START                    126   // Start character for each transmitted bluetooth sequence
  #define BT_RX_PRINTRATE             100   // Time interval to print received bluetooth data on the serial monitor (milliseconds) 
  #define FSR_MAX                      24   // The magnitude of the haptic effect is mapped to this maximum value (on the slave)

// Flex
  #define NS                            5   // Number of Servos
  #define FLEX_CALIBRATION_PERIOD    4000   // Flex sensor calibration time (milliseconds)
  #define CALIBRATION_BARS             50   // Progress bar; Used by both Flex and Servo calibration
  #define MINMAX_FLEX_TRESHOLD        680   // Total finger movement treshold. Below this, user must repeat calibration. 
  #define MINMAX_FLEX_TYPICAL_LIMIT  1700   // Typical sum for the range of finger flexion
  #define SERVO_CALIBRATION_PERIOD   5000   // Servo motor calibration time (milliseconds)
  #define BT_MAX                      125   // data transmitted over bluetooth limited to 7 bits
  #define CALIBRATION_BARS             50   // Progress bar; Used by both Flex and Servo calibration


// Statistics
  #include "Statistic.h"
  #define AVERAGING_WINDOW           100  //  Number of points over which to calculate the average flex/servo position  

// debugging
  #define LED_HIGH                   120  // Blinking led HIGH time (used in setup only)
  #define LED_LOW                    120  // Blinking led LOW time (used in setup only)
  #define SERIAL_DEBUG                 2  // 1: Calibration, 2: Print Rx FSR data, 3: Loop Timing, 4: Print Tx Flex data
  #define  SETUP_DELAY               200  // For giving the user enough time to follow the setup process
  #define CURRENT_BUILD     "24 May 2018"   
  
// ==================================================================================================================================  
//                                     Global variables and object instantiations
// ==================================================================================================================================  

// Bluetooth
  SoftwareSerial BTSerial(10, 11);                        // UART RX, TX

// Flex sensors / Servos
  int analogPin[NS]    = {A0, A1, A2, A3, A6};            // A4 and A5 are reserved for I2C, and cannot be used as analog inputs.
  int analogvalMax[NS] = {0};                             // for flex sensor calibration; initialized to 0
  int analogvalMin[NS] = {1023, 1023, 1023, 1023, 1023};  // for flex sensor calibration; initialized to 1023
  int servoMax[NS]     = {0};                             // for servo motor calibration; initialized to 0

// Haptics
  Adafruit_DRV2605 drv;                    // One instantiation is enough to handle all 5 haptic drivers (because they are multiplexed) 
  int hapticValues[NS] = {0};               // Array of 5 elements to store haptic values to be sent to the DRV2605 chips via the multiplexer

// Statistics
  Statistic myStat1, myStat2, myStat3, myStat4, myStat5;  // Statistic objects - used to determine moving average for noisy ADC values
  
// ==================================================================================================================================  
//                                                         SETUP
// ==================================================================================================================================  

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

  double_blink(2);                                   // Signal user that haptic drivers will be tested next
  setupDRV2605();  
  delay(SETUP_DELAY);
  testDRV2605();                                     // Test each driver sequentially

  Serial.print(F("Clearing stats... "));
  stats_clear();
  Serial.println(F("ok"));
  Serial.println();
  delay(SETUP_DELAY);

  double_blink(2);                                   // Signal user that Flex calibration is next
  calibrateFlex();
  delay(SETUP_DELAY);

  double_blink(2);                                   // Signal user that Servo calibration is next    
  calibrateServo();
  delay(SETUP_DELAY); 

  double_blink(2);                                   // Signal user that Setup is complete 
  Serial.println(F("Setup complete."));  
  delay(SETUP_DELAY); 
}

// ==================================================================================================================================  
//                                                           LOOP
// ==================================================================================================================================  

void loop(){
  static uint32_t timestamp_BT_Rx  = millis();       // UUsed to control how often printBluetoothRx() is called
  #if (SERIAL_DEBUG==3)
    static uint32_t timestamp_Loop = millis();       // Loop timing
  #endif
  
  bluetoothRX0();                                    // Read incoming FSR data
  bluetoothTX0();                                    // Send flex data
  writeERM();                                        // Write haptic data to ERM motors

  #if (SERIAL_DEBUG==2)
    if (millis()-timestamp_BT_Rx > BT_RX_PRINTRATE){ // Limit rate of printing on Serial Monitor 
      printBluetoothRx();
      timestamp_BT_Rx = millis();
    }
  #endif

  #if (SERIAL_DEBUG==3)
    Serial.println(millis()-timestamp_Loop);         // Loop timing
  #endif  
}
// ==================================================================================================================================  
//                                                SETUP functions: Blink & DRV2605
// ==================================================================================================================================  

void double_blink(int k){                   // Blinks in 2 sets of k short pulses. Each set is 500ms apart.
  for (uint8_t i=0; i<1; i++){
    for (uint8_t j=0; j<k; j++){
      digitalWrite(LED_BUILTIN, HIGH); 
    delay(LED_HIGH);
      digitalWrite(LED_BUILTIN, LOW); 
    delay(LED_LOW);
    }
    delay(500);  
  }
}

void setupDRV2605(){
  for(uint8_t i=0;i<NS;i++){
    tcaselect(i);                   // Select correct multiplexer channel
    if(!drv.begin()){                 // Attempt to initialize each driver individually
      Serial.print(F("Failed to initialize driver "));  
    Serial.println(i);
      Serial.println("..."); 
      Serial.print(F("Please restart program."));
      while(1);                     // Stop execution if driver failed to initialize
    }
  }
  Serial.println(F("Drivers initialized."));
  for(uint8_t m=0;m<NS;m++){
    tcaselect(m);
    drv.selectLibrary(1);                               // Argument is the library identifier. Choose 1 for ERMs.

    drv.setWaveform(0, 47);                             // The first argument of setWaveform() is the haptic playlist index (ranging from 0 to 7 inclusive).  
    drv.setWaveform(1, 0);                              // The second argument is the waveform number (ranging from 0 to 123 inclusive - see section 12.1.2 of the DRV2605L datasheet by Texas Instruments), choose 0 to denote end of playlist.

/*
    drv.setWaveform(0, 47);                             // Alternate example: 
  drv.setWaveform(1, 42);                             // Uncomment this commented block...
    drv.setWaveform(2, 51);                             // ...to setup 4 waveforms to be played sequentially every time the 
    drv.setWaveform(3, 90);                             // ...drv.go() command is executed.
    drv.setWaveform(4, 0);                              // End haptic playlist
*/
  }
  Serial.println(F("Libraries and Effects selected."));
}

void testDRV2605(){                                     // Trigger a haptic effect on each ERM sequentially
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

// ==================================================================================================================================  
//                                        SETUP functions: Calibration
// ==================================================================================================================================  

void calibrateFlex(){
  uint16_t analogval[NS]                   =   {0};       // Array to store ADC values for each flex sensor
  float MINMAX_FLEX_DIFFERENCE             =    0;        // Variable to track total amount of flexion (summed over all fingers) during calibration
  float percentageVal                      =    0;        // MINMAX_FLEX_DIFFERENCE expressed as a percentage of MINMAX_FLEX_TYPICAL_LIMIT
  uint32_t timestamp_FlexProgressBar       = millis();    // Timestamp for timing of progress bar 
  const uint32_t timestamp_FlexCalibration = millis();    // Keep track of total elapsed time at the start of calibration 

  #if (SERIAL_DEBUG>=1)
    Serial.println(F("Starting flex sensor calibration..."));
    Serial.println(F("Please open and close your gloved hand repeatedly for about 5 seconds."));
    Serial.print(F("|"));
    for (uint8_t i=0; i<(CALIBRATION_BARS-3); i++)
      Serial.print(F("-"));  
    Serial.println(F("|"));
  #endif  

  while(millis() < FLEX_CALIBRATION_PERIOD+timestamp_FlexCalibration){                 // Main flex calibration loop
    for(uint8_t i=0;i<NS;i++){                                                           
      analogval[i]=analogRead(analogPin[i]);                                             
      if (analogval[i]>analogvalMax[i]){ analogvalMax[i] = analogval[i]; }               // Keep track of max analogval
      if (analogval[i]<analogvalMin[i]){ analogvalMin[i] = analogval[i]; }               // Keep track of min analogval
    }
    if((millis()-timestamp_FlexProgressBar)>(FLEX_CALIBRATION_PERIOD/CALIBRATION_BARS)){ // Display progress bar
      #if (SERIAL_DEBUG>=1)
        Serial.print("|");
      #endif  
      timestamp_FlexProgressBar = millis();
    }
  }

  #if (SERIAL_DEBUG>=1)                                           // Print results of calibration on serial monitor.
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
    MINMAX_FLEX_DIFFERENCE +=  (analogvalMax[i]-analogvalMin[i]);          // Cumulative sum of flexion for all fingers
  }
  percentageVal = (MINMAX_FLEX_DIFFERENCE/MINMAX_FLEX_TYPICAL_LIMIT)*100;  // Calculate percentage flexion 

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

void calibrateServo(){                                                              // Determine max safe range of motion of servos given the mechanical constraints of the prototype
  int analogval[NS]                         =   {0};                                // Array to store ADC values for each flex sensor
  uint32_t timestamp_ServoProgressBar       = millis();                             // Timestamp for timing of progress bar
  const uint32_t timestamp_ServoCalibration = millis();                             // For timing of servo range calibration 

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

  while(millis() < SERVO_CALIBRATION_PERIOD+timestamp_ServoCalibration){
    for (uint8_t i=0; i<NS; i++){
      analogval[i] = analogRead(analogPin[i]);                                      // Read analogue flex sensor value
      analogval[i] = constrain(analogval[i],analogvalMin[i],analogvalMax[i]);       // Limit servo position to the previously calibrated flex sensor range
      analogval[i] = map(analogval[i],analogvalMin[i],analogvalMax[i], 0, BT_MAX);  // Format values for transmission over bluetooth
      stats_update(i,analogval[i]);                                                 // Calculate running average of servo position (to smooth out ADC noise). Only send the average values to the slave.
      if(analogval[i]>servoMax[i]) servoMax[i] = analogval[i];                      // Keep track of maximum servo positions. This is to prevent servos from over flexing (i.e., excessive mechanical stresses)
    }

    if((millis()-timestamp_ServoProgressBar)>((SERVO_CALIBRATION_PERIOD)/CALIBRATION_BARS)){ // Print progress bar 
    #if (SERIAL_DEBUG>=1)
      Serial.print("|");
    #endif  
      timestamp_ServoProgressBar = millis();
    }

    if(myStat1.count()==AVERAGING_WINDOW){                                          // Once the averaging window is full...
      for (uint8_t i=0; i<NS; i++){                                                 
        if(i==0)    BTSerial.write(BT_START);                                       // ...write a Start character at the beginning of each transmission
                    BTSerial.write(round(stats_getMean(i)));                        // ...and transmit the average of the past X values (as defined by AVERAGING_WINDOW). 
      }                                                                             
      stats_clear();                                                                // Reset stats for next window
    }      
  }
  Serial.println('\n');
  Serial.println(F("Servo calibration complete."));
}

// ==================================================================================================================================  
//                                          LOOP functions: TX/RX
// ==================================================================================================================================  
void bluetoothTX(){                           // Test function - Transmits sequence of integers from 0 to 100.
  static int i = 0;                             
  BTSerial.write(i++);                        
  if (i==100) i = 0;                            
}                                             
void bluetoothRX(){                           // Test function - Prints received data on the serial monitor.  
  if (BTSerial.available()) 
    Serial.println(BTSerial.read());
}

void bluetoothRX0(){                          // Extracts haptic data from incoming bluetooth data
  uint8_t char1;                              // Buffer for received bluetooth data.
  uint8_t index;                              // Finger index
  uint8_t hapticVal;                          // Magnitude of haptic effect corresponding to the finger index
                                            
  if (BTSerial.available()){                  // Check for incoming data
    char1 = BTSerial.read();                  // Read one byte at a time
    if (char1<BT_START){                      // If byte is valid...
      hapticVal = char1%(FSR_MAX+1);          // hapticVal ranges from 0 to (FSR_MAX+1).
      index = (char1-hapticVal)/(FSR_MAX+1);  // index = 0 (thumb), 1 (index), 2 (middle), 3 (ring), 4 (pinky)
      hapticValues[index] = hapticVal;        // Store haptic values in array
    }  
  }
}

void printBluetoothRx(){                      // Print data received over bluetooth, in rows of 5 bytes. 
  Serial.print(F("Rx:   "));
  Serial.print("\t");
  for (uint8_t i=0; i<NS; i++){
    Serial.print(hapticValues[i]);
    if(i<(NS-1)) Serial.print("\t");
    else Serial.println("");
  }
}                                                           

void bluetoothTX0(){                                                                  // Read and format flex sensor values. Transmit 5 bytes at a time.  
  int analogval[NS] = {0};                                                            // Array to store ADC values for each flex sensor
  for (uint8_t i=0; i<NS; i++){
    analogval[i] = analogRead(analogPin[i]);
    analogval[i] = constrain(analogval[i],analogvalMin[i],analogvalMax[i]);
    analogval[i] = map(analogval[i],analogvalMin[i],analogvalMax[i], 0, servoMax[i]); // Use ServoMax as maximum to avoid overactuating the servos.
    stats_update(i,analogval[i]);                                                     // Add current reading to stats object (to calculate moving average)
  }
  
  if(myStat1.count()==AVERAGING_WINDOW){                                              // Once the averaging window is full...
    #if (SERIAL_DEBUG == 4)                                                           
      Serial.print("Tx:    ");                                                        
    #endif                                                                            
    for (uint8_t i=0; i<NS; i++){                                                     
      if(i==0)    BTSerial.write(BT_START);                                           // ...write a start byte at the beginning of each transmission
                  BTSerial.write(round(stats_getMean(i)));                            // ...and transmit the average of the past X values (as defined by AVERAGING_WINDOW) for each finger. The round() function is required because stats_getMean() returns a float.
      #if (SERIAL_DEBUG == 4)                                                         
        Serial.print(round(stats_getMean(i)));                                        
        if(i<(NS-1)) Serial.print("\t");                                              
        else Serial.println("");                                                      
      #endif                                                                          
    }                                                                                 
    stats_clear();                                                                    // Reset stats for next window
  }
}

// ==================================================================================================================================  
//                                                LOOP functions: Haptics
// ==================================================================================================================================  
void writeERM(){                                           // Start playback of preset haptic playlist on ERMs
  static uint16_t delayValue[NS]        = {0};              // Array of delay values for each ERM. Delay depends on how strongly the FSRs of the slave are pressed. (Higher pressure = Smaller delay)
  static uint16_t previousFSRMillis[NS] = {0};              // Array for FSR timing
                                                         
  for (uint8_t i=0; i<NS; i++){                            // Each function call updates all the haptic drivers sequentially 
    tcaselect(i);                                          // Select correct multiplexer channel
    delayValue[i] = haptic2delay(hapticValues[i]);         // Convert haptic value (ranges from 0 to 24) into haptic delay (ranges from 200ms to 4s)
    if(millis() - previousFSRMillis[i] > delayValue[i]){   // Each finger's haptic firing rate is controlled independently of the other fingers.
      drv.go();
      previousFSRMillis[i] = millis();
    }
  }
}

int haptic2delay(int sensorval){                           // sensorval must be <= FSR_MAX (currently set at 24)
  int delayval;                                            // As sensorval increases, select stronger waveforms from the DRV2605 library AND decrease delay between firings.
  if      (sensorval < 3){                                 // i.e., Stronger FSR readings = stronger AND more frequent ERM action.
          drv.setWaveform(0, 3); delayval=4000;}
  else if (sensorval < 5){
          drv.setWaveform(0, 11); delayval=1000;}
  else if (sensorval < 8){
          drv.setWaveform(0, 49); delayval=800;}
  else if (sensorval < 11){
          drv.setWaveform(0, 49); delayval=600;}
  else if (sensorval < 14){
          drv.setWaveform(0, 49); delayval=500;}
  else if (sensorval < 17){
          drv.setWaveform(0, 49); delayval=400;}
  else if (sensorval < 20){
          drv.setWaveform(0, 49); delayval=300;}
  else if (sensorval < 24){
          drv.setWaveform(0, 49); delayval=200;}
  else{
          drv.setWaveform(0, 58); delayval=200;} 
  return delayval;  
}
  
void tcaselect(uint8_t i) {                                // For selecting the active channel (0 through 7) of the TCA9548A i2c Multiplexer
  const int TCAADDR     = 0x70;                            // Depends on how the A0, A1, A2 on the mux are hardwired. 
  const int MUX_SHIFT   =    3;                            // Depends on which pins of the i2c mux are wired to the drivers.
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << (i+MUX_SHIFT));
  Wire.endTransmission();
}

// ==================================================================================================================================  
//                                               LOOP functions: Statistics
// ==================================================================================================================================  
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

void stats_clear(){         // Clear all stats. Reset all means to zero
  myStat1.clear();
  myStat2.clear();  
  myStat3.clear();  
  myStat4.clear();  
  myStat5.clear();  
}

// ==================================================================================================================================  

