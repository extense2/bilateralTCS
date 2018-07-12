/*
 * Project name   : iGrasp (Robot/slave)
 * Description    : This code implements a bilateral teleoperation system involving a 3D printed, actuated mechatronic hand (i.e., the slave) fitted with Force Sensitive Resistors at the fingertips,
 *                  and a smart glove (i.e., the master) fitted with flex sensors and haptic feedback Eccentric Rotation Mass motors at each fingertip. 
 *                  Wireless communication is achieved by means of a Bluetooth 2.0 link, using off-the-shelf HC-05 modules. 
 *                  The STL files for the hand are based on the work by Sonia Verdu, which are available online. 
 *                  The STL of the fingertips were extensively modified to accomodate the FSRs and to allow the mechatronic hand to detect touch from a range of angles. 
 * 
 * Platform       : Arduino Uno
 * Compile date   : 24 May 2018
 * Author         : Kevin Too
*/

// ==================================================================================================================================  
//                                              Libraries & Definitions
// ==================================================================================================================================  

// Haptics
  #include <Wire.h>                       // i2c library.
  #define FSR_DELAY                 200   // FSR checking interval (milliseconds). This gives a max Tx rate of 5Bytes every 200ms, if all FSRs are triggered simultaneously = 25 Bytes per second.  
  #define FSR_CALIBRATION_PERIOD   5000   // Duration of FSR calibration period (milliseconds).
  #define CALIBRATION_BARS           50   // Defines the length of the progress bar used in the FSR calibration phase.
  #define MINMAX_FSR_TRESHOLD       800   // minmax_FSR_difference refers to the difference between the max and min ADC values recorded during calibration, summed over all five fingers. 
  #define MINMAX_FSR_TYPICAL_LIMIT 3000   // If minmax_FSR_difference is less than MINMAX_FSR_TRESHOLD, the calibration must be repeated. The max average value for the minmax_FSR_difference is given by MINMAX_FSR_TYPICAL_LIMIT.

// Bluetooth
  #include <sSoftSerial.h>                // This is a modified version of SoftSerial.h, and makes use of Timer2. The unmodified SoftwareSerial.h clashes with the Servo Library.
  #define BT_MIN                      0   // Minimum value of packet transmitted over bluetooth.
  #define BT_MAX                    125   // Maximum value of packet transmitted over bluetooth (excludes START and END characters).
  #define BT_START                  126   // START character, used to indicate the start of a bluetooth sequence.
  #define BT_END                    127   // END character, used to indicate the end of a bluetooth sequence.
  #define BT_TX_RATE                200   // How often to transmit data over bluetooth (milliseconds).
  #define BT_TX_DELAY              1000   // For stability: Bluetooth transmission delay (microseconds). 
  
// Servos
  #include <Servo.h> 
  #define NS                          5   // Number of Servos/Fingers present
  #define SERVO_MIN                   0   // Minimum servo angle (degrees)
  #define SERVO_MAX                 180   // Maximum servo angle (degrees)
  #define SERVO_RATE                 10   // Time interval for writing to servos (milliseconds)
  #define HANDWAVE_COUNT             90   // Servo initialization test: This is the angle to move the servos, in degrees.
  #define SERVO_TEST_DELAY           10   // Servo initialization test: Loop delay

// Servo Controller
  #define PROPORTIONAL_CONSTANT     0.2   // Servo controller gain
  
// Misc
  #define SERIAL_DEBUG                4   // 1: Calibration, 2: Rx'd Flex, 3: writeServo, 4: handWave, 5: Tx'd FSR
  #define SETUP_DELAY               800   // Control overall setup duration/speed
  #define CURRENT_BUILD    "24 May 2018"
  
// ==================================================================================================================================  
//                                     Global variables and object instantiations
// ==================================================================================================================================  

// Haptics
  uint8_t analog_pin[NS]  = {A0, A1, A2, A3, A4};            // Define Analog pins wired to the FSRs.
  uint8_t analogvalMax[NS]= {0};                             // Corresponds to maximum finger movement recorded during FSR calibration. Initialized to 0.
  uint8_t analogvalMin[NS]= {1023, 1023, 1023, 1023, 1023};  // Corresponds to minimum finger movement recorded during FSR calibration. Initialized to 1023.
  
// Bluetooth
  sSoftSerial BTSerial(8,9);                                 // Initialise the sSoftSerial Library with the RX/TX pins
  unsigned long timestampBT = 0;

// Servos
  Servo M[NS];                                               // Create 5 servo objects, one per finger
  int BTread[NS] = {0};                                      // For storing incoming bluetooth data
  unsigned long timestampSERVO = 0;                          // To control servo refresh rate

// ==================================================================================================================================  
//                                                         SETUP
// ==================================================================================================================================  

void setup(){
  Serial.begin(9600);
  BTSerial.begin(9600);
  
  Serial.println();
  Serial.println(F("Begin Setup.."));
  Serial.println();
  delay(SETUP_DELAY);

  ServoAttach();
  delay(SETUP_DELAY);

  ServoTest();
  delay(SETUP_DELAY);

  calibrateFSR();
  Serial.println(F("Setup complete."));
  delay(SETUP_DELAY*3);
}

// ==================================================================================================================================  
//                                                           LOOP
// ==================================================================================================================================  

void loop(){
  bluetoothRX0(); 
  
  if (millis()-timestampBT > BT_TX_RATE){
    bluetoothTX0();
    timestampBT = millis();
  }
  if (millis()-timestampSERVO > SERVO_RATE){
    writeServo();
    timestampSERVO = millis();
  }
}

// ==================================================================================================================================  
//                                            SETUP functions: Servo & FSR Calibration
// ==================================================================================================================================  

void ServoAttach(){
  for(uint8_t i=0;i<NS;i++){
    M[i].attach(i+2);                                             // Attach servos to corresponding digital pins
    pinMode(i+2, OUTPUT);                                         // Initialise pins 2-6 as outputs for the servos
    M[i].write(0);                                                // Reset servo position (i.e., fully extend all fingers)
  }
  Serial.println(F("Servos attached."));
}

void ServoTest(){
  uint8_t k         = 0;                                          // Finger position counter.
  uint8_t countup   = 1;                                          // Directional flag.
  Serial.print(F("Testing servo actuation..."));                  // Flex and extend all fingers simultaneously.
  for(uint8_t i=0; i<(HANDWAVE_COUNT*2); i++){                    // Multiply by 2 to account for opening and closing of hand.
    for(uint8_t j=0;j<NS;j++)                                     // Update each servo sequentially.
      M[j].write(k);                                              // Write position to servo.
    if(countup == 1) k++; else k--;                               // Increment/Decrement position.
    if (k==HANDWAVE_COUNT) countup = 0;                           // Toggle direction if limit of motion is reached.
    if (k==0)  countup = 1;                                       // Toggle direction if limit of motion is reached.
    delay(SERVO_TEST_DELAY);                                      // Limit finger movement speed.
  }
  Serial.println(F(" Ok"));  
}

void calibrateFSR(){ 
  uint16_t timestamp_FSRCalibration  = millis();                   // Calibration duration offset.
  uint16_t timestamp_progressBar     = millis();                   // Progress bar timing.
  uint16_t analogval[NS]             = {0};                        // ADC value corresponding to finger positions.
  float minmax_FSR_difference        =  0;                         // Difference between analogvalMax and analogvalMin, summed over all fingers.
  float percentageVal                =  0;                         // Measure of how much the FSRs have been pressed during calibration, expressed as a percentage of typical maximum.
  const uint8_t default_FSR_MIN[5]   = {65, 150, 60, 100, 85};     // Obtained by running several cycles of FSR-calibration (without pressing the FSRs) and finding the average. 

#if (SERIAL_DEBUG>=1)
  Serial.println();
  Serial.println(F("Starting FSR sensor calibration..."));
  Serial.println(F("Please press on each FSR sensor in turn."));
  Serial.print(F("|"));
  for (uint8_t i=0; i<(CALIBRATION_BARS-3); i++)                                        // Print a line to show how long the progress bar is.
  Serial.print(F("-"));  
  Serial.println(F("|"));
#endif  

  while(millis() < (FSR_CALIBRATION_PERIOD+timestamp_FSRCalibration)){                  // This is the actual calibration section.
    for(uint8_t i=0;i<NS;i++){
      analogval[i]=analogRead(analog_pin[i]);
      if (analogval[i]>analogvalMax[i]){ analogvalMax[i] = analogval[i]; }              // Keep track of max analogval.
      if (analogval[i]<analogvalMin[i]){ analogvalMin[i] = analogval[i]; }              // Keep track of min analogval.
    }
    if((millis()-timestamp_progressBar)>(FSR_CALIBRATION_PERIOD/CALIBRATION_BARS)){     // Display progress bar.
      #if (SERIAL_DEBUG>=1)
        Serial.print("|");
      #endif  
      timestamp_progressBar = millis();
    }
  }

  for(uint8_t i=0; i<NS; i++){
    if(analogvalMin[i]<default_FSR_MIN[i]){       // Enforce a lower bound on analogvalMin to compensate for mechanical imperfections in the FSR cover. Otherwise, false positives will occur frequently. 
      analogvalMin[i] = default_FSR_MIN[i];
    }
  }
  
  #if (SERIAL_DEBUG>=1)                           // Print results of calibration on serial monitor.
    Serial.println("");
    Serial.println(F("Calibration complete."));
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
    Serial.println("");
  #endif  

  for (uint8_t i = 0; i<NS; i++){
    minmax_FSR_difference +=  (analogvalMax[i]-analogvalMin[i]);           // Calculate difference between max and min. Sum over all fingers.
  }
  percentageVal = (minmax_FSR_difference/MINMAX_FSR_TYPICAL_LIMIT)*100;    // Express minmax_FSR_difference as a % of typical maximum.

  #if (SERIAL_DEBUG>=1)
    Serial.println();
    Serial.print(F("Sum of MinMax_Difference: "));
    Serial.print(minmax_FSR_difference,0);
    Serial.println();
    Serial.print(F("Typical Maximum: "));
    Serial.println(MINMAX_FSR_TYPICAL_LIMIT);
    Serial.println();
  
    if (minmax_FSR_difference < MINMAX_FSR_TRESHOLD){                       // A low minmax_FSR_difference will result in lower haptic resolution.
      Serial.println(F("Calibration not conducted properly. "));
      Serial.print(F("The FSRs have only been pressed to about "));
      Serial.print(percentageVal,0);
      Serial.println(F("% of the typical user's maximum."));
      Serial.println(F("Press on the FSRs with greater force next time."));
      Serial.println(F("Restart program to continue."));
      while(1);
    } else {
      Serial.println(F("FSR calibration complete."));    
      Serial.println("");    
    }
  #endif  
}

// ==================================================================================================================================  
//                                              LOOP functions: Bluetooth TX/RX
// ==================================================================================================================================  

void bluetoothTX(){                                                     // Test function - Transmits sequence of integers from 0 to 100.
  static int i = 0;
  BTSerial.write(i++);
  if (i==101) i=0;
}
void bluetoothRX(){                                                     // Test function - Prints received data on the serial monitor.  
  if (BTSerial.available())
    Serial.println(BTSerial.read());
}

void bluetoothTX0(){                                                    // Read FSRs, format bytes and transmit.
  uint8_t FSR[NS]       = {0};                                          // Variable to store the FSR reading for each finger.
  uint8_t flag_FSR      =  0;                                           // Flag to monitor FSR-presses.
  const uint8_t FSR_MIN =  0;                                           // FSR value will be mapped to the range from FSR_MIN to FSR_MAX (for bluetooth transmission).
  const uint8_t FSR_MAX = 24;    

  for (uint8_t i=0; i<NS; i++){
    FSR[i]=analogRead(analog_pin[i]);
    FSR[i]=constrain(FSR[i],analogvalMin[i],analogvalMax[i]);           // Ensure values fall within valid range.
    FSR[i]=map(FSR[i],analogvalMin[i],analogvalMax[i],FSR_MIN,FSR_MAX); // Map values to FSR range (the HC-05 bluetooth chip transmits 7-bit packets, with values ranging from 0 to 127). 
    flag_FSR = flag_FSR | FSR[i];                                       // Bitwise-OR to check if any one of the FSRs triggered.
    FSR[i] += ((FSR_MAX+1)*(i));                                        // Thumb = 0-24, index = 25-49, middle = 50-74, ring = 75-99, pinky = 100-124. Two of the remaining 3 possible values (125, 126) are used for start and stop characters. 
  }
  if(flag_FSR > 0){                                                     // If a valid FSR-press has been detected...
    #if (SERIAL_DEBUG==5)
      Serial.print(F("Tx: "));
      Serial.print("\t");
    #endif
    
    for (uint8_t i=0; i<NS; i++){                                       // ...transmit the value over bluetooth.
      BTSerial.write(FSR[i]);
      delayMicroseconds(BT_TX_DELAY); 
      #if (SERIAL_DEBUG==5)
        Serial.print(FSR[i]);
        if(i<(NS-1)) Serial.print("\t");
        else Serial.println("");
      #endif        
    }
    flag_FSR = 0;                                                       // Reset data-received flag.
  }
}

void bluetoothRX0(){                                                 // Function is implemented with a finite state machine (FSM).
  uint8_t tempChar;                                                  // Buffer for received bluetooth data.
  uint8_t FLEX_index = 0;                                            // Finger/Servo index for received data.
  uint8_t FSM_state  = 0;                                            // Keep track of FSM state.

  switch(FSM_state){
    case 0: if (BTSerial.available()){                               // Check for incoming data.
              tempChar = BTSerial.read();                            // Read one byte at a time.
              if (tempChar == BT_START)                              // If BT_START char detected, change state.
                FSM_state = 1;
            }
            break;  

    case 1: if (BTSerial.available()){
              tempChar = BTSerial.read();
              if (tempChar != BT_START && tempChar != BT_END){        // Check if byte is valid
                BTread[FLEX_index] = tempChar;                        // Store valid byte and move to next finger
                FLEX_index++;
              }
              else if (tempChar == BT_START) FLEX_index = 0;          // If BT_START char detected, next char corresponds to first finger (out of 5)
              if (FLEX_index == NS){                                  // Once a byte has been received for each finger...
                FLEX_index = 0;                                       // ...reset index for next sequence,
                #if (SERIAL_DEBUG==2)                                 // ...and print received data on serial monitor.
                  Serial.print(F("Rx: "));                          
                  Serial.print("\t");
                  for (uint8_t i=0; i<NS; i++){
                    Serial.print(BTread[i]);
                    if(i<(NS-1)) Serial.print("\t");
                    else Serial.println("");
                  }
                #endif
              FSM_state = 0;                                          // Return to state 0
              }
            }
            break;
  
    default: FSM_state = 0;
             break;          
  } // end switch
}   // end function

// ==================================================================================================================================  
//                                             LOOP functions: Write Servo
// ==================================================================================================================================  

void writeServo(){
  uint8_t currentPosition[NS]        = {0};                // Current Servo motor position (0 to 180)
  uint8_t nextPosition[NS]           = {0};                // Next Servo motor position (0 to 180)
  int16_t errorPosition[NS]          = {0};                // Error in Servo position (-180 to 180)

  #if (SERIAL_DEBUG==3)
    Serial.print(F("Writing to Servos: "));
    Serial.print("\t");
  #endif

  for(uint8_t i=0;i<NS;i++){
    currentPosition[i] = map(BTread[i], BT_MIN ,BT_MAX , SERVO_MIN, SERVO_MAX);   // Map data received over bluetooth onto the range of motion of servos.
    if(currentPosition[i]!=nextPosition[i]){                                      // Check if current position differs from next position.
      errorPosition[i] = currentPosition[i]-nextPosition[i];                      // Find error between current and desired position.
      nextPosition[i] += round(PROPORTIONAL_CONSTANT*errorPosition[i]);           // Implement proportional controller.
    }    
    else{
      nextPosition[i] = currentPosition[i];                                       // If no difference between current and desired position, stay at current position.
    }
        
    M[i].write(nextPosition[i]);                                                  // Write desired position to servos.
    #if (SERIAL_DEBUG==3)                                                         // Print newly-written position to serial monitor
      Serial.print(nextPosition[i]);
      if (i<(NS-1)) Serial.print("\t");
      else Serial.println("");
    #endif
  }
}

// ==================================================================================================================================  

