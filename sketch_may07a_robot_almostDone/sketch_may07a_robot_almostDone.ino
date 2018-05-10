//===========================================================     Libraries & Definitions
// Haptics
  #include <Wire.h>
  #define FSR_MIN                     0
  #define FSR_MAX                    24
  #define FSR_DELAY                 200  // FSR checking interval (milliseconds). This gives a max Tx rate of 5Bytes every 200ms, if all FSRs are triggered simultaneously = 25 Bytes per second.  
  #define FSR_CALIBRATION_PERIOD   5000
  #define CALIBRATION_BARS           50  // 
  #define MINMAX_FSR_TRESHOLD       800
  #define MINMAX_FSR_TYPICAL_LIMIT 3000

// Bluetooth
  #include <sSoftSerial.h>
  #define BT_MIN          0
  #define BT_MAX        125
  #define BT_START      126
  #define BT_END        127
  #define BT_TX_RATE    200 
  #define BT_TX_DELAY  1000
  
// Servos
  #include <Servo.h> 
  #define NS               5        // Number of Servos
  #define SERVO_MIN        0        // Minimum servo angle (degrees)
  #define SERVO_MAX      180        // Maximum servo angle (degrees)
  #define JITTER_VAL       1        // Half-window size for validating servo-writes (degrees)
  #define SERVO_STEP_SIZE  2        // Servo position can only change by one step size at a time (degrees)
  #define SERVO_RATE      10        // Time interval for writing to servos (milliseconds)
  #define HANDWAVE_COUNT  90        // For Setup. This is the angle to move the servos, in degrees.
  #define HANDWAVE_CYCLES  1        // For Setup
  #define PROPORTIONAL_CONSTANT 0.2 // 
  
// Statistics
  #include "Statistic.h"

// Misc
  #define SERIAL_DEBUG  1 // 1: Calibration, 2: Rx'd Flex, 3: writeServo, 4: handWave, 5: Tx'd FSR
  #define SETUP_DELAY 800
  
//===========================================================     Declarations

// Haptics
  int Pin[NS]={A0, A1, A2, A3, A4};                     // Analogue inputs (FSR)
  int FSR[NS]={0};                                      // Variable to store the FSR reading for each finger
  unsigned int previousMillis[NS] = {0};                // For timing of FSR readings
  unsigned int millis_FSR_CALIBRATION;                  // Calibrate Flex
  int analogval[NS]={0};                                // Calibrate Flex
  int analogvalMax[NS]={0};                             // Calibrate Flex
  int analogvalMin[NS]={1023, 1023, 1023, 1023, 1023};  // Calibrate Flex
  int flag_FSR = 0;
  const int default_FSR_MIN[5] = {65, 150, 60, 100, 85};  // obtained by running several cycles of FSR-calibration (without pressing the FSRs) and finding the average. 
  const int default_FSR_MAX[5] = {850, 550, 700, 750, 800};  // obtained by running several cycles of FSR-calibration (without pressing the FSRs) and finding the average. 
  
// Bluetooth
  sSoftSerial BTSerial(8,9);                  // Initialise the sSoftSerial Library with the RX/TX pins
  unsigned long timestampBT =0;

// Servos
  Servo M[NS];                                // Create 5 servo objects, one per finger
  int BTread[NS] = {0};
  uint8_t Mpos[NS]={0};                       // Next Servo position (0 to 180)
  uint8_t Current_pos[NS]={0};                // Current Servo position (0 to 180)
  int16_t Error_pos[NS]={0};                  // Error in Servo position (-180 to 180)
  double Error_sum[NS] = {0};                 // Integral of error 
  byte FLEX_index = 0;
  byte FSM_state  = 0;
  uint8_t flag = 0;
  uint8_t k=0;                                // Position counter for servo test during setup 
  uint8_t countup=1;                          // Directional flag for servo test during setup 
  unsigned long timestampSERVO =0;            // To control servo refresh rate
  unsigned long timestamp_FSR_CALIBRATION =0; // For accurate calibration duration

// Statistics
  Statistic myStat1, myStat2, myStat3, myStat4, myStat5;

//===========================================================     SETUP
void setup(){
  Serial.begin(9600);
  BTSerial.begin(9600);
  
  Serial.println();
  Serial.println(F("Begin Setup"));
  Serial.println();
  delay(SETUP_DELAY);
  
  ServoAttach();
  delay(SETUP_DELAY);

  ServoTest();
  delay(SETUP_DELAY);

  //calibrateFSR();
  FSR_defaults();
  Serial.println(F("Setup complete."));  
  delay(SETUP_DELAY*3);
}

//===========================================================     LOOP
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

//=========================================================== SETUP functions: Servo & FSR Calibration
void ServoAttach(){
  for(uint8_t i=0;i<NS;i++){
    M[i].attach(i+2);       // Attach servos to corresponding digital pins
    pinMode(i+2, OUTPUT);   // Initialise pins 2-6 as outputs for the servos
    M[i].write(0);          // reset servo position
  }
  Serial.println(F("Servo attached."));
}

void ServoTest(){
  Serial.print(F("Testing servo actuation..."));  
  for(uint8_t i=0; i<(HANDWAVE_COUNT*HANDWAVE_CYCLES*2); i++){
    for(uint8_t j=0;j<NS;j++)
      M[j].write(k);
    if(countup == 1) k++; else k--;
    if (k==HANDWAVE_COUNT) countup = 0;
    if (k==0)  countup = 1;
    delay(10);
  }
  Serial.println(F(" Ok"));  
}

void calibrateFSR(){ // add feature: let user know that calibration is in progress by blinking LED (the serial monitor won't always be available)
  float MINMAX_FSR_DIFFERENCE=0;
  float percentageVal=0;
  millis_FSR_CALIBRATION = millis();
  timestamp_FSR_CALIBRATION = millis();

#if (SERIAL_DEBUG>=1)
  Serial.println();
  Serial.println(F("Starting 5-second FSR sensor calibration..."));
  Serial.println(F("Please press on each FSR sensor in turn."));
  Serial.print(F("|"));
  for (uint8_t i=0; i<(CALIBRATION_BARS-3); i++)
    Serial.print(F("-"));  
  Serial.println(F("|"));
#endif  

  while(millis() < (FSR_CALIBRATION_PERIOD+timestamp_FSR_CALIBRATION)){
    for(uint8_t i=0;i<NS;i++){
      analogval[i]=analogRead(Pin[i]);
      if (analogval[i]>analogvalMax[i]){ analogvalMax[i] = analogval[i]; }
      if (analogval[i]<analogvalMin[i]){ analogvalMin[i] = analogval[i]; }
    }
    if((millis()-millis_FSR_CALIBRATION)>(FSR_CALIBRATION_PERIOD/CALIBRATION_BARS)){
      #if (SERIAL_DEBUG>=1)
      Serial.print("|");
      #endif  
      millis_FSR_CALIBRATION = millis();
    }
  }

  for(uint8_t i=0; i<NS; i++){
    if(analogvalMin[i]<default_FSR_MIN[i]){
      analogvalMin[i] = default_FSR_MIN[i];
    }
  }
  
  #if (SERIAL_DEBUG>=1)  
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
    MINMAX_FSR_DIFFERENCE +=  (analogvalMax[i]-analogvalMin[i]);
  }
  percentageVal = (MINMAX_FSR_DIFFERENCE/MINMAX_FSR_TYPICAL_LIMIT)*100; 

  #if (SERIAL_DEBUG>=1)  
    Serial.println();
    Serial.print(F("Sum of MinMax_Difference: "));
    Serial.print(MINMAX_FSR_DIFFERENCE,0);
    Serial.println();
    Serial.print(F("Typical Maximum: "));
    Serial.println(MINMAX_FSR_TYPICAL_LIMIT);
    Serial.println();
  
    if (MINMAX_FSR_DIFFERENCE < MINMAX_FSR_TRESHOLD){
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

void FSR_defaults(){
    for(uint8_t i=0; i<NS; i++){
      analogvalMin[i] = default_FSR_MIN[i];
      analogvalMax[i] = default_FSR_MAX[i];      
  }
}

//=========================================================== LOOP functions: TX/RX
void bluetoothTX(){                 // Transmit sequence of integers from 0 to 100
  static int i = 0;
  BTSerial.write(i++);
  if (i==101) i=0;
}
void bluetoothRX(){                 // Print whatever is received on serial monitor  
  if (BTSerial.available())
    Serial.println(BTSerial.read());
}

void bluetoothTX0(){                // Read FSRs, format bytes and transmit 
  for (uint8_t i=0; i<NS; i++){
    FSR[i]=analogRead(Pin[i]);
    FSR[i]=constrain(FSR[i],analogvalMin[i],analogvalMax[i]);
    FSR[i]=map(FSR[i],analogvalMin[i],analogvalMax[i],FSR_MIN,FSR_MAX);
    flag_FSR = flag_FSR | FSR[i];                // bitwise OR to check if any one of the FSRs triggered
    FSR[i] += ((FSR_MAX+1)*(i));         // thumb = 0-24, index = 25-49, middle = 50-74, etc...
  }
    if(flag_FSR > 0){

      #if (SERIAL_DEBUG==5)
        Serial.print(F("Tx: "));
        Serial.print("\t");
      #endif
      
      for (uint8_t i=0; i<NS; i++){
        BTSerial.write(FSR[i]);
        delayMicroseconds(BT_TX_DELAY); 
        #if (SERIAL_DEBUG==5)
          Serial.print(FSR[i]);
          if(i<(NS-1)) Serial.print("\t");
          else Serial.println("");
        #endif        
      }
      flag_FSR = 0;
//      #if (SERIAL_DEBUG==3) 
//        Serial.println("Tx."); 
//      #endif
    }
}
void bluetoothRX0(){              // 
  uint8_t char1;
  switch(FSM_state){
    case 0: if (BTSerial.available()){   // No incoming data. Keep Checking.  
              char1 = BTSerial.read();
              if (char1 == BT_START)
                FSM_state = 1;
            }
            break;  

    case 1: if (BTSerial.available()){
              char1 = BTSerial.read();
              if (char1 != BT_START && char1 != BT_END){
                BTread[FLEX_index] = char1;
//                stats_updpate(FLEX_index, BTread[FLEX_index]);
                FLEX_index++;
              }
              else if (char1 == BT_START) FLEX_index = 0;  
              if (FLEX_index == NS){         // once a byte received for each finger, print on serial monitor
                FLEX_index = 0;
                #if (SERIAL_DEBUG==2)
                  Serial.print(F("Rx: "));
                  Serial.print("\t");
                  for (uint8_t i=0; i<NS; i++){
                    Serial.print(BTread[i]);
                    if(i<(NS-1)) Serial.print("\t");
                    else Serial.println("");
                  }
                #endif
              FSM_state = 0;
              }
            }
            break;
  
    default: FSM_state = 0;
             break;          
  } // end switch
}   // end function

//=========================================================== LOOP functions: Write Servo
void writeServo(){
  #if (SERIAL_DEBUG==3)
    Serial.print(F("Writing to Servos: "));
    Serial.print("\t");
  #endif
  for(uint8_t i=0;i<NS;i++){
    Mpos[i]=map(BTread[i], BT_MIN ,BT_MAX , SERVO_MIN, SERVO_MAX);
    if(Mpos[i]!=Current_pos[i]){
      Error_pos[i] = Mpos[i]-Current_pos[i];
//      Error_sum[i] += Error_pos[i];
//      Current_pos[i] = Current_pos[i] + round(Error_pos[i]*PROPORTIONAL_CONSTANT) + round(Error_sum[i]*INTEGRAL_CONSTANT);
      Current_pos[i] += round(Error_pos[i]*PROPORTIONAL_CONSTANT);
    }
    else{
      Current_pos[i] = Mpos[i];
    }
    M[i].write(Current_pos[i]);
    #if (SERIAL_DEBUG==3)
      Serial.print(Current_pos[i]);
      if (i<(NS-1)) Serial.print("\t");
      else Serial.println("");
    #endif
  }
}

void writeServoWave(){
  Serial.print(F("k = ")); Serial.print(k);
  for(uint8_t i=0;i<NS;i++){
    M[i].write(k);
    Serial.print('.');
  }
  Serial.println("");
  if(countup == 1) k++; else k--;
  if (k==HANDWAVE_COUNT) countup = 0;
  if (k==0)  countup = 1;
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
