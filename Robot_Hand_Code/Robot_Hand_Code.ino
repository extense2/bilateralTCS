// Arrays were used to simply the code and ensure that each finger had a unique dataset and identifier
  #include <Wire.h> //Libraries required to run the Haptic code
  #include <Servo.h> //Library required to run the Servos

// The sSoftSerial.h library is required to run the Bluetooth communications. Library edited to work with Timer2. 
// The original SoftwareSerial library clashes with the Servo Library
// i.e., both libraries used the same timer and hence the program did not work.
// More info: https://forum.arduino.cc/index.php?topic=358183.30
// and        https://forum.arduino.cc/index.php?topic=374528.0
#include <sSoftSerial.h>

//Initialise the sSoftSerial Library with the RX/TX pins 
sSoftSerial BTserial(8,9);

Servo M[5]; // Create 5 servo objects, one per finger
int i=0;
int Mpos[5]={0}; //Variable to store servo position from flex sensor
int a[5]={0}; //Variable to store lower limit of jitter control code
int b[5]={0}; //Variable to store upper limit of jitter control code
int X=0; //Variable to read Bluetooth and store data temporarily

int Pin[5]={A0, A1, A2, A3, A4}; //Store analogue Pin #s to streamline code
int FSR[5]={0}; //Variable to store the FSR reading for each finger

void setup() {
  Serial.begin(9600);
  BTserial.begin(9600); //Initialise Bluetooth at a Baud rate of 9600
  for(i=0;i<5;i++) //Attach servos to corresponding digital pins
  M[i].attach(i+2);

  //Code to initialise Digital pins 2-6 as outputs for the servos
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
}


void loop() {
  //Code to write the position of the fingers by reading the data from the HC-05 and writing to the servo
  if(BTserial.read()=='<'){         //Checks for the start marker to align fingers correctly
    for(i=0;i<5;){
      X=BTserial.read();            // Reads the value sent by the Bluetooth module
      if(X>-1){                     // Verifies if any relevant data is received
        if(Mpos[i]>a && Mpos[i]<b){}//Jitter control code
        else{                       //Remaps the servo position from 0-127 to 0-180
          Mpos[i]=map(X,0,127,0,180);
          M[i].write(Mpos[i]); //Writes the finger position to the Servo
          a[i]=Mpos[i]-3;           //Sets the lower limit for the Jitter control
          b[i]=Mpos[i]+3;           //Sets the Upper limit for the Jitter control
        }
        i++;                        // i++ if data received is valid and moves to the next finger
      }
    }
  }
  
  //Code to read the analog value from the FSR and sending it via Bluetooth to the glove
  for(i=0;i<5;i++){
    if(i==0) {
      BTserial.write(222);
    }
//    FSR[i]=map(analogRead(Pin[i]),0,1023, 0, 127); //Maps data to the finger and readies it for transmission
    
    FSR[i]=analogRead(Pin[i]);
    FSR[i]=constrain(FSR[i],0,1023);
    FSR[i]=map(FSR[i],0,1023,0,99);

    BTserial.write(FSR[i]);


//    Serial.print(FSR[i]);
//    if(i<4){Serial.print("\t\t");}
  }
//  Serial.println("");
//  delayMicroseconds(250);
    delay(10);
}


