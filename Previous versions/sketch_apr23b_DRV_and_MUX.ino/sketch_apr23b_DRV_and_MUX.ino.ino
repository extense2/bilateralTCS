#include <Wire.h>
#include <Adafruit_DRV2605.h>
#define NMBR_DRIVERS 5
#define TCAADDR 0x70

Adafruit_DRV2605 drv;

void setup() {
  Serial.begin(9600);
  Serial.println("DRV test");
  setupDRV2605();
}

uint8_t effect = 1;

void loop() {
 for(uint8_t i=0; i<NMBR_DRIVERS; i++){
  writeERM(i);
  }
}

void writeERM(int i){
  Serial.print("DRV "); 
  Serial.println(i+1);
  tcaselect(i+3);
  delay(10);
  drv.go();
  delay(500);
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
void setupDRV2605(){
  for(uint8_t i=0;i<NMBR_DRIVERS;i++){
    tcaselect(i+3);
    if(!drv.begin()){
      Serial.print(F("Failed to initialize driver "));  Serial.println(i);
      Serial.println("..."); 
      Serial.print(F("Please restart program."));
      while(1);
    }
  }
  Serial.println(F("Drivers initialized."));
  for(uint8_t m=0;m<NMBR_DRIVERS;m++){
    tcaselect(m+3);
    drv.selectLibrary(1);     // Select Libraries and effects
    drv.setWaveform(0, 48);   // first digit: index, from 0 to 7. Second digit: waveform number, choose 0 to denote end. 
    drv.setWaveform(1, 0);    // end waveform
  }
  Serial.println(F("Libraries and Effects selected."));
}
