
#ifndef sSoftSerial_h
#define sSoftSerial_h
#include "Arduino.h"
#include <inttypes.h>

const byte RX_BUFFER_SIZE = 128;

class sSoftSerial : public Print
{
public:
  sSoftSerial(uint8_t receivePin, uint8_t transmitPin) {
    _rxPin = receivePin; _txPin = transmitPin;}

  void begin(uint16_t baudRate=9600);	// initialize, set baudrate, listen
  void listen();			// enable RX interrupts
  void ignore();			// disable RX interrupts
  void setBaudRate(uint16_t baudRate);	// set baud rate (9600 [default], 19200, 38400)
  uint8_t available();			// returns number of characters in buffer
  char read();				// get one character from buffer
  size_t write(uint8_t txChar);		// transmit a character
private:
  uint8_t _rxPin, _txPin;		// RX and TX digital pin numbers (0-19)
};
#endif
