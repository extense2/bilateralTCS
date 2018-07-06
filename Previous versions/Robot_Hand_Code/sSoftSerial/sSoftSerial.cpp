//
// sSoftSerial
// Dec 2015
//
// A timer 2, half-duplex based software serial implementation.
// A single (default 64 byte) buffer is used for RX.
// For best performance the buffer size should be a power of 2.
// TX is unbuffered and blocking, but runs with interrupts enabled.

// Only 9600 baud is currently supported.
// Only a single instance of the class is currently supported.
//
// Derived from a modified version of Robin's sss software serial code.

#include <Arduino.h>
#include <sSoftSerial.h>

#define SSS_9600_TIMER_INCR  52    // 9600 baud assuming default configuration of timer 0

byte rxTail = 0;
volatile byte rxHead = 0;
volatile byte rxBuffer[RX_BUFFER_SIZE]; // use same buffer for send and receive

volatile byte sssBitCount = 0; // counts bits as they are received
volatile byte sssRecvByte = 0; // holder for received byte

volatile byte sssBufferPos = 0; // keeps track of byte to send within the buffer
volatile byte sssNextBit;
volatile byte sssBitPos = 0; // position of bit within byte being sent

uint8_t rxBitMask, txBitMask;	// port bit masks
volatile uint8_t *rxPort, *txPort;	// port registers
volatile uint8_t rxPin, txPin;		// RX and TX digital pin numbers (0-19)

// ------------------------------------------------------------------------------
// begin() -- baudRate parameter is currently ignored (only 9600 is supported)
// ------------------------------------------------------------------------------
void sSoftSerial::begin(uint16_t baudRate) {

  rxPin = _rxPin;
  txPin = _txPin;
  
  pinMode(rxPin, INPUT_PULLUP);
  rxPort = portInputRegister(digitalPinToPort(rxPin));
  rxBitMask = digitalPinToBitMask(rxPin);

  txPort = portOutputRegister(digitalPinToPort(txPin));
  txBitMask = digitalPinToBitMask(txPin);
  *txPort |= txBitMask;		// high = idle
  pinMode(txPin, OUTPUT);	// set bit first then do pinmode to prevent low output
  delay(1);             	// need some idle time
  
  TCCR2A = B00000000; // set normal mode
  TCCR2B = B00000011;  // set the prescaler to 32 - this gives counts at 2usec intervals

  listen();
}

// ------------------------------------------------------------------------------
// listen for RX input
// ------------------------------------------------------------------------------
void sSoftSerial::listen()
{
  rxHead = rxTail = 0;		// no characters in buffer
  
  uint8_t prevSREG = SREG;
  cli();
  *digitalPinToPCMSK(rxPin) |= _BV(digitalPinToPCMSKbit(rxPin));
  *digitalPinToPCICR(rxPin) |= _BV(digitalPinToPCICRbit(rxPin));
  SREG = prevSREG;
}

// ------------------------------------------------------------------------------
// ignore RX interrupts
// ------------------------------------------------------------------------------
void sSoftSerial::ignore()
{
  *digitalPinToPCMSK(rxPin) &= ~_BV(digitalPinToPCMSKbit(rxPin));   // disable pin change
  TIMSK2 &= B11111101;     // Disable compareMatchA
}

// ------------------------------------------------------------------------------
// Return number of characters available.
// ------------------------------------------------------------------------------
uint8_t sSoftSerial::available() {
  return((rxHead - rxTail + RX_BUFFER_SIZE) % RX_BUFFER_SIZE);
}

// ------------------------------------------------------------------------------
// Return received character.
// ------------------------------------------------------------------------------
char sSoftSerial::read()
{
  uint8_t prevSREG = SREG;
  if (rxHead == rxTail) return 0;
  char c = rxBuffer[rxTail];
  rxTail = (rxTail + 1) % RX_BUFFER_SIZE;
  return c;
}

// ------------------------------------------------------------------------------
// Pin change interrupt ISR for start bit detection
// ------------------------------------------------------------------------------
void rxISR()
{
  if (*rxPort & rxBitMask) return;  // if RX is high it isn't a start bit
  
  *digitalPinToPCMSK(rxPin) &= ~_BV(digitalPinToPCMSKbit(rxPin));   // disable pin change

  // start the bitInterval timer
  OCR2A = TCNT2 + SSS_9600_TIMER_INCR;
  TIMSK2 &= B11111101;  // Disable compareMatchA                                 
  TIFR2  |= B00000010;  // Clear  Compare Match A Flag
  TIMSK2 |= B00000010;  // Enable Compare Match A Interrupt
    
  // set counters and buffer index
  sssBitCount = 0;
  sssRecvByte = 0;
}

// ------------------------------------------------------------------------------
// Timer 2 compare A ISR for periodic RX data bit reads
// ------------------------------------------------------------------------------
ISR(TIMER2_COMPA_vect) {

  uint8_t d;

  d = *rxPort & rxBitMask;                // read RX data bit
  if (d != 0) d = 0x01;                   // right justify bit

  OCR2A += SSS_9600_TIMER_INCR;           // update the counter for the next bit interval
 
  if (sssBitCount == 8) {                 // stop bit?
    TIMSK2 &= B11111101;                 // Disable Compare Match A
    *digitalPinToPCMSK(rxPin) |= _BV(digitalPinToPCMSKbit(rxPin));  // enable pin change
    *digitalPinToPCICR(rxPin) |= _BV(digitalPinToPCICRbit(rxPin));
  }

  d = d << sssBitCount;                   // moves it to the correct place
  sssRecvByte += d;

  if (sssBitCount == 7) {                 // last data bit
     byte sssTestHead = (rxHead + 1) % RX_BUFFER_SIZE;
     if (sssTestHead != rxTail) {
       rxHead = sssTestHead;
       rxBuffer[rxHead] = sssRecvByte;
     }
  }
  sssBitCount ++;
}

// ------------------------------------------------------------------------------
// write a character
//
// Each character is written without using a buffer, so this code blocks while the
// character is being transmitted. It uses timer 2 as a metronome but without
// disabling interrupts the whole time, allowing other ISRs to run.
// ------------------------------------------------------------------------------
size_t sSoftSerial::write(uint8_t txChar)
{
  uint8_t txBit, b, t0, width;

  txBit = 0;                    // first bit is start bit
  b = 0;			// start bit is low
  t0 = TCNT2;			// start time

  while (txBit++ < 9) {		// repeat for start bit + 8 data bits
    if (b)			// if bit is set
      *txPort |= txBitMask;     //   set TX line high
    else
      *txPort &= ~txBitMask;    //   else set TX line low
    width = SSS_9600_TIMER_INCR;
    while (uint8_t(TCNT2 - t0) < width) {}  // delay 1 bit width
    t0 += width;			// advance start time
    b = txChar & 0x01;                  // get next bit in the character to send
    txChar = txChar >> 1;               // shift character to expose the following bit
  }

  *txPort |= txBitMask;		        // stop bit is high
  while (uint8_t(TCNT2 - t0) < width) {}	// delay (at least) 1 bit width
  return 1;				// 1 character sent
}



// ------------------------------------------------------------
// Must define all of the pin change vectors even though only one is used.

#if defined(PCINT0_vect)
  ISR(PCINT0_vect) {rxISR();}
#endif
#if defined(PCINT1_vect)
  ISR(PCINT1_vect) {rxISR();}
#endif
#if defined(PCINT2_vect)
  ISR(PCINT2_vect) {rxISR();}
#endif
#if defined(PCINT3_vect)
  ISR(PCINT3_vect) {rxISR();}
#endif
