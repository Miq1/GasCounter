// Copyright 2020 Michael Harwerth
#include <Arduino.h>
#include <ALtSoftSerial.h>
#include <EEPROM.h>

// 3 bit switches connected to pins 4, 5 and 6
#define IDBIT1 4
#define IDBIT2 5
#define IDBIT4 6

// Debug switch connected to pin 7
#define DEBUG_ON 7

// Built-in LED
#define LED 13

// HOTWIRE pin connected to Reed switch
#define HOTWIRE 2

// Tick counter
volatile uint32_t ticks;
// Flag to signal pulse
volatile bool hitOne;
// Down time to prevent bouncing
volatile uint32_t latency;
#define LATENCY_TIME 1000
// Runtime since last boot in minutes
uint16_t runTime;

// History definitions
const uint8_t historyInterval(3);   // minutes per slot
uint16_t historyOffset;
uint16_t dayval = 0;                // days since init

// Magic value to recognize initialized EEPROM
const uint16_t MAGIC_VAL(0x4714);

// EEPROM memory layout
// 0..3: uint32_t pulse counter
//       Modbus register 1, 2
const uint8_t O_PULSES(0);

// 4..7: float offset value
//       Modbus register 3, 4
const uint8_t O_OFFSET(4);

// 8..11: float value per pulse
//       Modbus register 5, 6
const uint8_t O_PULSEVAL(8);

// 12..13: uint16_t magic value
const uint8_t O_MAGIC(12);

// 14: byte restart counter
//       Modbus register 494
const uint8_t O_RESTARTS(14);

// 15..16: uint16_t days since init
//       Modbus register 496
const uint8_t O_DAYVAL(15);

// 17..30: uint16_t[7] daily pulses
//       Modbus register 7..13
const uint8_t O_DAYCOUNTS(17);

// 31..511: byte[481] 3-minute pulse counts
//       Modbus register 14..493
const uint8_t O_HISTORY(31);

//       Modbus register 495 is runTime
//       Modbus register 497 is ticks today

// Array to hold byte order of native floats
uint8_t floatOrder[4];

// MODBUS request buffer
// normal size is 8 (ID, FC, address 16bit, words 16bit)
// but we may catch packets for other purposes in future
const uint16_t INPACKETSIZE(32);
uint8_t inPacket[INPACKETSIZE];
uint16_t inPacketLen;

// MODBUS response buffer
// maximum size is 3 (ID,FC,Byte count) + 255 (payload bytes) + 2 (CRC) = 260
const uint16_t OUTPACKETSIZE(260);
uint8_t outPacket[OUTPACKETSIZE];
uint16_t outPacketLen;

// Own MODBUS slave ID
uint8_t ownID;

// State machine states
enum STATES : uint8_t { INIT=0, IDLE, PAUSE, IN_PACKET, PROCESS };
STATES state = INIT;

// RS485 module connected to AltSoftSerial
AltSoftSerial MODBUS;
const uint32_t BAUDRATE(19200);

// EEPROM-based config data
float tickFactor;
float offset;

// Highest allowed Modbus register
const uint16_t MB_HIGH(497);

#ifdef TESTING
const char *statenames[] = { "INIT", "IDLE", "PAUSE", "IN_PACKET", "PROCESS" };
const uint16_t BUFLEN(80);
char buffer[BUFLEN];

// Helper function to generate a printable hexadecimal dump
// head: printed first
// sep:  separator char printed between each byte. Omitted, if ==0
// data: pointer to data to be dumped
// length: number of bytes in data
// buffer: target to be printed into
// max_buffer_length: obvious... ;)
uint16_t hexDump(const char *head, const char sep, uint8_t *data, uint16_t length, char *buffer, uint16_t max_buffer_length)
{
  const char *PRINTABLES = "0123456789ABCDEF";
  uint16_t remaining = max_buffer_length - 1;

  while (remaining && *head) {
    *buffer++ = *head++;
    remaining--;
  }
  while (remaining && length) {
    if (sep)  {
      *buffer++ = sep;
      remaining--;
    }
    if (remaining > 2) {
      *buffer++ = PRINTABLES[(*data >> 4) & 0x0F];
      *buffer++ = PRINTABLES[*data & 0x0F];
      data++;
      remaining -= 2;
      length--;
    }
  }
  *buffer = 0;
  return max_buffer_length - remaining;
}
#endif 

// CRC16 tables and calculation function
const uint8_t crcHiTable[] PROGMEM = {
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
  0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
  0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
  0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
  0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
  0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
  0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
  0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
  0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
  0x40
};

const uint8_t crcLoTable[] PROGMEM = {
  0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
  0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
  0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
  0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
  0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
  0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
  0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
  0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
  0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
  0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
  0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
  0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
  0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
  0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
  0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
  0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
  0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
  0x40
};

inline uint16_t CRC16(uint8_t* msg, size_t len) 
{
  register uint8_t crcHi = 0xFF;
  register uint8_t crcLo = 0xFF;
  register uint8_t index;

  while (len--) {
    index = crcLo ^ *msg++;
    crcLo = crcHi ^ pgm_read_byte(&crcHiTable[index]);
    crcHi = pgm_read_byte(&crcLoTable[index]);
  }
  return (crcHi << 8 | crcLo);
}

// Return MODBUS error packet
void error(uint8_t code, uint8_t err)
{
  outPacketLen = 0;
  outPacket[outPacketLen++] = ownID;
  outPacket[outPacketLen++] = code|0x80;
  outPacket[outPacketLen++] = err;
  uint16_t crc = CRC16(outPacket, outPacketLen);
  outPacket[outPacketLen++] = crc&0xFF;
  outPacket[outPacketLen++] = (crc>>8)&0xFF;
#ifdef TESTING
  if (digitalRead(DEBUG_ON)==LOW)
  {
    hexDump("ERR:", ' ', outPacket, outPacketLen, buffer, BUFLEN);
    Serial.println(buffer);
    Serial.flush();
  }
#endif

  MODBUS.write(outPacket, outPacketLen);
  MODBUS.flush();
}

// Interrupt function to count ticks
void cntTick() {
  if (millis() - latency > LATENCY_TIME) {
    cli();
    ticks++;
    hitOne = true;
    sei();
    latency = millis();
  }
}

// determineFloatOrder: calculate the sequence of bytes in a float value
void determineFloatOrder() {
  constexpr uint8_t floatSize = sizeof(float);

  uint32_t i = 77230;                             // int value to go into a float without rounding error
  float f = i;                                    // assign it
  uint8_t *b = (uint8_t *)&f;                     // Pointer to bytes of f
  uint8_t expect[floatSize] = { 0x47, 0x96, 0xd7, 0x00 }; // IEEE754 representation 
  uint8_t matches = 0;                            // number of bytes successfully matched
    
  // Loop over the bytes of the expected sequence
  for (uint8_t inx = 0; inx < floatSize; ++inx) {
    // Loop over the real bytes of f
    for (uint8_t trg = 0; trg < floatSize; ++trg) {
      if (expect[inx] == b[trg]) {
        floatOrder[inx] = trg;
        matches++;
        break;
      }
    }
  }
}

// Helper functions to set and get float, uint16_t and uint32_t values in Modbus messages
void setFloat(uint8_t *dest, float v) {
  union {
    float f;
    uint8_t fvb[4];
  } u;
  u.f = v;
  for (uint8_t i = 0; i < 4; ++i) {
    dest[floatOrder[i]] = u.fvb[i];
  }
}

float getFloat(uint8_t *src) {
  union {
    float f;
    uint8_t fvb[4];
  } u;
  for (uint8_t i = 0; i < 4; ++i) {
    u.fvb[i] = src[floatOrder[i]];
  }
  return u.f;
}

inline void set16(uint8_t *dest, uint16_t v) {
  dest[0] = (v >>8) & 0xFF;
  dest[1] = v & 0xFF;
}

inline uint16_t get16(uint8_t *src) {
  uint16_t v = (src[0] << 8) | src[1];
  return v;
}

inline void set32(uint8_t *dest, uint32_t v) {
  set16(dest, (v >> 16) & 0xFFFF);
  set16(dest + 2, v & 0xFFFF);
}

inline uint32_t get32(uint8_t *src) {
  uint32_t v = ((uint32_t)get16(src) << 16) | get16(src + 2);
  return v;
}

// initEEPROM: re-initialize all counters in EEPROM, due to a changed offset
void initEEPROM(float offsetValue) {
  // Set new offset value
  EEPROM.put(O_OFFSET, offsetValue);
  // clear pulses
  ticks = 0;
  EEPROM.put(O_PULSES, (uint32_t)0);
  // clear daily pulse counts
  EEPROM.put(O_DAYVAL, (uint16_t)0);
  dayval = 0;
  for (uint8_t i = 0; i < 7; ++i) {
    EEPROM.put(O_DAYCOUNTS + i * 2, (uint16_t)0);
  }
  // clear restart counter
  EEPROM.put(O_RESTARTS, (uint8_t)0);
  // clear history
  for (uint16_t i = 0; i < 481; ++i) {
    EEPROM.update(O_HISTORY + i, 0);
  }
  // Set new history offset marker
  historyOffset = 0;
  EEPROM.update(O_HISTORY + historyOffset, 0xFF);
}

void setup() 
{
  // Init LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  // Init HOTWIRE
  pinMode(HOTWIRE, INPUT_PULLUP);

  // Get float order
  determineFloatOrder();

  // Start MODBUS
  MODBUS.begin(BAUDRATE);

  // Set own slave ID
  // if pins are jumpered to GND, bits will be added to slave ID 1
  pinMode(IDBIT1, INPUT_PULLUP);
  pinMode(IDBIT2, INPUT_PULLUP);
  pinMode(IDBIT4, INPUT_PULLUP);

  ownID = 1;
  if (digitalRead(IDBIT1)==LOW) ownID++;
  if (digitalRead(IDBIT2)==LOW) ownID += 2;
  if (digitalRead(IDBIT4)==LOW) ownID += 4;

  Serial.begin(115200);
  Serial.println("");
  Serial.print("ownID=");
  Serial.println((int)ownID);
  Serial.flush();

  // Set up Debug line
  pinMode(DEBUG_ON, INPUT_PULLUP);

  // Read out EEPROM
  uint16_t magic;
  // Get magic word
  EEPROM.get(O_MAGIC, magic);
  // Is it correct?
  if (magic == MAGIC_VAL) {
    // Yes. Get data
    uint32_t pval;
    EEPROM.get(O_PULSES, pval);
    ticks = pval;
    EEPROM.get(O_PULSEVAL, tickFactor);
    EEPROM.get(O_OFFSET, offset);
    EEPROM.get(O_DAYVAL, dayval);
    // Update restart counter
    uint8_t restarts = EEPROM.read(O_RESTARTS);
    if (restarts < 255) {
      restarts++;
    }
    EEPROM.update(O_RESTARTS, restarts);
    // get history offset marker and read not yet saved pulses to properly init ticks
    for (historyOffset = 0; historyOffset < 481; historyOffset++) {
      uint8_t tp = EEPROM.read(O_HISTORY + historyOffset);
      if (tp == 0xFF) break;
      ticks += tp;
    }
    // Did we find it?
    if (historyOffset >= 481) {
      // No... use 0 instead
      Serial.println("History>=481!");
      historyOffset = 0;
      EEPROM.update(O_HISTORY + historyOffset, 0xFF);
      // restore to last saved ticks - we do not know it any better here!
      EEPROM.get(O_PULSES, pval);
      ticks = pval;
    }
  } else {
    // No. Initialize EEPROM data
    EEPROM.put(O_MAGIC, MAGIC_VAL);
    tickFactor = 1.0;
    offset = 0.0;
    EEPROM.put(O_PULSEVAL, tickFactor);
    initEEPROM(offset);
  }

  attachInterrupt(digitalPinToInterrupt(HOTWIRE), cntTick, FALLING);

  runTime = 0;

#if TESTING
  if (digitalRead(DEBUG_ON) == LOW)
  {
    for (uint16_t i = 0; i < 512; ++i) {
      Serial.print(EEPROM.read(i), HEX);
      Serial.print(" ");
      if (i % 16 == 15) Serial.print("\n");
    }
    Serial.print("T=");
    Serial.println(ticks);
  }
#endif
}

void loop() 
{
  static uint8_t readByte = 0;
  static uint8_t histCnt = 0;
  static uint32_t lastTicks = ticks;
  static uint32_t pulseTime = 0;
  // PAUSETIME gives the required silence time on the bus to detect a new packet.
  // With 19200/s, a byte takes about 0.52 ms, so we are waiting for a pause of
  // approx. 1822us here!
  // (The MODBUS standard is requiring >3.5)
  const uint32_t PAUSETIME = 35000000UL / BAUDRATE;
  static uint32_t starttime = micros();
  static uint32_t runCnt = millis();

#ifdef TESTING
  // Print out state changes in debug mode
  static STATES oldState = INIT;

  if (digitalRead(DEBUG_ON) == LOW)
  {
    if (oldState != state)
    {
      Serial.print(statenames[state]);
      Serial.print(' ');
      Serial.flush();
      oldState = state;
    }
  }
#endif

  switch (state)
  {
  case INIT: // Init means skip to pause, so next data will be a packet most probably
    // Read all available bytes into the bin
    while (MODBUS.available()) readByte = MODBUS.read();
    state = PAUSE;
    starttime = micros();
    // No break; - Fallthrough intended!
  case PAUSE:  // Discard all bytes, then wait for a PAUSETIME gap
    // PAUSETIME passed without a byte?
    if (micros() - starttime >= PAUSETIME)
    {
      // Yes. Rewind timer to open I²C writing with a delay
      starttime = micros();
      // Go look for a packet
      state = IDLE;
    }
    // No, but is a byte available?
    else if (MODBUS.available())
    {
      // Yes, there is one. Pause was interrupted - skip to next pause
      state = INIT;
    }
    break;
  case IDLE: // Waiting for a packet after the pause
    // Do we have a byte? It will be the first of a packet = Slave ID
    if (MODBUS.available())
    {
      // Yes. Is it our own slave ID?
      readByte = MODBUS.read();
      if (readByte != ownID)
      {
        // No. Skip packet and wait for the next
        state = INIT;
      }
      else
      {
        // This is for us - go collect the packet
        inPacketLen = 0;
        inPacket[inPacketLen++] = readByte;
        // Signal to D0 processor to stop sending data
        state = IN_PACKET;
        starttime = micros();
      }
    }
    break;
  case IN_PACKET:  // We caught a byte of a packet addressed to us
    // If a byte is available...
    if (MODBUS.available())
    {
      while (MODBUS.available())
      {
        // Catch it.
        readByte = MODBUS.read();
        if (inPacketLen<INPACKETSIZE) 
        {
          inPacket[inPacketLen++] = readByte;
        }
        else
        {
          // Overflow... Discard all data
          state = INIT;
          break;
        }
      }
      starttime = micros();
    }

    // Did we encounter a gap time again?
    if (micros() - starttime >= PAUSETIME)
    {
      // Yes. Process data
      state = PROCESS;
    }
    break;
  case PROCESS:  // we seem to have caught a packet.
    {
      digitalWrite(LED, HIGH);
#ifdef TESTING
      if (digitalRead(DEBUG_ON) == LOW)
      {
        hexDump("REQ:", ' ', inPacket, inPacketLen, buffer, BUFLEN);
        Serial.println(buffer);
        Serial.flush();
      }
#endif

      uint8_t fc = inPacket[1];
      // Is the function code right?
      switch (fc) {
      case 0x03:
      case 0x04:
        // Sufficcient data for FC 0x03/0x04?
        if (inPacketLen == 8)
        {
          // Yes, fc is okay, get address to read
          uint16_t addr = get16(inPacket + 2);
          // Is it out of bounds?
          if (addr < 1 || addr > MB_HIGH)
          {
            // Yes, return error
            error(fc, 0x02);
          } else {
            // No, address is okay. get number of words requested
            uint16_t wrds = get16(inPacket + 4);
            uint16_t byts = wrds * 2;
            // Words out of bounds?
            if (!wrds || byts > (OUTPACKETSIZE - 5) || ((addr - 1 + wrds) > MB_HIGH))
            {
              // Yes, return error
              error(fc, 0x02);
            } else {
              // No, words will fit in memory. 
              // Check if packet is intact
              if (CRC16(inPacket, inPacketLen - 2) != (uint16_t)((inPacket[7] << 8) | inPacket[6]))
              {
                // No, seems to be corrupt
                error(fc, 0xE3);
              } else {
                // Packet is intact. 
                // Build response packet and return it
                outPacketLen = 0;
                outPacket[outPacketLen++] = ownID;
                outPacket[outPacketLen++] = fc;
                outPacket[outPacketLen++] = byts;

                // Fill in requested data
                uint16_t pos = 0;
                uint16_t val = 0;
                unsigned char ordered[12];
                // We may need the first three 4 byte values in MSB-first order
                if (addr < 7) {
                  // We do - prepare a char array with these
                  set32(ordered, ticks);
                  setFloat(ordered + 4, offset);
                  setFloat(ordered + 8, tickFactor);
                }
                for (uint8_t nbr = wrds; nbr > 0 ; nbr--) {
                  // Depending on the address we may need to reformat data
                  // 1..6: bytes starting at 0
                  if (addr < 7) {
                    memcpy(outPacket + outPacketLen, ordered + (addr - 1) * 2, 2);
                    outPacketLen += 2;
                  // 7..13: bytes starting at O_DAYCOUNTS, offset by dayval mod 7
                  } else if (addr < 14) {
                    pos = O_DAYCOUNTS + ((dayval + (addr - 7)) % 7) * 2;
                    EEPROM.get(pos, val);
                    set16(outPacket + outPacketLen, val);
                    outPacketLen += 2;
                  // 14..493: history bytes to be converted to unit16_t
                  } else if (addr < 494) {
                    pos = O_HISTORY + (historyOffset + addr - 13) % 481;
                    set16(outPacket + outPacketLen, EEPROM.read(pos));
                    outPacketLen += 2;
                  // 494: Number of restarts
                  } else if (addr == 494) {
                    val = EEPROM.read(O_RESTARTS);
                    set16(outPacket + outPacketLen, val);
                    outPacketLen += 2;
                  // 495: Run time 
                  } else if (addr == 495) {
                    set16(outPacket + outPacketLen, runTime);
                    outPacketLen += 2;
                  // 496: Days since init
                  } else if (addr == 496) {
                    set16(outPacket + outPacketLen, dayval);
                    outPacketLen += 2;
                  // 497: Ticks today
                  } else if (addr == 497) {
                    uint32_t baseval = 0;
                    EEPROM.get(O_PULSES, baseval);
                    set16(outPacket + outPacketLen, (ticks - baseval) & 0xFFFF);
                    outPacketLen += 2;
                  }
                  addr++;
                }
  
                // Calculate CRC for the response packet
                uint16_t crc = CRC16(outPacket, outPacketLen);
                outPacket[outPacketLen++] = crc & 0xFF;
                outPacket[outPacketLen++] = (crc >> 8) & 0xFF;
#ifdef TESTING
                if (digitalRead(DEBUG_ON)==LOW)
                {
                  hexDump("RSP:", ' ', outPacket, outPacketLen, buffer, BUFLEN);
                  Serial.println(buffer);
                  Serial.flush();
                }
#endif
                // Write response to MODBUS requester
                MODBUS.write(outPacket, outPacketLen);
                MODBUS.flushOutput();
              }
            }
          }
        } else {
          // Return packet length mismatch
          error(fc, 0xE5);
        }
        break;
      case 0x10:
        // Sufficient data for FC 0x10?
        if (inPacketLen > 9 && inPacketLen == (uint16_t)(9 + inPacket[6]))
        {
          // Yes, fc is okay, get address to read
          uint16_t addr = get16(inPacket + 2);
          // Get length to be written
          uint8_t count = inPacket[6];
          // Is it out of bounds? only addresses 3 and 5 and data lengths of 4 or 8 are possible
          if ((addr == 3 && (count == 4 || count == 8)) ||
              (addr == 5 && count == 4)) {
            // No, address and length are within limits
            // Check if packet is intact
            if (CRC16(inPacket, inPacketLen - 2) != (uint16_t)((inPacket[inPacketLen - 1] << 8) | inPacket[inPacketLen - 2]))
            {
              // No, seems to be corrupt
              error(fc, 0xE3);
            } else {
              // Packet is intact. Write data, Build response packet and return it
              outPacketLen = 6;
              memcpy(outPacket, inPacket, 6);
              // Calculate CRC for the response packet
              uint16_t crc = CRC16(outPacket, outPacketLen);
              outPacket[outPacketLen++] = crc & 0xFF;
              outPacket[outPacketLen++] = (crc >> 8) & 0xFF;
              // Write response to MODBUS requester
              MODBUS.write(outPacket, outPacketLen);
              MODBUS.flushOutput();
              // Rewrite EEPROM
              // Offset changed?
              if (addr == 3) {
                offset = getFloat(inPacket + 7);
                initEEPROM(offset);
              }
              // Tick factor changed?
              if (addr == 5 || count == 8) {
                tickFactor = getFloat(inPacket + (addr == 3 ? 11 : 7));
                EEPROM.put(O_PULSEVAL, tickFactor);
              }
            }
          } else {
            // Yes, return error
            error(fc, 0x02);
          }
        }
        break;
      default:
        error(fc, 0x01);
        break;
      }
      
      digitalWrite(LED, LOW);
      state = INIT;
    }
    break;
  }

  // Blink LED to signal a pulse
  if (hitOne) {
    digitalWrite(LED, HIGH);
    pulseTime = millis();
    hitOne = false;
  } else {
    if (pulseTime && millis() - pulseTime > 100) {
      digitalWrite(LED, LOW);
      pulseTime = 0;
    }
  }

  // Every minute, increment runTime counter
  if (millis() - runCnt > 60000) {
    runCnt += 60000;
    // Update run time up to 65535 minutes
    if (runTime < 0xFFFF) {
      runTime++;
    }
    // Every 3 minutes, store accumulated ticks into history
    histCnt++;
    if (histCnt >= historyInterval) {
      // Need to save the tick count. 
      uint16_t iTicks = ticks - lastTicks;
      lastTicks = ticks;
      // Put value in current history slot
      // Overflow?
      if (iTicks > 253) {
        // Yes. write overflow flag instead
        EEPROM.update(O_HISTORY + historyOffset, 0xFE);
      } else {
        // No, it fits
        EEPROM.update(O_HISTORY + historyOffset, iTicks & 0xFF);
      }
      // Advance slot, wrap around at HISTORY_SLOTS
      historyOffset++;
      // Are we at the last slot?
      if (historyOffset > 480) {
        // Yes. collect daily data and save it
        uint32_t sum = 0;
        // Loop over all history values
        for (uint16_t i = 0; i < 481; i++) {
          // Read value. Add it only if not 0xFF (update marker)
          uint8_t hv = EEPROM.read(i + O_HISTORY);
          if (hv != 0xFF) {
            sum += hv;
          }
        }
        uint16_t val = sum;
        // 16 bit overflowing?
        if (sum & 0xFFFF0000) {
          // YES! use default
          val = 0xFFFF;
        }
        EEPROM.put(O_DAYCOUNTS + (dayval % 7) * 2, val);
        // Advance a day
        if (dayval < 0xFFFF) {
          dayval++;
          EEPROM.put(O_DAYVAL, dayval);
          // save collected pulses
          uint32_t pval = ticks;
          EEPROM.put(O_PULSES, pval);
        } else {
          // Days counter is at overflow.
          // We need to re-init most saved data
          // Offset can be advanced by the counted ticks
          initEEPROM(offset + ticks * tickFactor);
        }
        historyOffset = 0;
      }
      // Update marker byte
      EEPROM.update(O_HISTORY + historyOffset, 0xFF);
      histCnt = 0;
    }
  }
}
