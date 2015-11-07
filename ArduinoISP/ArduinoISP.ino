// Fork of ArduinoISP by Randall Bohn
// https://github.com/satr/ArduinoISP
//
// Features:
// - Optional (regulated with "#define"): display for indication of mode, error, hearbit
// -- Supports: LCD 5110
// - SoftSerial to tranfer serial data to\from the client (the target board which is programmed); Baud: 19200 (same as for ISP)
// -- When not in programming mode - Serial can be use to transfer data to and from the client
// -- Automatically detects the request to program the client and blocks transfer data between ISP and the client
// - Optional (regulated with "#define"): LED indication; pins are changed from original - to leave SPI pins free
//---------------------------------------------------------------------------------
// ArduinoISP version 04m3
// Copyright (c) 2008-2011 Randall Bohn
// If you require a license, see 
//     http://www.opensource.org/licenses/bsd-license.php
//
// This sketch turns the Arduino into a AVRISP
// using the following arduino pins:
//
// pin name:    not-mega:         mega(1280 and 2560)
// slave reset: 10:               53 
// MOSI:        11:               51 
// MISO:        12:               50 
// SCK:         13:               52 
//
// Put an LED (with resistor) on the following pins:
// 9: Heartbeat   - shows the programmer is running
// 8: Error       - Lights up if something goes wrong (use red if that makes sense)
// 7: Programming - In communication with the slave
//
// 23 July 2011 Randall Bohn
// -Address Arduino issue 509 :: Portability of ArduinoISP
// http://code.google.com/p/arduino/issues/detail?id=509
//
// October 2010 by Randall Bohn
// - Write to EEPROM > 256 bytes
// - Better use of LEDs:
// -- Flash LED_PMODE on each flash commit
// -- Flash LED_PMODE while writing EEPROM (both give visual feedback of writing progress)
// - Light LED_ERR whenever we hit a STK_NOSYNC. Turn it off when back in sync.
// - Use pins_arduino.h (should also work on Arduino Mega)
//
// October 2009 by David A. Mellis
// - Added support for the read signature command
// 
// February 2009 by Randall Bohn
// - Added support for writing to EEPROM (what took so long?)
// Windows users should consider WinAVR's avrdude instead of the
// avrdude included with Arduino software.
//
// January 2008 by Randall Bohn
// - Thanks to Amplificar for helping me with the STK500 protocol
// - The AVRISP/STK500 (mk I) protocol is used in the arduino bootloader
// - The SPI functions herein were developed for the AVR910_ARD programmer 
// - More information at http://code.google.com/p/mega-isp

#include "pins_arduino.h"
#define RESET     SS

//#define USE_LEDs // uncomment if LEDs are used - check conflicts between LED-pins, display and SoftwareSerial pins (see initialization below)
#ifdef USE_LEDs  
#define LED_HB    A2
#define LED_ERR   3
#define LED_PMODE 2
#endif

#define PROG_FLICKER true
#define HWVER 2
#define SWMAJ 1
#define SWMIN 18

// STK Definitions
#define STK_OK      0x10
#define STK_FAILED  0x11
#define STK_UNKNOWN 0x12
#define STK_INSYNC  0x14
#define STK_NOSYNC  0x15
#define CRC_EOP     0x20 //ok it is a space...
#define CRC_REQ_PROG 0x30 //request program mode

void pulse(int pin, int times);

//-----------------------------------------------------------------------------------
//  84x84 LCD 5110 display library
// Copyright (C)2015 Rinky-Dink Electronics, Henning Karlsen. All right reserved
// web: http://www.RinkyDinkElectronics.com/
//
// This program is a demo of how to use print().
//
// This program requires a Nokia 5110 LCD module.
//
// It is assumed that the LCD module is connected to
// the following pins using a levelshifter to get the
// correct voltage to the module.
//      SCK      - Pin 9
//      MOSI(DIN)- Pin 8
//      DC       - Pin 7
//      RST      - Pin 6
//      CS(CE)   - Pin 5
//
// Pins 8, 9, 10, 11, 12 defined in original library tutorial were chenged to avoid conflicts with other SPI devices

#define USE_LCD5110_Display // keep uncomment to use LCD 5110 display
#ifdef USE_LCD5110_Display
#include <LCD5110_Graph.h>
LCD5110 lcdDisplay(9,8,7,6,5);
extern uint8_t SmallFont[];
#endif

//#if defined(USE_LCD5110_Display)
#define USE_Display //use one of displays if defined
//#endif

void setupDisplay(){
#ifdef USE_LCD5110_Display
  lcdDisplay.InitLCD();
  lcdDisplay.setFont(SmallFont);
#endif
}

void clearDisplayRect(int x1, int y1, int x2, int y2){
#ifdef USE_LCD5110_Display
  for(byte y = y1; y < y2; y++)
    lcdDisplay.clrLine(x1, y, x2, y);
  lcdDisplay.update();
#endif
}

void printOnDisplay(String message, int x, int y){
#ifdef USE_LCD5110_Display
  lcdDisplay.print(message, x, y);//x: coordinate of the upper, left corner of the first character;
                                  //y: coordinate of the upper, left corner of the first character 0, 8, 16, 24, 32 and 40
  lcdDisplay.update();
#endif
}
//--------------------------------------------------------------------------------

//--------------------------------------------------------------------------------
// SoftwareSerial
// created back in the mists of time
// modified 25 May 2012
// by Tom Igoe
// based on Mikal Hart's example
// 
#include <SoftwareSerial.h>
SoftwareSerial swSerial(4, A3);//RX, TX - to translate Serial data to/from the client, Not all pins can work for RX!
//--------------------------------------------------------------------------------

boolean inProgramMode = false;
boolean isError = false;
byte requestedProgModeCount = 0;
char prevChar = 0x00;
long lastActivityTimestamp = 0;
long lastConfirmedProgModeTimestamp = 0;
long heartbitCount = 0;

void registerSerialDataActivity(){
  lastActivityTimestamp = millis();
}

void clearError(){
  clearDisplayRect(0, 10, 83, 20);
  isError = false;
}

void printError(String message){
  printOnDisplay("Err:" + message, 0, 8);
  isError = true;
}

void clearMode(){
  clearDisplayRect(0, 0, 83, 9);
}

void printMode(String message){
  clearMode();
  printOnDisplay("Mod:" + message, 0, 0);
}

void setProgramMode(){
  printMode("Program");
  inProgramMode = true;
  prevChar = 0x00;
}

void clearProgramMode(){
  printMode("Ready");
  inProgramMode = false;
  requestedProgModeCount = 0;
  lastConfirmedProgModeTimestamp = 0;
  prevChar = 0x00;
}

void setup() {
  Serial.begin(19200);
#ifdef USE_LEDs  
  pinMode(LED_PMODE, OUTPUT);
  pulse(LED_PMODE, 2);
  pinMode(LED_ERR, OUTPUT);
  pulse(LED_ERR, 2);
  pinMode(LED_HB, OUTPUT);
  pulse(LED_HB, 2);
#endif

  swSerial.begin(19200);
  setupDisplay();
  printMode("Ready");
  clearError();
}

int error=0;
int pmode=0;
// address for reading and writing, set by 'U' command
int here;
uint8_t buff[256]; // global block storage

#define beget16(addr) (*addr * 256 + *(addr+1) )
typedef struct param {
  uint8_t devicecode;
  uint8_t revision;
  uint8_t progtype;
  uint8_t parmode;
  uint8_t polling;
  uint8_t selftimed;
  uint8_t lockbytes;
  uint8_t fusebytes;
  int flashpoll;
  int eeprompoll;
  int pagesize;
  int eepromsize;
  int flashsize;
} 
parameter;

parameter param;

// this provides a heartbeat on pin 9, so you can tell the software is running.
#ifdef USE_LEDs
uint8_t hbval=128;
int8_t hbdelta=8;
#endif
void heartbeat() {
  if (inProgramMode || isProgModeRequested()) // do not tick during programming
    return;

#ifdef USE_LEDs //heartbit by LEDs
  if (hbval > 192) hbdelta = -hbdelta;
  if (hbval < 32) hbdelta = -hbdelta;
  hbval += hbdelta;
  analogWrite(LED_HB, hbval);
  delay(20);
#endif

#ifdef USE_Display //heartbit by display
  heartbitCount++;
  switch(heartbitCount){
    case 10000:
      printHeartbeat(" ");//"empty" char
      break;
    case 30000:
      printHeartbeat("*");//"filled" char
      break;
    default:
      if(heartbitCount > 40000)
        heartbitCount = 0;
      break;
  }
#endif
}

void printHeartbeat(String value){
  printOnDisplay(value, 0, 24);
}

char currChar = 0x00;

void updateLEDs(){
#ifdef USE_LEDs  
  // is pmode active?
  if (pmode) digitalWrite(LED_PMODE, HIGH); 
  else digitalWrite(LED_PMODE, LOW);
  // is there an error?
  if (error) digitalWrite(LED_ERR, HIGH); 
  else digitalWrite(LED_ERR, LOW);
#endif
}

void writeSerial(){
  if (inProgramMode || isProgModeRequested() || !swSerial.available())
    return;
  Serial.write(swSerial.read());
  registerSerialDataActivity();
}

boolean isProgModeRequested(){
  return requestedProgModeCount >= 2;
}

void readSerial(){
  if (!Serial.available())
    return;

  currChar = Serial.read();
  registerSerialDataActivity();

  if(inProgramMode || isProgModeRequested()) {//started program mode
    avrisp(currChar);
    return;
  }

  long currTimestamp = millis();
  
  if(CRC_EOP == currChar && CRC_REQ_PROG == prevChar//watch for "init programming" mode requested each 250 msec
     && (currTimestamp - lastConfirmedProgModeTimestamp) > 200){

    requestedProgModeCount++;
    lastConfirmedProgModeTimestamp = currTimestamp;

    if(isProgModeRequested())
      replayInsyncOk();

  } else if(requestedProgModeCount > 0 //watch no other data coming within 250 msec till next request
           && (CRC_REQ_PROG != currChar || (currTimestamp - lastConfirmedProgModeTimestamp) < 200)){

    requestedProgModeCount = 0;//reset request programming if not CRC_REQ_PROG + CRC_EOP sequence

  }
  
  prevChar = currChar;

  if (requestedProgModeCount < 2)//in non-program mode - translate data through serial to client
    swSerial.write(currChar);
}

boolean programModeWatchDogEvent(){
  if(!(inProgramMode || isProgModeRequested()) || (millis() - lastActivityTimestamp) < 1000)//if there are some activity during 1 sec
    return false;

  clearProgramMode();//if there is no activity during 1 sec - fire event
  printError("Timeout");
  return true;
}

void loop(void) {

  updateLEDs();
  heartbeat();

  readSerial();
  programModeWatchDogEvent();
  writeSerial();    

}

uint8_t getch() {
  while(!Serial.available()){
    if(programModeWatchDogEvent())
      return STK_FAILED;
  }
  registerSerialDataActivity();
  return Serial.read();
}

void fill(int n) {
  for (int x = 0; x < n; x++) {
    buff[x] = getch();
    registerSerialDataActivity();
  }
}

#define PTIME 30
void pulse(int pin, int times) {
#ifdef USE_LEDs
  do {
    digitalWrite(pin, HIGH);
    delay(PTIME);
    digitalWrite(pin, LOW);
    delay(PTIME);
  } 
  while (times--);
#endif
}

void prog_lamp(int state) {
#ifdef USE_LEDs
  if (PROG_FLICKER)
    digitalWrite(LED_PMODE, state);
#endif
}

void spi_init() {
  uint8_t x;
  SPCR = 0x53;
  x=SPSR;
  x=SPDR;
}

void spi_wait() {
  do {
    registerSerialDataActivity();
  } 
  while (!(SPSR & (1 << SPIF)));
}

uint8_t spi_send(uint8_t b) {
  uint8_t reply;
  SPDR=b;
  spi_wait();
  reply = SPDR;
  return reply;
}

uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  uint8_t n;
  spi_send(a); 
  n=spi_send(b);
  //if (n != a) error = -1;
  n=spi_send(c);
  return spi_send(d);
}

void  printErrorStkNoSync(){
  printError("STK_NOSYNC");
}

void empty_reply() {
  if (CRC_EOP == getch()) {
    replayInsyncOk();
  } 
  else {
    error++;
    Serial.print((char)STK_NOSYNC);
    printErrorStkNoSync();
  }
  registerSerialDataActivity();
}

void replayInsyncOk(){
  Serial.print((char)STK_INSYNC);
  Serial.print((char)STK_OK);
}

void breply(uint8_t b) {
  if (CRC_EOP == getch()) {
    Serial.print((char)STK_INSYNC);
    Serial.print((char)b);
    Serial.print((char)STK_OK);
  } 
  else {
    error++;
    Serial.print((char)STK_NOSYNC);
    printErrorStkNoSync();
  }
  registerSerialDataActivity();
}

void get_version(uint8_t c) {
  switch(c) {
  case 0x80:
    breply(HWVER);
    break;
  case 0x81:
    breply(SWMAJ);
    break;
  case 0x82:
    breply(SWMIN);
    break;
  case 0x93:
    breply('S'); // serial programmer
    break;
  default:
    breply(0);
  }
  registerSerialDataActivity();
}

void set_parameters() {
  // call this after reading paramter packet into buff[]
  param.devicecode = buff[0];
  param.revision   = buff[1];
  param.progtype   = buff[2];
  param.parmode    = buff[3];
  param.polling    = buff[4];
  param.selftimed  = buff[5];
  param.lockbytes  = buff[6];
  param.fusebytes  = buff[7];
  param.flashpoll  = buff[8]; 
  // ignore buff[9] (= buff[8])
  // following are 16 bits (big endian)
  param.eeprompoll = beget16(&buff[10]);
  param.pagesize   = beget16(&buff[12]);
  param.eepromsize = beget16(&buff[14]);

  // 32 bits flashsize (big endian)
  param.flashsize = buff[16] * 0x01000000
    + buff[17] * 0x00010000
    + buff[18] * 0x00000100
    + buff[19];

}

void start_pmode() {
  spi_init();
  // following delays may not work on all targets...
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, HIGH);
  pinMode(SCK, OUTPUT);
  digitalWrite(SCK, LOW);
  delay(50);
  digitalWrite(RESET, LOW);
  delay(50);
  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);
  spi_transaction(0xAC, 0x53, 0x00, 0x00);
  pmode = 1;
  setProgramMode();
  registerSerialDataActivity();
}

void end_pmode() {
  pinMode(MISO, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
  pinMode(RESET, INPUT);
  pmode = 0;
  clearProgramMode();
  clearError();
  registerSerialDataActivity();
}

void universal() {
  int w;
  uint8_t ch;

  fill(4);
  ch = spi_transaction(buff[0], buff[1], buff[2], buff[3]);
  breply(ch);
  registerSerialDataActivity();
}

void flash(uint8_t hilo, int addr, uint8_t data) {
  spi_transaction(0x40+8*hilo, 
  addr>>8 & 0xFF, 
  addr & 0xFF,
  data);
  registerSerialDataActivity();
}
void commit(int addr) {
  if (PROG_FLICKER) 
    prog_lamp(LOW);
  spi_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0);
  if (PROG_FLICKER) {
    delay(PTIME);
    prog_lamp(HIGH);
  }
  registerSerialDataActivity();
}

//#define _current_page(x) (here & 0xFFFFE0)
int current_page(int addr) {
  if (param.pagesize == 32)  return here & 0xFFFFFFF0;
  if (param.pagesize == 64)  return here & 0xFFFFFFE0;
  if (param.pagesize == 128) return here & 0xFFFFFFC0;
  if (param.pagesize == 256) return here & 0xFFFFFF80;
  return here;
}


void write_flash(int length) {
  fill(length);
  if (CRC_EOP == getch()) {
    Serial.print((char) STK_INSYNC);
    Serial.print((char) write_flash_pages(length));
  } 
  else {
    error++;
    Serial.print((char) STK_NOSYNC);
    printErrorStkNoSync();
  }
  registerSerialDataActivity();
}

uint8_t write_flash_pages(int length) {
  int x = 0;
  int page = current_page(here);
  while (x < length) {
    if (page != current_page(here)) {
      commit(page);
      page = current_page(here);
    }
    flash(LOW, here, buff[x++]);
    flash(HIGH, here, buff[x++]);
    here++;
  }

  commit(page);

  registerSerialDataActivity();
  return STK_OK;
}

#define EECHUNK (32)
uint8_t write_eeprom(int length) {
  // here is a word address, get the byte address
  int start = here * 2;
  int remaining = length;
  if (length > param.eepromsize) {
    error++;
    printError("STK_FAILED");
    return STK_FAILED;
  }
  while (remaining > EECHUNK) {
    write_eeprom_chunk(start, EECHUNK);
    start += EECHUNK;
    remaining -= EECHUNK;
  }
  write_eeprom_chunk(start, remaining);
  registerSerialDataActivity();
  return STK_OK;
}
// write (length) bytes, (start) is a byte address
uint8_t write_eeprom_chunk(int start, int length) {
  // this writes byte-by-byte,
  // page writing may be faster (4 bytes at a time)
  fill(length);
  prog_lamp(LOW);
  for (int x = 0; x < length; x++) {
    int addr = start+x;
    spi_transaction(0xC0, (addr>>8) & 0xFF, addr & 0xFF, buff[x]);
    delay(45);
  }
  prog_lamp(HIGH); 
  registerSerialDataActivity();
  return STK_OK;
}

void program_page() {
  char result = (char) STK_FAILED;
  int length = 256 * getch();
  length += getch();
  char memtype = getch();
  // flash memory @here, (length) bytes
  if (memtype == 'F') {
    write_flash(length);
    registerSerialDataActivity();
    return;
  }
  if (memtype == 'E') {
    result = (char)write_eeprom(length);
    if (CRC_EOP == getch()) {
      Serial.print((char) STK_INSYNC);
      Serial.print(result);
    } 
    else {
      error++;
      Serial.print((char) STK_NOSYNC);
      printErrorStkNoSync();
    }
    registerSerialDataActivity();
    return;
  }
  Serial.print((char)STK_FAILED);
  registerSerialDataActivity();
  return;
}

uint8_t flash_read(uint8_t hilo, int addr) {
  uint8_t value = spi_transaction(0x20 + hilo * 8,
                                  (addr >> 8) & 0xFF,
                                  addr & 0xFF,
                                  0);
  registerSerialDataActivity();
  return value;
}

char flash_read_page(int length) {
  for (int x = 0; x < length; x+=2) {
    uint8_t low = flash_read(LOW, here);
    Serial.print((char) low);
    uint8_t high = flash_read(HIGH, here);
    Serial.print((char) high);
    here++;
  }
  registerSerialDataActivity();
  return STK_OK;
}

char eeprom_read_page(int length) {
  // here again we have a word address
  int start = here * 2;
  for (int x = 0; x < length; x++) {
    int addr = start + x;
    uint8_t ee = spi_transaction(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
    Serial.print((char) ee);
  }
  registerSerialDataActivity();
  return STK_OK;
}

void read_page() {
  char result = (char)STK_FAILED;
  int length = 256 * getch();
  length += getch();
  char memtype = getch();
  if (CRC_EOP != getch()) {
    error++;
    Serial.print((char) STK_NOSYNC);
    printErrorStkNoSync();
    registerSerialDataActivity();
    return;
  }
  Serial.print((char) STK_INSYNC);
  if (memtype == 'F') result = flash_read_page(length);
  if (memtype == 'E') result = eeprom_read_page(length);
  Serial.print(result);
  registerSerialDataActivity();
  return;
}

void read_signature() {
  if (CRC_EOP != getch()) {
    error++;
    Serial.print((char) STK_NOSYNC);
    printErrorStkNoSync();
    registerSerialDataActivity();
    return;
  }
  Serial.print((char) STK_INSYNC);
  uint8_t high = spi_transaction(0x30, 0x00, 0x00, 0x00);
  Serial.print((char) high);
  uint8_t middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
  Serial.print((char) middle);
  uint8_t low = spi_transaction(0x30, 0x00, 0x02, 0x00);
  Serial.print((char) low);
  Serial.print((char) STK_OK);
  registerSerialDataActivity();
}
//////////////////////////////////////////
//////////////////////////////////////////


////////////////////////////////////
////////////////////////////////////
int avrisp(uint8_t ch) { 
  uint8_t data, low, high;
  switch (ch) {
  case '0': // 0x30 signon
    error = 0;
    empty_reply();
    clearError();
    break;
  case '1': //0x31
    if (getch() == CRC_EOP) {
      Serial.print((char) STK_INSYNC);
      Serial.print("AVR ISP");
      Serial.print((char) STK_OK);
    }
    break;
  case 'A': //0x41
    get_version(getch());
    break;
  case 'B': //0x42
    fill(20);
    set_parameters();
    empty_reply();
    break;
  case 'E': //0x45 extended parameters - ignore for now
    fill(5);
    empty_reply();
    break;

  case 'P': //0x50
    start_pmode();
    empty_reply();
    break;
  case 'U': //0x55 set address (word)
    here = getch();
    here += 256 * getch();
    empty_reply();
    break;

  case 0x60: //STK_PROG_FLASH
    low = getch();
    high = getch();
    empty_reply();
    break;
  case 0x61: //STK_PROG_DATA
    data = getch();
    empty_reply();
    break;

  case 0x64: //STK_PROG_PAGE
    program_page();
    break;

  case 0x74: //STK_READ_PAGE 't'
    read_page();    
    break;

  case 'V': //0x56
    universal();
    break;
  case 'Q': //0x51 end programming
    error=0;
    end_pmode();
    empty_reply();
    clearError();
    break;

  case 0x75: //STK_READ_SIGN 'u'
    read_signature();
    break;

    // expecting a command, not CRC_EOP
    // this is how we can get back in sync
  case CRC_EOP: // 0x20
    error++;
    Serial.print((char) STK_NOSYNC);
    printErrorStkNoSync();
    break;

    // anything else we will return STK_UNKNOWN
  default:
    error++;
    if (CRC_EOP == getch()){
      Serial.print((char)STK_UNKNOWN);
      printError("STK_UNKNOWN");
    }else{
      Serial.print((char)STK_NOSYNC);
      printErrorStkNoSync();
    }
  }
  registerSerialDataActivity();
}

