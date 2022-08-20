// ===================================================================================
// Project:   Battery Capacity Tester based on ATtiny412
// Version:   v1.0
// Year:      2022
// Author:    Stefan Wagner
// Github:    https://github.com/wagiminator
// EasyEDA:   https://easyeda.com/wagiminator
// License:   http://creativecommons.org/licenses/by-sa/3.0/
// ===================================================================================
//
// Description:
// ------------
// The ATtiny412 controlled battery capacity tester measures the capacity of 
// single-cell Li-Ion, LiPo and LiFePO4 batteries using the built-in constant 
// current load. Discharge termination (cutoff) voltage and discharge current
// can be selected by the user. During the discharging process, all relevant 
// data is displayed on an OLED.
//
// References:
// -----------
// SSD1306 OLED datasheet:
// https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
//
// The OLED font was adapted from Neven Boyanov and Stephen Denne:
// https://github.com/datacute/Tiny4kOLED
//
// Wiring:
// -------
//                      +-\/-+
//                Vdd  1|°   |8  GND
//        DAC --- PA6  2|    |7  PA3 --- KEYS
// VOLT SENSE --- PA7  3|    |6  PA0 --- UPDI
//   OLED SDA --- PA1  4|    |5  PA2 --- OLED SCL
//                      +----+
//
// Compilation Settings:
// ---------------------
// Core:    megaTinyCore (https://github.com/SpenceKonde/megaTinyCore)
// Board:   ATtiny412/402/212/202
// Chip:    ATtiny412
// Clock:   5 MHz internal
//
// Leave the rest on default settings.Don't forget to "Burn bootloader". 
// Compile and upload the code.
//
// No Arduino core functions or libraries are used. To compile and upload without
// Arduino IDE download AVR 8-bit toolchain at:
// https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers
// and extract to tools/avr-gcc. Use the makefile to compile and upload.
//
// Fuse Settings: 0:0x00 1:0x00 2:0x02 4:0x00 5:0xC5 6:0x04 7:0x00 8:0x00
//
// Operating Instruction:
// ----------------------
// 1. Connect the device to a 5V power supply via the USB-C port.
// 2. Use the buttons to select the discharge termination voltage and the maximum 
//    discharge current.
// 3. Connect the previously fully charged battery to be tested to the device.
// 4. Press the START button.
// 5. Wait for the discharging process to finish. The capacity can now be read.
// 6. Remove the battery afterwards and press any key.


// ===================================================================================
// Libraries, Definitions and Macros
// ===================================================================================

// Libraries
#include <avr/io.h>                 // for GPIO
#include <avr/interrupt.h>          // for interrupts
#include <util/delay.h>             // for delays

// Pin assignments
#define PIN_SDA       PA1           // I2C Serial Data,  connect to OLED
#define PIN_SCL       PA2           // I2C Serial Clock, connect to OLED
#define PIN_KEYS      PA3           // buttons
#define PIN_DAC       PA6           // DAC, controls electronic load
#define PIN_VOLT      PA7           // voltage sense pin

// Firmware parameters
#define VOLT_MIN      2500          // minimum selectable termination voltage
#define VOLT_MAX      3200          // maximum selectable termination voltage
#define VOLT_STEP     100           // voltage selection steps
#define VOLT_START    3000          // voltage start value
#define CURR_MIN      200           // minimum selectable load current
#define CURR_MAX      2000          // maximum selectable load current
#define CURR_STEP     200           // current selection steps
#define CURR_START    200           // current start value
#define CYCLE_END     1             // 0-stop immediately, 1-decrease load

// Pin manipulation macros
enum {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7};    // enumerate pin designators
#define pinInput(x)   VPORTA.DIR &= ~(1<<(x))     // set pin to INPUT
#define pinOutput(x)  VPORTA.DIR |=  (1<<(x))     // set pin to OUTPUT
#define pinLow(x)     VPORTA.OUT &= ~(1<<(x))     // set pin to LOW
#define pinHigh(x)    VPORTA.OUT |=  (1<<(x))     // set pin to HIGH
#define pinToggle(x)  VPORTA.IN  |=  (1<<(x))     // TOGGLE pin
#define pinRead(x)    (VPORTA.IN &   (1<<(x)))    // READ pin
#define pinPullup(x)  (&PORTA.PIN0CTRL)[x] |= PORT_PULLUPEN_bm  // enable pullup
#define pinDisable(x) (&PORTA.PIN0CTRL)[x] |= PORT_ISC_INPUT_DISABLE_gc

// ===================================================================================
// I2C Master Implementation (Write only)
// ===================================================================================

#define I2C_FREQ  400000                          // I2C clock frequency in Hz
#define I2C_BAUD  (F_CPU / I2C_FREQ - 10) / 2;    // simplified BAUD calculation

// I2C init function
void I2C_init(void) {
  TWI0.MBAUD   = I2C_BAUD;                        // set TWI master BAUD rate
  TWI0.MCTRLA  = TWI_ENABLE_bm;                   // enable TWI master
  TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;            // set bus state to idle
}

// I2C start transmission
void I2C_start(uint8_t addr) {
  TWI0.MADDR = addr;                              // start sending address
}

// I2C stop transmission
void I2C_stop(void) {
  while(~TWI0.MSTATUS & TWI_WIF_bm);              // wait for last transfer to complete
  TWI0.MCTRLB = TWI_MCMD_STOP_gc;                 // send stop condition
}

// I2C transmit one data byte to the slave, ignore ACK bit
void I2C_write(uint8_t data) {
  while(~TWI0.MSTATUS & TWI_WIF_bm);              // wait for last transfer to complete
  TWI0.MDATA = data;                              // start sending data byte 
}

// ===================================================================================
// OLED Implementation
// ===================================================================================

// OLED definitions
#define OLED_ADDR       0x78                      // OLED write address
#define OLED_CMD_MODE   0x00                      // set command mode
#define OLED_DAT_MODE   0x40                      // set data modearray

// OLED init settings
const uint8_t OLED_INIT_CMD[] = {
  0xC8, 0xA1,                                     // flip screen
  0xA8, 0x1F,                                     // set multiplex ratio
  0xDA, 0x02,                                     // set com pins hardware configuration
  0x8D, 0x14,                                     // set DC-DC enable
  0xAF                                            // display on
};

// Standard ASCII 5x8 font (adapted from Neven Boyanov and Stephen Denne)
const uint8_t OLED_FONT[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2F, 0x00, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00,
  0x14, 0x7F, 0x14, 0x7F, 0x14, 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x23, 0x13, 0x08, 0x64, 0x62,
  0x36, 0x49, 0x55, 0x22, 0x50, 0x00, 0x05, 0x03, 0x00, 0x00, 0x00, 0x1C, 0x22, 0x41, 0x00,
  0x00, 0x41, 0x22, 0x1C, 0x00, 0x14, 0x08, 0x3E, 0x08, 0x14, 0x08, 0x08, 0x3E, 0x08, 0x08,
  0x00, 0x00, 0xA0, 0x60, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x60, 0x60, 0x00, 0x00,
  0x20, 0x10, 0x08, 0x04, 0x02, 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x42, 0x7F, 0x40, 0x00,
  0x42, 0x61, 0x51, 0x49, 0x46, 0x21, 0x41, 0x45, 0x4B, 0x31, 0x18, 0x14, 0x12, 0x7F, 0x10,
  0x27, 0x45, 0x45, 0x45, 0x39, 0x3C, 0x4A, 0x49, 0x49, 0x30, 0x01, 0x71, 0x09, 0x05, 0x03,
  0x36, 0x49, 0x49, 0x49, 0x36, 0x06, 0x49, 0x49, 0x29, 0x1E, 0x00, 0x36, 0x36, 0x00, 0x00,
  0x00, 0x56, 0x36, 0x00, 0x00, 0x08, 0x14, 0x22, 0x41, 0x00, 0x14, 0x14, 0x14, 0x14, 0x14,
  0x00, 0x41, 0x22, 0x14, 0x08, 0x02, 0x01, 0x51, 0x09, 0x06, 0x32, 0x49, 0x59, 0x51, 0x3E,
  0x7C, 0x12, 0x11, 0x12, 0x7C, 0x7F, 0x49, 0x49, 0x49, 0x36, 0x3E, 0x41, 0x41, 0x41, 0x22,
  0x7F, 0x41, 0x41, 0x22, 0x1C, 0x7F, 0x49, 0x49, 0x49, 0x41, 0x7F, 0x09, 0x09, 0x09, 0x01,
  0x3E, 0x41, 0x49, 0x49, 0x7A, 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x41, 0x7F, 0x41, 0x00,
  0x20, 0x40, 0x41, 0x3F, 0x01, 0x7F, 0x08, 0x14, 0x22, 0x41, 0x7F, 0x40, 0x40, 0x40, 0x40,
  0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x4E, 0x71, 0x01, 0x71, 0x4E,
  0x7F, 0x09, 0x09, 0x09, 0x06, 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x7F, 0x09, 0x19, 0x29, 0x46,
  0x46, 0x49, 0x49, 0x49, 0x31, 0x01, 0x01, 0x7F, 0x01, 0x01, 0x3F, 0x40, 0x40, 0x40, 0x3F,
  0x1F, 0x20, 0x40, 0x20, 0x1F, 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x63, 0x14, 0x08, 0x14, 0x63,
  0x07, 0x08, 0x70, 0x08, 0x07, 0x61, 0x51, 0x49, 0x45, 0x43, 0x00, 0x7F, 0x41, 0x41, 0x00,
  0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x41, 0x41, 0x7F, 0x00, 0x04, 0x02, 0x01, 0x02, 0x04,
  0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 0x01, 0x02, 0x04, 0x00, 0x20, 0x54, 0x54, 0x54, 0x78,
  0x7F, 0x48, 0x44, 0x44, 0x38, 0x38, 0x44, 0x44, 0x44, 0x20, 0x38, 0x44, 0x44, 0x48, 0x7F,
  0x38, 0x54, 0x54, 0x54, 0x18, 0x08, 0x7E, 0x09, 0x01, 0x02, 0x18, 0xA4, 0xA4, 0xA4, 0x7C,
  0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, 0x44, 0x7D, 0x40, 0x00, 0x40, 0x80, 0x84, 0x7D, 0x00,
  0x7F, 0x10, 0x28, 0x44, 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78,
  0x7C, 0x08, 0x04, 0x04, 0x78, 0x38, 0x44, 0x44, 0x44, 0x38, 0xFC, 0x24, 0x24, 0x24, 0x18,
  0x18, 0x24, 0x24, 0x18, 0xFC, 0x7C, 0x08, 0x04, 0x04, 0x08, 0x48, 0x54, 0x54, 0x54, 0x20,
  0x04, 0x3F, 0x44, 0x40, 0x20, 0x3C, 0x40, 0x40, 0x20, 0x7C, 0x1C, 0x20, 0x40, 0x20, 0x1C,
  0x3C, 0x40, 0x30, 0x40, 0x3C, 0x44, 0x28, 0x10, 0x28, 0x44, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C,
  0x44, 0x64, 0x54, 0x4C, 0x44, 0x08, 0x36, 0x41, 0x41, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00,
  0x00, 0x41, 0x41, 0x36, 0x08, 0x08, 0x04, 0x08, 0x10, 0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

// OLED variables
uint8_t OLED_x, OLED_y;                           // current cursor position
const uint16_t DIVIDER[] = {1, 10, 100, 1000, 10000}; // for BCD conversion

// OLED init function
void OLED_init(void) {
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_CMD_MODE);                       // set command mode
  for (uint8_t i = 0; i < sizeof(OLED_INIT_CMD); i++)
    I2C_write(OLED_INIT_CMD[i]);                  // send the command bytes
  I2C_stop();                                     // stop transmission
}

// OLED set the cursor
void OLED_setCursor(uint8_t xpos, uint8_t ypos) {
  xpos &= 0x7F; ypos &= 0x03;                     // limit cursor position values
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_CMD_MODE);                       // set command mode
  I2C_write(xpos & 0x0F);                         // set low nibble of start column
  I2C_write(0x10 | (xpos >> 4));                  // set high nibble of start column
  I2C_write(0xB0 |  ypos);                        // set start page
  I2C_stop();                                     // stop transmission
  OLED_x = xpos; OLED_y = ypos;                   // set the cursor variables
}

// OLED clear rest of the current line
void OLED_clearLine(void) {
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                       // set data mode
  while(OLED_x++ < 128) I2C_write(0);             // clear rest of the line
  I2C_stop();                                     // stop transmission
  OLED_setCursor(0, ++OLED_y);                    // set cursor to start of next line
}

// OLED clear screen
void OLED_clearScreen(void) {
  OLED_setCursor(0, 0);                           // set cursor to home position
  for(uint8_t i=4; i; i--) OLED_clearLine();      // clear all 4 lines
}

// OLED plot a single character
void OLED_plotChar(char c) {
  uint16_t ptr = c - 32;                          // character pointer
  ptr += ptr << 2;                                // -> ptr = (ch - 32) * 5;
  I2C_write(0x00);                                // write space between characters
  for (uint8_t i=5 ; i; i--) I2C_write(OLED_FONT[ptr++]);
  OLED_x += 6;                                    // update cursor
  if (OLED_x > 122) {                             // line end ?
    I2C_stop();                                   // stop data transmission
    OLED_setCursor(0,++OLED_y);                   // set next line start
    I2C_start(OLED_ADDR);                         // start transmission to OLED
    I2C_write(OLED_DAT_MODE);                     // set data mode
  }
}

// OLED print a string
void OLED_print(const char* str) {
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                       // set data mode
  while(*str) OLED_plotChar(*str++);              // plot each character
  I2C_stop();                                     // stop transmission
}

// OLED print a string with new line
void OLED_println(const char* str) {
  OLED_print(str);                                // print the string
  OLED_clearLine();                               // clear rest of the line
}

// OLED print value (BCD conversion by substraction method)
void OLED_printVal(uint16_t value) {
  uint8_t digits   = 5;                           // print 5 digits
  uint8_t leadflag = 0;                           // flag for leading spaces
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                       // set data mode
  while(digits--) {                               // for all digits digits
    uint8_t digitval = 0;                         // start with digit value 0
    uint16_t divider = DIVIDER[digits];           // read current divider
    while(value >= divider) {                     // if current divider fits into the value
      leadflag = 1;                               // end of leading spaces
      digitval++;                                 // increase digit value
      value -= divider;                           // decrease value by divider
    }
    if(!digits)  leadflag++;                      // least digit has to be printed
    if(leadflag) OLED_plotChar(digitval + '0');   // print the digit
    else         OLED_plotChar(' ');              // or print leading space
  }
  I2C_stop();                                     // stop transmission
}

// OLED print 8-bit value as 2-digit decimal (BCD conversion by substraction method)
void OLED_printDec(uint8_t value) {
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                       // set data mode
  uint8_t digitval = 0;                           // start with digit value 0
  while(value >= 10) {                            // if current divider fits into the value
    digitval++;                                   // increase digit value
    value -= 10;                                  // decrease value by divider
  }
  OLED_plotChar(digitval + '0');                  // print first digit
  OLED_plotChar(value + '0');                     // print second digit
  I2C_stop();                                     // stop transmission
}

// ===================================================================================
// DAC Implementation for Electronic Load
// ===================================================================================

// DAC reference voltages (load current = DAC voltage * R2 / (R1 + R2) / R4)
// Reference voltages:    0.55V, 1.1V, 1.5V, 2.5V, 4.3V
const uint8_t  DACREF[] = {0x00, 0x01, 0x04, 0x02, 0x03}; // CTRLA.DAC0REFSEL values
const uint16_t DACCUR[] = { 500, 1000, 1364, 2273, 3909}; // max current in mA
uint8_t DACreference = 0;                                 // start with 0.55V reference

// Setup the digital to analog converter (DAC)
void DAC_init(void) {
  pinDisable(PIN_DAC);                            // disable digital input buffer
  DAC0.CTRLA  = DAC_ENABLE_bm                     // enable DAC
              | DAC_OUTEN_bm;                     // enable output buffer
}

// Set load current
void DAC_setLoad(uint16_t current) {
  DACreference = 0;                               // start finding best reference
  if(current > 3000) current = 3000;              // limit current to max 300mA
  while(current > DACCUR[DACreference]) DACreference++; // search for best reference
  VREF_CTRLA &= 0xf8;                             // clear reference bits
  VREF_CTRLA |= DACREF[DACreference];             // set new reference
  _delay_us(25);                                  // wait for reference to settle
  DAC0.DATA = (uint32_t)255 * current / DACCUR[DACreference]; // set DAC accordingly
}

// Get load current
uint16_t DAC_readLoad(void) {
  return((uint32_t)DACCUR[DACreference] * DAC0.DATA / 255);   // return load current
}

// ===================================================================================
// ADC Implementation for Voltage Sensing and Button Detection
// ===================================================================================

// Button ADC thresholds
const uint16_t ADC_THRESHOLDS[] = {852, 597, 256, 0};
enum{KEY_NONE, KEY_START, KEY_CURR, KEY_VOLT};

// Init analog to digital converter (ADC)
void ADC_init(void) {
  pinDisable(PIN_KEYS);                         // disable digital input buffer
  pinDisable(PIN_VOLT);
  ADC0.CTRLA  = ADC_ENABLE_bm;                  // enable ADC with 10-bit resolution
  ADC0.CTRLD  = ADC_INITDLY_DLY64_gc;           // set init delay to 64 cycles
  VREF.CTRLA |= VREF_ADC0REFSEL_4V34_gc;        // select 4.3V ADC reference
  VREF.CTRLB |= VREF_ADC0REFEN_bm;              // keep the reference running
}

// ADC read button and return button number
uint8_t ADC_readButton(void) {
  ADC0.CTRLB   = 0;                             // single conversion
  ADC0.CTRLC   = ADC_SAMPCAP_bm                 // select sample capacitance
               | ADC_REFSEL_VDDREF_gc           // set Vdd as reference
               | ADC_PRESC_DIV8_gc;             // set prescaler -> 625kHz ADC clock
  ADC0.MUXPOS  = PIN_KEYS;                      // set the input pin
  ADC0.COMMAND = ADC_STCONV_bm;                 // start sampling
  while(ADC0.COMMAND & ADC_STCONV_bm);          // wait until sampling complete
  uint16_t raw = ADC0.RES;                      // read sampling result
  uint8_t  button = 0;                          // get button number ...
  while(raw < ADC_THRESHOLDS[button]) button++;
  return button;                                // return button number
}

// ADC read battery voltage in mV
uint16_t ADC_readVolt(void) {
  ADC0.CTRLB    = ADC_SAMPNUM_ACC64_gc;         // accumulate 64 samples
  ADC0.CTRLC    = ADC_SAMPCAP_bm                // select sample capacitance
                | ADC_REFSEL_INTREF_gc          // set internal reference
                | ADC_PRESC_DIV8_gc;            // set prescaler -> 625kHz ADC clock
  ADC0.MUXPOS   = PIN_VOLT;                     // set the input pin
  ADC0.INTFLAGS = ADC_RESRDY_bm;                // clear RESRDY flag
  ADC0.COMMAND  = ADC_STCONV_bm;                // start sampling
  while(~ADC0.INTFLAGS & ADC_RESRDY_bm);        // wait until sampling complete
  uint16_t raw = ADC0.RES;                      // read sampling result
  return((uint32_t)raw * 4300 / 65472);         // calculate and return voltage
}

// ===================================================================================
// Millis Counter using TCB Periodic Interrupt
// ===================================================================================

volatile uint32_t MIL_counter = 0;                // millis counter
#define MIL_start() TCB0.CTRLA = TCB_ENABLE_bm;   // (re)start the counter
#define MIL_stop()  TCB0.CTRLA = 0;               // stop/pause the counter

// Init millis counter (TCB)
void MIL_init(void) {
  TCB0.CCMP    = (F_CPU / 1000) - 1;              // set TOP value (period)
  TCB0.CTRLA   = TCB_ENABLE_bm;                   // enable timer/counter
  TCB0.INTCTRL = TCB_CAPT_bm;                     // enable periodic interrupt
}

// Read millis counter value
uint32_t MIL_read(void) {
  cli();                                          // disable interrupts for atomic read
  uint32_t result = MIL_counter;                  // get millis counter value
  sei();                                          // enable interrupts
  return result;                                  // return value
}

// Timer interrupt service routine (every millisecond)
ISR(TCB0_INT_vect) {
  TCB0.INTFLAGS = TCB_CAPT_bm;                    // clear interrupt flag
  MIL_counter++;                                  // increase millis counter
}

// ===================================================================================
// Main Function
// ===================================================================================

int main(void) {
  // Setup
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 3);         // set clock frequency to 5 MHz
  I2C_init();                                     // init I2C
  OLED_init();                                    // init OLED
  DAC_init();                                     // init DAC for electronic load
  ADC_init();                                     // init ADC for buttons and voltage
  MIL_init();                                     // init TCB for millis counter
  sei();                                          // enable interrupts

  // Local variables
  uint16_t volt_set = VOLT_START;                 // discharge termination voltage
  uint16_t curr_set = CURR_START;                 // discharge current
  uint16_t volt, curr, power;                     // voltage in mV, current in mA, power in mW
  uint16_t resistance;                            // internal battery resistance in mO
  uint32_t capacity, energy;                      // counter for capacity and energy
  uint32_t nextmillis;                            // for timing calculation in millis
  uint16_t seconds, minutes;                      // total duration in seconds and minutes

  // Loop
  while(1) {                                      // loop until forever
    // Set start screen
    OLED_clearScreen();
    OLED_setCursor(0, 1);
    OLED_println("Make your settings,");
    OLED_println("connect the battery");
    OLED_println("and press START.");

    // Get user settings for termination voltage and discharge current
    while((ADC_readButton() != KEY_START) || (ADC_readVolt() <= volt_set)) {
      OLED_setCursor(0, 0);
      OLED_print("Set: ");
      OLED_printVal(volt_set); OLED_print("mV ");
      OLED_printVal(curr_set); OLED_println("mA");
      
      if(ADC_readButton() == KEY_VOLT) {
        volt_set += VOLT_STEP;
        if(volt_set > VOLT_MAX) volt_set = VOLT_MIN;
        while(ADC_readButton());
      }

      if(ADC_readButton() == KEY_CURR) {
        curr_set += CURR_STEP;
        if(curr_set > CURR_MAX) curr_set = CURR_MIN;
        while(ADC_readButton());
      }
    }

    // Start discharge
    volt = ADC_readVolt();                        // read voltage prior to load
    DAC_setLoad(curr_set);                        // set load current
    seconds  = 0;                                 // reset discharge time
    capacity = 0;                                 // reset capacity counter
    energy   = 0;                                 // reset energy counter
    nextmillis = MIL_read() + 1000;               // first cycle
    OLED_setCursor(0, 1);
    OLED_println(""); OLED_println("<<<<< STARTING >>>>>"); OLED_println("");
    _delay_ms(200);                               // wait a little
    resistance = (uint32_t)1000 * (volt - ADC_readVolt()) / DAC_readLoad();

    // Discharge loop
    while(DAC0.DATA) {                            // loop until load is zero
      // Wait for next cycle (one cycle/second)
      while(MIL_read() < nextmillis);             // wait for the next cycle
      nextmillis += 1000;                         // set end time for next cycle
      
      // Read voltage and current
      volt = ADC_readVolt();                      // read battery voltage in mV
      curr = DAC_readLoad();                      // read load current    in mA

      // Check if discharge termination voltage is reached
      if(volt < volt_set) {                       // termination voltage reached?
        if(CYCLE_END) DAC0.DATA--;                // decrease load...
        else DAC0.DATA = 0;                       // ... or stop it
        if(volt < 500) DAC0.DATA = 0;             // stop if battery removed
      }

      // Calculate timings
      seconds += 1;                               // calculate total seconds
      minutes  = seconds / 60;                    // calculate minutes
      
      // Calculate power, capacity and energy
      power = (uint32_t)volt * curr  / 1000;      // calculate power in uW
      capacity += curr;                           // calculate capacity
      energy   += power;                          // calculate energy

      // Update OLED
      OLED_setCursor(0, 1);
      OLED_printVal(volt); OLED_print("mV      ");
      OLED_printVal(curr); OLED_println("mA");
      OLED_printVal(resistance); OLED_print("mO     ");
      OLED_printDec(minutes / 60); OLED_print(":");
      OLED_printDec(minutes % 60); OLED_print(":");
      OLED_printDec(seconds % 60); OLED_println("");
      OLED_printVal(capacity / 3600); OLED_print("mAh    ");
      OLED_printVal(energy   / 3600); OLED_println("mWh");
    }

    // Terminate discharge
    DAC_setLoad(0);
    OLED_setCursor(0, 1);
    OLED_print  ("Discharging finished,");
    OLED_println("remove the battery !");
    while((ADC_readButton() != KEY_START) || (ADC_readVolt() > 500));
  }
}
