/*
author: https://github.com/m8dhouse
date: 13/6/2020
License: https://creativecommons.org/licenses/by-nc-sa/4.0/
Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0) 

battery powered Arduino Pro mini
serial led + AM117 removed
power direct to vcc/ground using 3v lithium battery CR123A
ADXL345 on i2c
RFM68W radio with helical coil antenna + 10uf on power
MySensors 8266 GW with link to MQTT server
4 positions to control light intensity (1,2,3,4)
2 positions on cube with OFF (5,6)

*/

/*
based on work from TNTS
https://forum.arduino.cc/index.php?action=profile;u=720419
https://forum.arduino.cc/index.php?topic=486562.0

*/



/*********** INCLUDES SELECTION ***********/
#include <SparkFun_ADXL345.h>        
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>


/*********** VARS SELECTION ***********/
ADXL345 adxl = ADXL345();             
boolean volatile intADXL = false; //check if we woke up
double roll;
double pitch;
double xyz[3]; // xyz values
const float alpha = 0.5; // low pass filter

byte interrupts = 0;
unsigned int lastSideSeen = 0; // what was the last side we saw
const int offset = 5; //offset for approx xyz value

/****************** PINS ******************/
int interruptPin = 3; // IRQ on Pro mini
#define SensorREDLED 5  // RED led to show confirmation
#define SensorGREENLED 4  // GREEN led to show confirmation

// nested array - second number denotes how many row in the data array
// first value is pitch, second value is roll, last is value for MQTT - see above
const int cubesides[][3] = { {5,-4,1},{-85,-19,2},{2,85,3},{85,-47,4},{2,-85,5},{6,-180,6} };
// 1 = off, 2 = 25 , 3 = 50, 4 = 75, 5 = 100, 6 = off


/****************** MYSENSORS ******************/
// Enable debug prints to serial monitor
#define MY_DEBUG
#define MY_RADIO_RFM69
#define MY_RFM69_FREQUENCY RFM69_868MHZ // Set your frequency here
#define MY_NODE_ID 6
#define SKETCH_NAME "Cube Sensor"
#define SKETCH_MAJOR_VER "1"
#define SKETCH_MINOR_VER "0"
#define SECONDARY_CHILD_ID 3

#include <MySensors.h>

MyMessage msg(SECONDARY_CHILD_ID, V_TRIPPED);



/*********** SLEEP SELECTION ***********/
enum period_t
{
  SLEEP_15MS,
  SLEEP_30MS,
  SLEEP_60MS,
  SLEEP_120MS,
  SLEEP_250MS,
  SLEEP_500MS,
  SLEEP_1S,
  SLEEP_2S,
  SLEEP_4S,
  SLEEP_8S,
  SLEEP_FOREVER
};

enum bod_t
{
  BOD_OFF,
  BOD_ON
};

enum adc_t
{
  ADC_OFF,
  ADC_ON
};

enum timer5_t
{
  TIMER5_OFF,
  TIMER5_ON
};

enum timer4_t
{
  TIMER4_OFF,
  TIMER4_ON
};

enum timer3_t
{
  TIMER3_OFF,
  TIMER3_ON
};

enum timer2_t
{
  TIMER2_OFF,
  TIMER2_ON
};

enum timer1_t
{
  TIMER1_OFF,
  TIMER1_ON
};

enum timer0_t
{
  TIMER0_OFF,
  TIMER0_ON
};

enum spi_t
{
  SPI_OFF,
  SPI_ON
};

enum usart0_t
{
  USART0_OFF,
  USART0_ON
};

enum usart1_t
{
  USART1_OFF,
  USART1_ON
};

enum usart2_t
{
  USART2_OFF,
  USART2_ON
};

enum usart3_t
{
  USART3_OFF,
  USART3_ON
};

enum twi_t
{
  TWI_OFF,
  TWI_ON
};

enum usb_t
{
  USB_OFF,
  USB_ON
};

enum idle_t
{
  IDLE_0,
  IDLE_1,
  IDLE_2
};

// function to check if the pitch/roll is within the offset value
int isValueInBounds(int value, int midPoint){
  if ((value+offset>=midPoint) && (value-offset<=midPoint))
  {
  //position is confirmed - values matches within offset
   return true;  
  }
  return false;
  // weird angle - do nothing
  
}

// function to check if the value detected is the same as the previous one - if so nothing to send to MQTT
int checkNewValue(unsigned int newValue){

  if (lastSideSeen != newValue) {
    // we're at a different confirmed angle 
    lastSideSeen = newValue;
    for (int x = 0; x < 5; x++) {  
    digitalWrite(SensorREDLED,HIGH); //set Led blink 
    delay(50);
    digitalWrite(SensorREDLED,LOW);
    delay(50);
  }

    return true;
  }
  for (int x = 0; x < 5; x++) {  
    digitalWrite(SensorGREENLED,HIGH); //set Led blink 
    delay(50);
    digitalWrite(SensorGREENLED,LOW);
    delay(50);
  }
  //same angle as before - do nothing
  
  return false;
  
}


void presentation()
{
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER "." SKETCH_MINOR_VER);

	// Register binary input sensor to sensor_node (they will be created as child devices)
	// You can use S_DOOR, S_MOTION or S_LIGHT here depending on your usage.
	// If S_LIGHT is used, remember to update variable type you send in. See "msg" above.
	present(SECONDARY_CHILD_ID, S_DOOR);
}


/*********** IRQ from ADXL ***********/

void ADXL_ISR() {
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  intADXL = true;
}

/*********** SLEEP CODE ***********/
#define  lowPowerBodOn(mode) \
  do {            \
    set_sleep_mode(mode); \
    cli();        \
    sleep_enable();   \
    sei();        \
    sleep_cpu();      \
    sleep_disable();    \
    sei();        \
  } while (0);


#define  lowPowerBodOff(mode)\
  do {            \
    set_sleep_mode(mode); \
    cli();        \
    attachInterrupt(digitalPinToInterrupt(interruptPin), ADXL_ISR, RISING);   \
    sleep_enable();   \
    sleep_bod_disable(); \
    sei();        \
    sleep_cpu();      \
    sleep_disable();    \
    sei();        \
  } while (0);


#ifndef sleep_bod_disable
#define sleep_bod_disable()                     \
  do {                                \
    unsigned char tempreg;                          \
    __asm__ __volatile__("in %[tempreg], %[mcucr]" "\n\t"       \
                         "ori %[tempreg], %[bods_bodse]" "\n\t"     \
                         "out %[mcucr], %[tempreg]" "\n\t"      \
                         "andi %[tempreg], %[not_bodse]" "\n\t"     \
                         "out %[mcucr], %[tempreg]"           \
                         : [tempreg] "=&d" (tempreg)          \
                         : [mcucr] "I" _SFR_IO_ADDR(MCUCR),       \
                         [bods_bodse] "i" (_BV(BODS) | _BV(BODSE)), \
                         [not_bodse] "i" (~_BV(BODSE)));      \
  } while (0)
#endif

void  pDown(period_t period, adc_t adc, bod_t bod)
{
  if (adc == ADC_OFF) ADCSRA &= ~(1 << ADEN);

  if (period != SLEEP_FOREVER)
  {
    wdt_enable(period);
    WDTCSR |= (1 << WDIE);
  }
  if (bod == BOD_OFF)
  {

    lowPowerBodOff(SLEEP_MODE_PWR_DOWN);
  }
  else
  {

    lowPowerBodOn(SLEEP_MODE_PWR_DOWN);
  }

  if (adc == ADC_OFF) ADCSRA |= (1 << ADEN);
}



/******************** SETUP ********************/
/*          Configure ADXL345 Settings         */
void setupADXL() {
  attachInterrupt(digitalPinToInterrupt(interruptPin), ADXL_ISR, RISING);   // Attach Interrupt
  adxl.powerOn();                     // Power on the ADXL345
  adxl.setRangeSetting(16);           // Give the range settings
  adxl.setRate(100);
  adxl.setInterruptLevelBit(0);       
  adxl.setActivityXYZ(1, 1, 1);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setActivityThreshold(60);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)
  adxl.setTapDetectionOnXYZ(1, 1, 1); // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setDoubleTapLatency(80);       // 1.25 ms per increment
  adxl.setDoubleTapWindow(200);       // 1.25 ms per increment
  adxl.setImportantInterruptMapping(2, 1, 2, 1, 2);     // Sets "adxl.setEveryInterruptMapping(single tap, double tap, free fall, activity, inactivity);"
  adxl.InactivityINT(0);
  adxl.ActivityINT(1);
  adxl.FreeFallINT(0);
  adxl.doubleTapINT(1);
  adxl.singleTapINT(0);
}

void setup() {
  Serial.begin(115200);                 // Start the serial terminal
  pinMode(SensorREDLED, OUTPUT);
  pinMode(SensorGREENLED, OUTPUT);
  setupADXL();
}

void loop() {
  //Low Pass Filter
  xyz[0] = xyz[0] * alpha + (xyz[0] * (1.0 - alpha));
  xyz[1] = xyz[1] * alpha + (xyz[1] * (1.0 - alpha));
  xyz[2] = xyz[2] * alpha + (xyz[2] * (1.0 - alpha));

  roll  = (atan2(-xyz[1], xyz[2]) * 180.0) / M_PI;
  pitch = (atan2(xyz[0], sqrt(xyz[1] * xyz[1] + xyz[2] * xyz[2])) * 180.0) / M_PI;
 // Serial.println ("Pitch = " + (String)pitch); Serial.print (" - Roll = " + (String)roll); 

  if (digitalRead(interruptPin) == LOW) {
    adxl.getInterruptSource();
    adxl.ActivityINT(0);
    adxl.ActivityINT(1);
    Serial.println ("Sleep");
    delay(200);
    pDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  }
  else {
    while (  digitalRead(interruptPin) == HIGH) {
      // stabilise
      delay(100);
      adxl.get_Gxyz(xyz);

      // check if any of the values match a side
      for (int a = 0; a < 6; a++) {
        //iterate sides
        if ((isValueInBounds(pitch, cubesides [a][0])) && (isValueInBounds(roll, cubesides [a][1]))){
          
          // double match found now check if it's different from last time
          if (checkNewValue(cubesides [a][2]))
          {
            // yes new value - send to MQTT - blink x times as side number
            for (int x = 0; x < lastSideSeen; x++) {
              digitalWrite(SensorGREENLED,HIGH); //set Led blink 
              delay(500);
              digitalWrite(SensorGREENLED,LOW);
              delay(500);
            }

           Serial.println("MQTT _ SENDING");
           send(msg.set(lastSideSeen));
           delay(1000);

            // stop iteration
            break;

          }  else{
          // same value - do nothing - fast blink 
          // Serial.println("lastside " +lastSideSeen );
          // for (int x = 0; x < 5; x++) {
          //   digitalWrite(SensorREDLED,HIGH); //set Led blink 
          //   delay(50);
          //   digitalWrite(SensorREDLED,LOW);
          //   delay(50);
          // }

          // stop iteration as found something
          break;

          }
          
          }
        
        } // END CHECK

      // reset ADXL to go to sleep
      adxl.ActivityINT(0);
      adxl.getInterruptSource();
    }
    // reset IRQ flag
    intADXL = false;
  }
}



