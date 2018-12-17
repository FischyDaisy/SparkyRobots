/*This is code for the Sparky Control Panel

This sketch reads controls and sends command values to the Sparky robot.
It has a debug flag turned off for normal running.

It has controlled 5 bar LED showing activity in the panel.

Developed by Miss Daisy FRC Team 341
 
*/

//  this is for test mode
#include <AltSoftSerial.h>
AltSoftSerial altser;
const int mybaud = 600;
boolean runTimeMonitorEnabled = false;
// D8 - AltSoftSerial RX
// D9 - AltSoftSerial TX

#include <EasyTransfer.h>
//create two objects
EasyTransfer ETin, ETout; 

// Transfer data strucures must be the same at both ends
//   so, they are defined in a library included by both sketches
#include <SparkyXfrBuffers.h>

// declare global structures for data used by the transfer objects
// reverse naming on opposite ends
FROM_SPARKY_DATA_STRUCTURE rxdata;
TO_SPARKY_DATA_STRUCTURE txdata;

#define DRIVE_MODE        13 
#define SYSTEM_ENABLE   12
//  #define PANEL_LED_5     11  panel LEDs are controlled by setLED function, 0 to 4
//  #define PANEL_LED_4     10
//  #define PANEL_LED_3     9
#define SHOOT_BUTTON    8
#define INTAKE_BUTTON   7
//  #define PANEL_LED_2     6
//  #define PANEL_LED_1     5
#define TEST_SWITCH     4
#define R_STICK_BUTTON  3
#define L_STICK_BUTTON  2

#define L_STICK_X      0    // left stick attached to A0,A1,D2
#define L_STICK_Y      1
#define R_STICK_X      2    //right stick attached to A2,A3,D3
#define R_STICK_Y      3
#define SHOOTERSPEED 4

unsigned long triggerTime;
unsigned long headingTime;
const int panelLedArr[5] = {5,6,9,10,11}; //map of wired pins to LEDs
long int messageCounter = 0;

/////////////////////////////////////////////////////////////////////////////
// FUNCTION: setLED,   returns nothing
// ARGUMENTS: LEDnum is value 0 to 4, brightness is 0 (off) to 255 (full on)
void setLED(int LEDnum, unsigned int brightness) {
  if ( brightness > 255 ) brightness = 255;
  unsigned long brightness_l = brightness & 255; 
  //  index to pins, use panelLedArr   ,  value to write is non-linear
  if ( 0 <= LEDnum < 5 ) {
    int LEDoutput = min( 255-((brightness_l+7)/8), (255-brightness_l)*3 );
   analogWrite( panelLedArr[LEDnum], LEDoutput );
  }
}
/////////////////////////////////////////////////////////////////////////////////
// called once at start
void setup(){
  Serial.begin(9600);
  while (!Serial) ; // wait for serial port to connect. Needed for native USB
  
  //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
  ETin.begin(details(rxdata), &Serial);
  ETout.begin(details(txdata), &Serial);

  altser.begin(600);

  //  init LEDs     //////////////////////////
  for (int i=0; i<5; i++) {
    pinMode( panelLedArr[i], OUTPUT);
    setLED( i, 255); // on
    delay(1000);
    setLED( i, 0);
  }
  
  // init inputs and enable pullup
  pinMode(DRIVE_MODE      , INPUT_PULLUP); // 13
  pinMode(SYSTEM_ENABLE   , INPUT_PULLUP); // 12 LOW is ENABLEd
  pinMode(SHOOT_BUTTON    , INPUT_PULLUP); // 8 LOW is SHOOT
  pinMode(INTAKE_BUTTON   , INPUT_PULLUP); // 7 LOW is INATKE
  pinMode(TEST_SWITCH     , INPUT_PULLUP); // 4 LOW is on
  pinMode(R_STICK_BUTTON  , INPUT_PULLUP); // 3 LOW is active
  pinMode(L_STICK_BUTTON  , INPUT_PULLUP); // 2 LOW is active

  triggerTime = millis() + 3000;  // 3 seconds from now
  delay(2000);
  altser.println();
  altser.print("Sparky Control Panel    :Created ");
  altser.print( __DATE__ );
  altser.print(" ");
  altser.println( __TIME__ );
  if ( digitalRead(TEST_SWITCH) == HIGH ) { // LOW is on
    runTimeMonitorEnabled = false;
    altser.println("Runtime Monitor is disabled");
  }
}
/////////////////////  MAIN LOOP  /////////////////////////////
void loop(){
  // read our potentiometers and buttons and store raw data in data structure for transmission
  txdata.stickLx = analogRead(L_STICK_X);
  txdata.stickLy = analogRead(L_STICK_Y);
  txdata.stickLbutton = !digitalRead(L_STICK_BUTTON);
  txdata.stickRx = analogRead(R_STICK_X);
  txdata.stickRy = analogRead(R_STICK_Y);
  txdata.stickRbutton = !digitalRead(R_STICK_BUTTON);

  txdata.drivemode = !digitalRead(DRIVE_MODE);
  txdata.enabled = !digitalRead(SYSTEM_ENABLE);
  
  txdata.shooterspeed = 1023-analogRead(SHOOTERSPEED);

  int buttonValue = 240;
  if( !digitalRead(INTAKE_BUTTON) ){
    txdata.intake = 1;
    buttonValue -= 80;
  }
  else {
    txdata.intake = 0;
  }
  if( !digitalRead(SHOOT_BUTTON) ){
    txdata.shoot = HIGH;
    buttonValue -= 80;
  }
  else {
    txdata.shoot = LOW;
  }
  // Check if the controller is enabled
  if(txdata.enabled){
    buttonValue -= 80;
    messageCounter += 1;
    txdata.counter = messageCounter;
  }
  //then we will go ahead and send that data out
  ETout.sendData();

  
 //there's a loop here so that we run the recieve function more often then the 
 //transmit function. This is important due to the slight differences in 
 //the clock speed of different Arduinos. If we didn't do this, messages 
 //would build up in the buffer and appear to cause a delay.
  for(int i=0; i<5; i++){
    //remember, you could use an if() here to check for new data, this time it's not needed.
    ETin.receiveData();
    delay(10);   // short delay between receive attempts
  }
  
  //delay for good measure
  delay(10);

//  *********  RUNTIME DISPLAY or DISPLAY TESTING *****************
  // check is display test on
  if ( digitalRead(TEST_SWITCH) == LOW ) { // LOW is on
    // ****  DISPLAY TESTING ******
    if ( !runTimeMonitorEnabled ) {
      altser.println("Monitor Activated");
    }
    runTimeMonitorEnabled = true;
    unsigned long testnow = millis();
    // once per second 
    if ( testnow >= triggerTime ) {
      triggerTime = testnow + 2000;
      int value1 = (analogRead(SHOOTERSPEED)+3)>>2;
      if ( value1 < 128 ) {
        setLED( 0, (127-value1)*2 );
        setLED( 1, 0);
      } else {
        setLED( 0, 0);
        setLED( 1, (value1-128)*2 );
      }
      setLED( 4, buttonValue);

      // ****  AltSerial TESTING ****
      if ( headingTime > testnow ) {
        altser.print( rxdata.supplyvoltagereading);
        altser.print( ", " );
        altser.print( rxdata.transmitpacketcount );
        altser.print( ", " );
        altser.print( rxdata.packetreceivedcount );
        altser.print( ", " );
        altser.print( rxdata.shooterspeedecho );
        altser.print( ", " );
        altser.print( rxdata.buttonstate );
        altser.print( ", " );
        altser.print( rxdata.ballready );
        altser.print( ", " );
        altser.println( txdata.counter );
        
      } else {
        altser.println( "VOLT XMIT RECV SSECHO D12 BALL" );
        headingTime = testnow + 20000;
      }
    }
  }
//////////////////////   NORMAL DISPLAY   ////////////////////////
  else {  // test mode OFF, show main robot control signals
    static unsigned long updateDue = 0;
    unsigned long now = millis();
    if ( runTimeMonitorEnabled ) {
      altser.println("Monitor Deactivated");
    }
    runTimeMonitorEnabled = false;
    if ( now > updateDue ) {
      updateDue = now + 100; // 10 updates per second, max
      setLED( 0, (analogRead(L_STICK_X)+3)>>2);
      setLED( 1, (analogRead(R_STICK_X)+3)>>2);
      setLED( 4, buttonValue);
    }  
  }
}

/*
 * 
 * 
Controller Connections 
Legend: 
A - Analog input
D - Digital Input
Left stick:
===========
ground
5V
xval = A0
yval = A1
button = D2
Right Stick:
============
ground
5v
xval = A2
yval = A3
button = D3
===============
Shooter Knob = A4
  -> has a 5v pin
  -> signal
  -> has a ground pin
  ===============
Intake Button = D7
  -> has a pin to ground
==========================
Shoot Button = D8
  -> has a pin to ground
===========================  
Enable Toggle = D12
  -> has a pin to ground
====================================
shooter knob   5v,sig,GND for a pot,  3 others  2-pin GND,sig.    
 D2,3 for stick buttons. 
 Put LEDs and test switch on others D4 - D9. 


SYSTEM_ENABLE   
SHOOT_BUTTON    
INTAKE_BUTTON   
TEST_SWITCH     
R-STICK_BUTTON  
L_STICK_BUTTON 

#define BLINKLED        13 
#define PANEL_LED_5     11
#define PANEL_LED_4     10
#define PANEL_LED_3     9
#define PANEL_LED_2     6
#define PANEL_LED_1     5


L_STICK_X
L_STICK_Y
R_STICK_X
R_STICK_Y
SHOOTERSPEED 
*/
