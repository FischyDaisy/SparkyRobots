/*This is code for the Sparky Robot brain

This sketch recieves command values from the control Panel to control the Sparky robot.
It has a debug flag turned off for normal running.

It has an LED to indcate link status

Developed by Miss Daisy FRC Team 341
 
*/
#include <Servo.h>
#include <EasyTransfer.h>
#include <SparkyXfrBuffers.h>
#include <NewPing.h>
#include <EEPROM.h>

// declare local mode routines
void enabledState(void);
void disabledState(void);
void calibrationAndTests(void);

const int TRIGGER_PIN_10 = 10; // Arduino pin tied to trigger pin on the ultrasonic sensor.
const int ECHO_PIN_8     =  8;      // Arduino pin tied to echo pin on the ultrasonic sensor.
const int MAX_DISTANCE   = 80; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
const int BALL_DISTANCE  = 36;   // anything smaller means we see a ball

NewPing sonar( TRIGGER_PIN_10, ECHO_PIN_8, MAX_DISTANCE); // NewPing constructor, sets pins and maximum distance.

//create two transfer objects
EasyTransfer ETin, ETout; 

//  global timing variables
long unsigned int lastUpdateTime = 0; // asynchronous link verification 
long unsigned int lastBlinkToggle = 0; // link status LED, follows its own definite timing

// filter accumulators
int VIN_accum;

// Message count tracking
long int lastMessageCounter = 0;
long int messageDropCounter = 1000;

//  declare servo objects
Servo leftDriveMotor;
Servo rightDriveMotor;
Servo intakeMotor;
Servo conveyorMotor;
Servo shooterMotor;

// Servo ouptut is from 0 to 179
const int servoHaltVal     = 90;   // 90 is no motion
const int servoFullForeVal = 179;  // 179 is full forward
const int servoFullBackVal = 0;    // 0 is full reverse

// Stick input is from 0 to 1023
const int stickHaltVal     = 512;   // this is center, no motion

// declare EEPROM addresses and variables to store values locally
const int maxKnobAddr = 0;
int16_t shootSpeedKnobMax;  // size of int16_t is always 2
const int minKnobAddr = 2;
int16_t shootSpeedKnobMin;

//  declare the transfer buffers
TO_SPARKY_DATA_STRUCTURE rxdata;
FROM_SPARKY_DATA_STRUCTURE txdata;

const int BALL_OVERRIDE_2    =  2;
const int TEST_SWITCH_4      =  4;
const int LINK_STATUS_LED_11 = 11; 
const int LINK_DATA_TEST_12  = 12;
const int LINK_DATA_LED_13   = 13;
const int VIN_PIN_0          = 0;

///////////////////// SETUP, called once at start ///////////////////////////////////////////////
void setup(){
  // Serial is connected throught the Bluetooth modules to the master
  Serial.begin(9600);
  while (!Serial) ; // wait for serial port to connect. Needed for native USB

  //start the library, pass in the data details and the name of the serial port.
  ETin.begin(details(rxdata), &Serial);
  ETout.begin(details(txdata), &Serial);

  // set transmitter buffer to default values
  txdata.buttonstate = HIGH;
  txdata.supplyvoltagereading = analogRead(VIN_PIN_0);
  txdata.ballready = false;
  txdata.packetreceivedcount = 0;

  // init rxdata to safe values, in case they are used before first packet sets them
  rxdata.stickLx = stickHaltVal;
  rxdata.stickLy = stickHaltVal;
  rxdata.stickLbutton = HIGH;
  rxdata.stickRx = stickHaltVal;
  rxdata.stickRy = stickHaltVal;
  rxdata.stickRbutton = HIGH;
  rxdata.shooterspeed = 1023; // 1023 is lowest speed, pot wired 5V ccw
  rxdata.intake = HIGH;
  rxdata.shoot = HIGH;
  rxdata.drivemode = HIGH;
  rxdata.enabled = LOW;
  rxdata.counter = -1;

// pin 0 is rx, 1 is tx - for serial port, not used as DIO
                            pinMode( BALL_OVERRIDE_2, INPUT_PULLUP); // pin 2 used during test mode as ball override
  leftDriveMotor.attach(3);
                            pinMode( TEST_SWITCH_4, INPUT_PULLUP);
  rightDriveMotor.attach(5);
  intakeMotor.attach(6);
  shooterMotor.attach(7);
                            // pin 8 set to sonar sensor output, pinmode input, by newping constructor
  conveyorMotor.attach(9);
                            // pin 10 set to sonar sensor input, pinmode output, by newping constructor
                            pinMode(LINK_STATUS_LED_11, OUTPUT);  
                            pinMode(LINK_DATA_TEST_12, INPUT_PULLUP); // push_button for link data test
                            pinMode(LINK_DATA_LED_13, OUTPUT);       // link data test output

  /////   sync with EEPROM
  // NOTE:  speed knob is wired so max (cw) is near 0 and min (ccw) is near 1023
  EEPROM.get( maxKnobAddr, shootSpeedKnobMax);  // read int in
  if ( shootSpeedKnobMax < 0 || shootSpeedKnobMax > 256 ) { // if not in range, probably never calibrated
    shootSpeedKnobMax = 256; // max must be positive, start high, calibrate down at runtime
    EEPROM.put( maxKnobAddr, shootSpeedKnobMax);
  }
  EEPROM.get( minKnobAddr, shootSpeedKnobMin);
  if ( shootSpeedKnobMin < 768 || shootSpeedKnobMin > 1023 ) { // if not in range, probably never calibrated
    shootSpeedKnobMin = 768; // Min must be positive, start low, calibrate up at runtime
    EEPROM.put( minKnobAddr, shootSpeedKnobMin);
  }

}

void loop(){
  //each time we will unconditionally go ahead and send the data out
  txdata.transmitpacketcount++;
  ETout.sendData();
    
  //there's a loop here so that we run the recieve function more often then the 
  //transmit function. This is important due to the slight differences in 
  //the clock speed of different Arduinos. If we didn't do this, messages 
  //would build up in the buffer and appear to cause a delay.
 
  for(int i=0; i<5; i++){
    // Check if new data packets were received
    if (ETin.receiveData()){
      txdata.packetreceivedcount++;
      lastUpdateTime = millis();
    }
    
    if ((rxdata.counter - lastMessageCounter) == 0){
      messageDropCounter += 1;
    }else{
      messageDropCounter = 0;
    }

    // pull rx data before checking state
    // Check that the sparky is safe to operate
    if ( /*rxdata.counter > 0 &&*/ rxdata.enabled > 0 && messageDropCounter <= 10 ) {

      // An update was received recently, process the data packet
      enabledState();
    } else {
      // There are no new data packets to control the robot, turn everything off so
      // the robot doesn't continue issuing the last received commands
      disabledState();
    }
    lastMessageCounter = rxdata.counter;
    
    delay(10);   // delay between each read
  }
    //delay for good measure before write routine
  delay(10);

  //  check for TEST mode
  if ( !digitalRead( TEST_SWITCH_4)  ) {  // LOW is active
    txdata.buttonstate = !digitalRead( LINK_DATA_TEST_12 ); // when active send pin 12
    calibrationAndTests();      // run testing routine, returns immediately unless cal is signaled
  } else {
    txdata.buttonstate = -1;  // TEST not active
  }

  // scale and filter the voltage reading, accum is 16 times reading
  VIN_accum = VIN_accum - (VIN_accum>>4)  + ((analogRead(VIN_PIN_0)*32)/20);
  txdata.supplyvoltagereading = VIN_accum>>4;
   
} // end of loop

///////////////////  set ball presence  //////////////////
boolean isBallPresent() {
  int sonarDistance, ballReady;
  sonarDistance = sonar.ping_cm();
  if ( rxdata.enabled ) {
    if ( sonarDistance <= BALL_DISTANCE || sonarDistance >= MAX_DISTANCE ) { // if ball is where it needs to be
      ballReady = true;
    } else { 
      ballReady = false;
    }
  } else {
      ballReady = sonarDistance; // for testing, it is not referenced, just sent to panel
  }
  // when sensor is not working it says ball is present, need to say no ball to test system
  // when sensor is working we need to say the ball is there to test the system
  if ( !digitalRead(BALL_OVERRIDE_2) ) {   // LOW is active, invert ball present state
    ballReady = !ballReady;
  }
  txdata.ballready = ballReady;
  return ballReady;
}

///////////////////  apply stick profile  //////////////////
// argument:   stick value from receive buffer, 0 - 1023
// Return:     servo value, 0-179, scaled and profile applied
// profile:   Apply a deadband to all joystick values so that 
//            anything between +/-50 from stick center is converted to servo center.
const long int deadband = 50;
//------------------------------------------------
int convertStickToServo(int stickValue) {
  long int longServoValue;  // use longs, 180*1024 > 32768

  if ( stickValue > (stickHaltVal + deadband) ) {
    longServoValue = ( ((long)(stickValue - deadband)) * 180) >> 10; // /1024; // servo range / stick range
  } else {
    if ( stickValue < (stickHaltVal - deadband) ) {
      longServoValue = ( ((long)(stickValue + deadband)) * 180) >> 10; //  /1024;
    } else {
      longServoValue = servoHaltVal; //else stick in deadband, set Halt value
    }
  }
  return (int) longServoValue;
}


//////////////  stop all activity if communications not working
void disabledState(){
  // One or more conditions are not satisfied to allow the sparky to operate, disable all motors

  // Set all speed controllers to output 0V.
  leftDriveMotor.write(servoHaltVal);
  rightDriveMotor.write(servoHaltVal);
  intakeMotor.write(servoHaltVal);
  conveyorMotor.write(servoHaltVal);
  shooterMotor.write(servoHaltVal);
  txdata.shooterspeedecho = -rxdata.shooterspeed; 
  isBallPresent();   // check ball for testing purposes
 
  // do a fast blink
  if ( lastBlinkToggle < millis()-200 ) { //if more than a 1/5 second ago
    lastBlinkToggle = millis();  // triggered and reset.
    if ( bitRead( PORTB,3) ) {   // this how to read an output pin
      digitalWrite( LINK_STATUS_LED_11, LOW);
    } else {
      digitalWrite( LINK_STATUS_LED_11, HIGH);
    }
  }
}

 
unsigned long shootReleaseTime = 0;
/////////////////////  enabledState   /////////////////////////////
void enabledState(){
  int shooterSpeed, rawShooterSpeed;
  
  // If in the enabled state, the sparky bot is allowed to move 

  // Steer the robot based on selected drive mode
  int leftMotorSpeed = servoHaltVal;
  int rightMotorSpeed = servoHaltVal;
  if (rxdata.drivemode < 1){
    // Tank Mode - left joystick control left drive, right joystick controls right drive
    leftMotorSpeed  = convertStickToServo(rxdata.stickLx); 
    rightMotorSpeed = convertStickToServo(rxdata.stickRx);
  } else {
    // Arcade Mode - left joystick controls speed, right joystick controls turning
    int speedVal = convertStickToServo(rxdata.stickLx); 
    int  turnVal = convertStickToServo(rxdata.stickRy);
    leftMotorSpeed  = speedVal + turnVal - 90;
    rightMotorSpeed = speedVal - turnVal + 90;
    
    if ( leftMotorSpeed < 0  ) leftMotorSpeed  = 0;    // eg.   0   + 0   - 90
    if ( leftMotorSpeed > 179) leftMotorSpeed  = 179;  // eg.   180 + 180 - 90
    if (rightMotorSpeed < 0  ) rightMotorSpeed = 0;    // eg.   0   - 180 + 90
    if (rightMotorSpeed > 179) rightMotorSpeed = 179;   // eg.   180 - 0   + 90
  }
  // Issue the commanded speed to the drive motors
  // both motors spin ull clockwise for 179, left motor mounted opposite direction, so
  leftDriveMotor.write(179 - leftMotorSpeed); // left wheel must spin opposite
  rightDriveMotor.write(rightMotorSpeed);
  txdata.leftmotorcommand = leftMotorSpeed;
  txdata.rightmotorcommand = rightMotorSpeed;

///////  BELT FUNCTIONS: ONLY ENABLED WHEN BALL IS NOT IN SHOOTER  ////////////
//      SHOOTER FUNCTIONS: ONLY ENABLED WHEN BALL IS IN SHOOTER

  ///////  calculate shooter wheel speed    ////////////
  // NOTE:  speed knob is wired so max (cw) is near 0 and min (ccw) is near 1023
  // when transmitted 1023 is lowest speed, 0 is highest
  rawShooterSpeed = 1023 - rxdata.shooterspeed; // invert it to a natural 0 low, 1023 high
  if ( rawShooterSpeed > shootSpeedKnobMax  ) { //recalibrate, first time seeing value this high
    shootSpeedKnobMax = rawShooterSpeed;
    EEPROM.put( maxKnobAddr, shootSpeedKnobMax);
  }
  if ( rawShooterSpeed < shootSpeedKnobMin  ) { //recalibrate, first time seeing value this low
    shootSpeedKnobMin = rawShooterSpeed;
    EEPROM.put( minKnobAddr, shootSpeedKnobMin);
  }
  // Map the potentiometer dial (min to max) to a valid positive shooter speed (90+20 to 179), 20 is min speed
  shooterSpeed = map(rawShooterSpeed, shootSpeedKnobMin, shootSpeedKnobMax, servoHaltVal+20, servoFullForeVal);
  txdata.shooterspeedecho = shooterSpeed;      // assigning shooterSpeed to echo for testing
  
  if ( isBallPresent() ) {
    //  light the green ballLEDs   TBD
    
   // Run the shooter
    shooterMotor.write(shooterSpeed);  // ball is here, run shooter motor
    if (rxdata.shoot > 0 ) {    // shooter button pressed
      shootReleaseTime = millis() + 1500;   // trigger and hold shoot even if button released
    } else {
      // Stop the conveyor
      conveyorMotor.write(servoHaltVal);
    }
    intakeMotor.write(servoHaltVal);   // in case this is first time seeing ball
    
  } else {   // no ball   //////////////
    // turn ballLEDs off TBD
    
    shooterMotor.write(servoHaltVal);   ///off shooterSpeed);
    if (rxdata.intake > 0){  // intake button pressed 
      // Run the intake feed
      intakeMotor.write(servoFullBackVal);
      conveyorMotor.write((servoFullBackVal*3)/4);  // (.75 speed)
      digitalWrite(LINK_DATA_LED_13, LOW); 
    } else {
      // Stop the intake
      intakeMotor.write(servoHaltVal);
      conveyorMotor.write(servoHaltVal);
      digitalWrite(LINK_DATA_LED_13, HIGH); 
    }
  }

  // if a shoot is in progress
  if ( shootReleaseTime >= millis() ) {   // if shoot release is in the future
    // Run the conveyor forward
    conveyorMotor.write(servoFullBackVal);
    // ball may be gone from sensor, but shoot still in progress, run shooter motor
    shooterMotor.write(shooterSpeed);  
  }
  
  // do a slow blink to show enabled and running
  if ( lastBlinkToggle < millis()-1000 ) { //if more than a second ago
    lastBlinkToggle = millis();
    if ( bitRead( PORTB, 3) ) {   // check the output pin
      digitalWrite( LINK_STATUS_LED_11, LOW);
    } else {
      digitalWrite( LINK_STATUS_LED_11, HIGH);
    }
  }
}   // end of enabledState()

////////////   doCalibrationSweep   //////////////
void doCalibrationSweep( Servo *theservo ) {   // note that this routine is blocking
  // make a calibration start sound  TBD

  for (int i = 90; i >= 0; i -= 5 ) {
    theservo->write( i );
    delay(50);
    digitalWrite( LINK_STATUS_LED_11,  !bitRead( PORTB,3) ); // flip the LED fast
  }
  delay(1000);
  for (int i = 9; i <= 179; i += 5 ) {
    theservo->write( i );
    delay(50);
    digitalWrite( LINK_STATUS_LED_11,  !bitRead( PORTB,3) ); // flip the LED fast
  }
  delay(1000);
  for (int i = 179; i >= 90; i -= 5 ) {
    theservo->write( i );
    delay(50);
    digitalWrite( LINK_STATUS_LED_11,  !bitRead( PORTB,3) ); // flip the LED fast
 }
  theservo->write(servoHaltVal); 
  delay(500);
  digitalWrite( LINK_STATUS_LED_11,  HIGH ); // flip the LED off to end
  delay(3000);    //before going back to test mode
   
  // make a calibration done sound  TBD
}

//  ////////////////   SW 2 is on, test mode.     //////////////
void calibrationAndTests(){
  leftDriveMotor.write(servoHaltVal);
  rightDriveMotor.write(servoHaltVal);
  shooterMotor.write(servoHaltVal);   ///off shooterSpeed);
  intakeMotor.write(servoHaltVal);
  conveyorMotor.write(servoHaltVal);

/////// code for calibrating computer controlled servos
  if (rxdata.intake > 0){  // intake button pressed 
    delay(500);
    if (rxdata.intake > 0){  // intake button still pressed 
      if (rxdata.shoot > 0 ) { // shooter button now also pressed
        doCalibrationSweep( &shooterMotor );
      } else {   // only the intake pressed
        doCalibrationSweep( &intakeMotor );
      }
    }
  }
  if (rxdata.shoot > 0){  // shoot button pressed 
    delay(500);
    if (rxdata.shoot > 0){  // shot button still pressed 
      if (rxdata.intake > 0 ) { // intake button now also pressed
        doCalibrationSweep( &shooterMotor );
      } else {   // only the intake pressed
        doCalibrationSweep( &intakeMotor );
      }
    }
  }

  isBallPresent();   // check ball for test  transmit purposes
  
  // do a quick blink
  if ( lastBlinkToggle < millis()-200 ) { //if more than a 1/5 second ago
    lastBlinkToggle = millis();
    static int interval = 0;
    if ( !bitRead( PORTB,3) ) {   // this how to read an output pin
      digitalWrite( LINK_STATUS_LED_11, HIGH);
    } else {
      if ( ++interval > 5 ) {
        digitalWrite( LINK_STATUS_LED_11, LOW);
        interval = 0;
      }
    }
  }  // end of blink
} // end of function
