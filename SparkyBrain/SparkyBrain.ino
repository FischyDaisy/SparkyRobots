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

void enabledState(void);
void disabledState(void);
void localRobotRoutine(void);

const int TRIGGER_PIN_10 = 10; // Arduino pin tied to trigger pin on the ultrasonic sensor.
const int ECHO_PIN_8     =  8;      // Arduino pin tied to echo pin on the ultrasonic sensor.
const int MAX_DISTANCE   = 80; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
const int BALL_DISTANCE  = 36;   // anything smaller means we see a ball

NewPing sonar( TRIGGER_PIN_10, ECHO_PIN_8, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

//create two transfer objects
EasyTransfer ETin, ETout; 

//  global timing variables
long unsigned int lastUpdateTime = 0; // asynchronous link verification 
long unsigned int lastBlinkToggle = 0; // link status LED, follows its own definite timing
static int VIN_accum;

long int lastMessageCounter = 0;
long int messageDropCounter = 1000;

//  declare servo objects
Servo leftDriveMotor;
Servo rightDriveMotor;
Servo intakeMotor;
Servo conveyorMotor;
Servo shooterMotor;

// Servo ouptut is between 0 and 179
const int servoHaltVal     = 90;   // 90 is no motion
const int servoFullForeVal = 179;  // 179 is full forward
const int servoFullBackVal = 0;    // 0 is full reverse

//  declare the transfer buffers
TO_SPARKY_DATA_STRUCTURE rxdata;
FROM_SPARKY_DATA_STRUCTURE txdata;

const int TEST_SWITCH_4      =  4;
const int LINK_STATUS_LED_11 = 11; 
const int LINK_DATA_TEST_12  = 12;
const int LINK_DATA_LED_13   = 13;
const int VIN_PIN_0          = 0;

///////////////////// SETUP, called once at start ///////////////////////////////////////////////
void setup(){
  Serial.begin(9600);
  //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
  ETin.begin(details(rxdata), &Serial);
  ETout.begin(details(txdata), &Serial);

  txdata.buttonstate = HIGH;
  txdata.supplyvoltagereading = analogRead(VIN_PIN_0);
  txdata.ballready = false;
  txdata.packetreceivedcount = 0;

  // init rxdata to safe values for before first packet
  rxdata.stick1x = 512;
  rxdata.stick1y = 512;
  rxdata.stick1button = HIGH;
  rxdata.stick2x = 512;
  rxdata.stick2y = 512;
  rxdata.stick2button = HIGH;
  rxdata.shooterspeed = 1023;
  rxdata.intake = HIGH;
  rxdata.shoot = HIGH;
  rxdata.drivemode = HIGH;
  rxdata.enabled = LOW;
  rxdata.counter = -1;

// pin 0 is rx, 1 is tx - for serial port, not used as DIO
// pin 2 not used
  leftDriveMotor.attach(3);
                            pinMode( TEST_SWITCH_4, INPUT_PULLUP);
  rightDriveMotor.attach(5);
  intakeMotor.attach(6);
  shooterMotor.attach(7);
                            // pin 8 set to sonar sensor input, pinmode output, by newping declare
  conveyorMotor.attach(9);
                            // pin 10 set to sonar sensor output, pinmode input, by newping declare
                            pinMode(LINK_STATUS_LED_11, OUTPUT);  
                            pinMode(LINK_DATA_TEST_12, INPUT_PULLUP); // push_button for link data test
                            pinMode(LINK_DATA_LED_13, OUTPUT);       // link data test output
}

void loop(){
  //then we will always go ahead and send the data out
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
    
    // Check that the sparky is safe to operate
//    if (  { millis() - lastUpdateTime) < 250 ) {
    if ( rxdata.counter > 0 && rxdata.enabled > 0 && messageDropCounter <= 10 ) {

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

  txdata.buttonstate = digitalRead( TEST_SWITCH_4 );
  if ( !txdata.buttonstate ) {
    localRobotRoutine(); 
  }  //end of test if

  // scale and filter the voltage reading, accum is 16 times reading
  VIN_accum = VIN_accum - (VIN_accum>>4)  + ((analogRead(VIN_PIN_0)*32)/20);
  txdata.supplyvoltagereading = VIN_accum>>4;
   
} // end of loop

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
  txdata.ballready = sonar.ping_cm();
  
  // do a fast blink
  if ( lastBlinkToggle < millis()-200 ) { //if more than a 1/5 second ago
    lastBlinkToggle = millis();
    if ( bitRead( PORTB,3) ) {   // this how to read an output pin
      digitalWrite( LINK_STATUS_LED_11, LOW);
      txdata.ballready = sonar.ping_cm();
    } else {
      digitalWrite( LINK_STATUS_LED_11, HIGH);
      txdata.ballready = false;
    }
  }
}

int shooterSpeed; 
int sonarDistance;

void enabledState(){
  // If in the enabled state, the sparky bot is allowed to move 

  // Apply a deadband to all joystick values so that anything between +/-50 from center is considered center
  if (rxdata.stick1x > 462 && rxdata.stick1x < 562){
    rxdata.stick1x = 512;
  }
  if (rxdata.stick1y > 462 && rxdata.stick1y < 562){
    rxdata.stick1y = 512;
  }
  if (rxdata.stick2x > 462 && rxdata.stick2x < 562){
    rxdata.stick2x = 512;
  }
  if (rxdata.stick2y > 462 && rxdata.stick2y < 562){
    rxdata.stick2y = 512;
  }
  
  // Steer the robot based on selected drive mode
  int leftMotorSpeed = servoHaltVal;
  int rightMotorSpeed = servoHaltVal;
  if (rxdata.drivemode < 1){
    // Tank Mode - left joystick control left drive, right joystick controls right drive
    // Need to remap the joystick value range of 0-1023 to 0-179
    leftMotorSpeed = map(rxdata.stick1x, 0, 1023, 0, 179);
    rightMotorSpeed = map(rxdata.stick2x, 0, 1023, 0, 179);
  } else {
    // Arcade Mode - left joystick controls speed, right joystick controls turning
    int speedVal = map(rxdata.stick1x, 0, 1023, 0, 179);
    int turnVal = map(rxdata.stick2y, 0, 1023, 0, 179);
    leftMotorSpeed = speedVal - turnVal + 90;
    rightMotorSpeed = speedVal + turnVal - 90;
    if (leftMotorSpeed < 0){
      leftMotorSpeed = 0;
    }
    if (rightMotorSpeed > 179){
      rightMotorSpeed = 179;
    }
  }
  // Issue the commanded speed to the drive motors
  leftDriveMotor.write(leftMotorSpeed);
  rightDriveMotor.write(rightMotorSpeed);

///////  BELT FUNCTIONS: ONLY ENABLED WHEN BALL IS NOT IN SHOOTER  ////////////
//      SHOOTER FUNCTIONS: ONLY ENABLED WHEN BALL IS IN SHOOTER
  sonarDistance = sonar.ping_cm();
  if ( sonarDistance <= BALL_DISTANCE || sonarDistance >= MAX_DISTANCE ) { // if ball is where it needs to be
    txdata.ballready = true;

    //  light the green LEDs   TBD
    //  set ball seen flag for panel  TBD
    
    //  // Map the potentiometer dial (0-1023) to a valid positive shooter speed (90-179)
      shooterSpeed = map(rxdata.shooterspeed, 0, 1023, servoHaltVal, servoFullForeVal);
    // Run the shooter
    txdata.shooterspeedecho = shooterSpeed;      // aassigning shooterSpeed to echo 
    shooterMotor.write(shooterSpeed);  // ball is here, run shoother motor
    if (rxdata.shoot > 0 ) {    // shooter button pressed
      // Run the conveyor forward
      conveyorMotor.write(servoFullBackVal);
    } else {
      // Stop the conveyor
      conveyorMotor.write(servoHaltVal);
    }
    intakeMotor.write(servoHaltVal);   // in case this is first time seeing ball
    
  } else {   // no ball   //////////////
    txdata.ballready = false;

    // turn LEDs off TBD
    
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
  
  // do a slow blink to show enabled and running
  if ( lastBlinkToggle < millis()-1000 ) { //if more than a second ago
    lastBlinkToggle = millis();
    if ( bitRead( PORTB, 3) ) {   // check the output pin
      digitalWrite( LINK_STATUS_LED_11, LOW);
    } else {
      digitalWrite( LINK_STATUS_LED_11, HIGH);
    }
  }
}

void localRobotRoutine(){
//  leftDriveMotor.write(servoHaltVal);
//  rightDriveMotor.write(servoHaltVal);
//  intakeMotor.write(servoHaltVal);
//  conveyorMotor.write(servoHaltVal);
//  shooterMotor.write(servoHaltVal);

  
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
