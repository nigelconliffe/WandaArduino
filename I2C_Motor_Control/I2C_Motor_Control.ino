/*
 * Motor Control
 * 
 * This sketch drives the on-board motor-control arduino.
 * 
 */
#include <Servo.h>
#include <Wire.h>

/*
 * Keep track of which version of the motor control program is running.
 * We may send this to the master as part of a data response.
 */
#define VERSION 00

/*
 * I2C address
 */
#define SLAVE_ADDRESS 0x10

/*
 * Motor Stuff (related to specific UNO board in the robot
 */
// Turns out MOTOR 1 is RIGHT and MOTOR 2 is LEFT
#define  LEFT_MOTOR  6
#define  RIGHT_MOTOR 5

// this is the pin for int 0
#define LEFT_ENCODER  2
// this is the pin for int 1   
#define RIGHT_ENCODER 3

 
//
// Empirically the limit seems to be:
//  Full reverse = 140
//  Full forward =  50
//  Midpoint = 90 seems to work well
#define FULL_SPEED  45
#define NORMAL_SPEED 30
#define SLOW_SPEED 15
#define CREEP_SPEED 10

#define OFF 92

/*
 * Motors actually appear as servos to the Arduiino
 */
Servo leftMotor;
Servo rightMotor;

volatile unsigned long motorLeftCount;
volatile unsigned long motorRightCount;

volatile uint8_t  leftThrottle;
volatile uint8_t  rightThrottle;

/*
 * Command definitions
 * 
 * Commands consost of a command Direction and a Modifier.
 * Passed in a single byte, the format is
 * ssmmsddd 
 *   - s = spare
 *   - m = modifier
 *   - d = direction
 *   
 * Special note: 0x00  (all zeros) is *always* a stop command
 */
// COMMAND (bits ....dddd)
const uint8_t STOP = 0x00;

const uint8_t FORWARD  = 0x01;
const uint8_t REVERSE  = 0x02;

const uint8_t LEFT = 0x05;
const uint8_t RIGHT= 0x06;

/*
 * Raw mode passes the throttle settings as a param
 */
const uint8_t RAW  = 0x08;

// MODIFIERS - SPEED (bits ..mm....)
const uint8_t CREEP  = 0x00;
const uint8_t SLOW   = 0x10;
const uint8_t NORMAL = 0x20;
const uint8_t FAST   = 0x30;

// MODIFIERS - TURN (bits ..mm....) [TBD]

// MASKS
const uint8_t COMMAND = 0x0f;
const uint8_t MODIFIERS = 0x30;

/*
 * FOrward and reverse values, indexed by the speed modifier.
 * These are the actual values sent to the motors (servoes).
 * Forward speeds are in the range 40-90, and
 * reverse speeds are in the range 90-140 or so
 */
const uint8_t SPEED_FORWARD[] = {
  OFF-CREEP_SPEED,      // creep
  OFF-SLOW_SPEED,       // slow
  OFF-NORMAL_SPEED,     // normal
  OFF-FULL_SPEED        // fast
};
const uint8_t SPEED_REVERSE[] = {
  OFF+CREEP_SPEED,      // creep
  OFF+SLOW_SPEED,       // slow
  OFF+NORMAL_SPEED,     // normal
  OFF+FULL_SPEED        // fast
};

/*
 * Command processing flags
 */
bool measuring;         // running a command which exists for a distance
uint16_t  distance;     // and how far we have to go

/*
 * Timing stuff
 */
uint16_t RESPONSE_DELAY = 500;    // haqlf-second between responses
unsigned long eventTime;

//-------------------------------------------------------------------------------------------

void setup () {
  motorSetup();

  // clear all flags
  measuring = false;

  // and set the event timer 
  eventTime = millis() + RESPONSE_DELAY;
  
  // initialize digital pin RED as an output.
  // We will use this to indicate activity
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // initialize serial communication 
  Serial.begin(115200);
  Serial.println("----------------------");
  Serial.print("I2CMC Responding to address: ");
  Serial.println(SLAVE_ADDRESS, HEX);

  // Initialize I2C  
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  Wire.begin(SLAVE_ADDRESS);
}

/*
 * Main loop does several things
 *  - if we are measuring distance gone (measuring == true) then check to see 
 *    whether we have gone further than the distance requested, and if so STOP
 *   - every so often, calls writeResponse to publish an updated status message
 *  - More as things get more complicated
 */
void loop () {
  // ***   MAIN EVENT LOOP START

  // are we measuring how far we have gone?
  if ((measuring) && (motorLeftCount >= distance)) {
    // this is the same code as the case STOP action below
    measuring = false;
    setSpeeds(OFF);   
  };

  //** MAIN EVENT LOOP END
}

void commandProc(uint8_t command, uint8_t modifier, int distance) { 
  char buf[30];
  switch(command) {
    case STOP :
      measuring = false;
      setSpeeds(OFF);
      break;
      
    case FORWARD:
      motorLeftCount = 0;
      motorRightCount = 0;
      measuring = true;
      setSpeeds(SPEED_FORWARD[modifier]);
      break;

    case REVERSE:
      motorLeftCount = 0;
      motorRightCount = 0;
      measuring = true;
      setSpeeds(SPEED_REVERSE[modifier]);
      break;

//    case RAW:
//      motorLeftCount = 0;
//      motorRightCount = 0;
//      uint8_t right = cmd.param1 & 0xFF;
//      uint8_t left  = cmd.param1 >> 8;
//      setSpeeds(left, right);
//      break;

    default:
      sprintf(buf,"Unprocessed command %X", command);
  }
}

/*
 * This section of the program handles I2C communication 
 * with the host
 */
// ** DUMMY DATA FOR NOW **
byte data[] = {1,22,3,44,5, 66, 7, 98};
int ptr = 0;
const byte LENGTH = 8;
// ************************
void receiveData(int bytecount)
{
  Serial.print(bytecount); Serial.print(" ");
  for (int i = 0; i < bytecount; i++) {
    uint8_t data_to_echo = Wire.read();
    Serial.print(data_to_echo, HEX); Serial.print(" ");
  }
  Serial.println();
}

void sendData()
{
 // delay(100);
  Wire.write(data[ptr]);
  ptr++;
  if (ptr>= LENGTH) ptr = 0;
}
/*
 * This section of the program concerns ths motors, and actually
 * sending commands to the motors/handling the interrupts and 
 * other stuff.
 */

volatile byte motorLeftlast = 0;
volatile byte motorRightlast = 0;


void motorSetup() {
  // Setup motor control interfaces
  // make sure motor is OFF on startup
  leftMotor.attach(LEFT_MOTOR);
  leftMotor.write(OFF);
  rightMotor.attach(RIGHT_MOTOR);
  rightMotor.write(OFF);

// reset the counters that are sent back to the host
  motorLeftCount = 0;
  motorRightCount = 0;
  leftThrottle = OFF;
  rightThrottle = OFF;
  
  // set up to receive the motor interrupts
  pinMode(motorLeftCount, INPUT_PULLUP);
  pinMode(motorRightCount, INPUT_PULLUP);

  attachInterrupt(0, isrLeft, CHANGE);
  attachInterrupt(1, isrRight, CHANGE);
}


/*
 * Useful routine to set both motor throttles and
 * record the throttle setting for trasmission back 
 * to the ROS host
 */
void setSpeeds(const byte left_spd, const byte right_spd) {
  leftThrottle = left_spd;
  rightThrottle = right_spd;
  leftMotor.write(left_spd); 
  rightMotor.write(right_spd);
}
/*
 * Convenience function to set both motors to the same speed
 */
void setSpeeds(const byte spd) {
  setSpeeds(spd, spd);
}

/*
 * Interrupt service routine for motor 0 (int 0, pin 2)
 * isr0 is for the left motor
 */
void isrLeft()
{
  byte lstate = digitalRead(LEFT_ENCODER);
  if ( (motorLeftlast == LOW)  &&  (lstate == HIGH)) { 
      motorLeftCount++;
  }
  motorLeftlast = lstate;
}

/*
 * Interrupt service routine for motor 1 (int 1, pin 3)
 * isr1 is for the right motor
 */
void isrRight()
{
  byte lstate = digitalRead(RIGHT_ENCODER);
  if ( (motorRightlast == LOW)  &&  (lstate == HIGH)) { 
      motorRightCount++;
  }
  motorRightlast = lstate;
}
