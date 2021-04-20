/*
 * Motor Control
 * 
 * This sketch drives the on-board motor-control arduino.
 * 
 * Pin settings are based on the proto layout on the robot
 * 
 * V1.6   Debugging motor count reporting
 * V1.7   Figure out left and right for the motors
 */
#include <Servo.h>
#include <Wire.h>

/*
 * Keep track of which version of the motor control program is running.
 * We may send this to the master as part of a data response.
 */
const uint8_t MAJOR = 1;
const uint8_t MINOR = 7;

/*
 * For debugging, the speed of the serial connecion
 */
const uint32_t  SERIAL_SPEED = 115200;

/*
 * I2C address
 */
const uint8_t  DEVICE_ADDRESS = 0x10;

/*
 * Motor Stuff (related to specific UNO board in the robot
 */
const uint8_t   LEFT_MOTOR = 5;
const uint8_t   RIGHT_MOTOR= 6;

// this is the pin for int 0
const uint8_t  LEFT_ENCODER = 3;
// this is the pin for int 1   
const uint8_t  RIGHT_ENCODER = 2;

//
// Empirically the limit seems to be:
//  Full reverse = 140
//  Full forward =  50
//  Midpoint = 92 seems to work well
const uint8_t FAST_SPEED = 45;
const uint8_t NORMAL_SPEED = 30;
const uint8_t SLOW_SPEED = 15;
const uint8_t CREEP_SPEED = 10;
const uint8_t OFF = 92;

/*
 * Map throttle "speeds" to actual throttle values
 * Order is
 *   CREEP, SLOW, NORMAL, FAST
 */
const uint8_t THROTTLE_TABLE_SIZE = 4;
const uint8_t REVERSE[THROTTLE_TABLE_SIZE] = {
  OFF+CREEP_SPEED, OFF+SLOW_SPEED, 
  OFF+NORMAL_SPEED, OFF+FAST_SPEED};

const uint8_t FORWARD[THROTTLE_TABLE_SIZE] = {
  OFF-CREEP_SPEED, OFF-SLOW_SPEED, 
  OFF-NORMAL_SPEED, OFF-FAST_SPEED}; 
 
/*
 * Motors actually appear as servos to the Arduiino
 */
Servo leftMotor;
Servo rightMotor;

/*
 * This data structure holds the current throttle settings and 
 * motor encoder counts.  Organized like this so it can be passed
 * to the ROSPI as an I2C request
 */

struct MotorDataStruct{
  volatile uint32_t motorLeftCount;
  volatile uint32_t motorRightCount;

  volatile uint8_t  leftThrottle;
  volatile uint8_t  rightThrottle;

  uint8_t major = MAJOR;
  uint8_t minor = MINOR;
} motorData;

/*
 * Maybe this should be a union???
 */
char*  writeBuffer = (char*)&motorData;


//-------------------------------------------------------------------------------------------

char debugText[60];

void setup () {
  motorSetup();

  // initialize digital pin RED as an output.
  // We will use this to indicate an error
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // initialize serial communication 
  Serial.begin(SERIAL_SPEED);
  sprintf(debugText, "\n------------------\nI2C MC V%d.%d responding to address: %x",MAJOR, MINOR, 
        DEVICE_ADDRESS);
  Serial.println(debugText);

  // Initialize I2C  
  Wire.onReceive(receiveData);
  Wire.onRequest(sendMotorData);
  Wire.begin(DEVICE_ADDRESS);
}

/*
 * ********* Command Processing *********
 */
uint8_t  command = 255;       // command sent from ROSPI
uint8_t  param8;              // 8 or 16-bit parameter depending on command
uint16_t param16;
 
/*
 Main loop simply decodes commands and sets the motor power accordingly.
 The command and param8/16 values are set asynchronously by the I2C receiveData
 routine below. 
 A command of 255 is a NOP
 */
void loop () {
//  digitalWrite(LED_BUILTIN, LOW);
  delay(250);
  switch(command) {
  case 255:
      /* NOP command. Do nothing*/
      break;

  case 0:
      /*  STOP command. 
       *  Send off to both motors 
       */
      setSpeeds(OFF);
      delay(500);       // allow time for motors to stop spinning (DEBUGGING)
      printCounts();
      break;

  case 1:
     /*  RAW command. 
      *  Unpack the individual throttle settings
      *  and send to both motors
      */
      setSpeeds((param16 >> 8),(param16 & 255));
      break;

  case 2:
    /*  FORWARD command
     *   param8 contains the throttle setting
     */
     if (param8 <= THROTTLE_TABLE_SIZE)   setSpeeds(FORWARD[param8]);
     else   digitalWrite(LED_BUILTIN, HIGH);      // turn on error light
     break;

 case 3:
   /*  REVERSE command
    *   param8 contains thethrottle setting
    */
     if (param8 <= THROTTLE_TABLE_SIZE)  setSpeeds(REVERSE[param8]);
     else   digitalWrite(LED_BUILTIN, HIGH);      // turn on error light
     break;

 case 10:
   /* Zero counters command
    *  parameters ignored
    */
    // reset the counters that are sent back to the host
    motorData.motorLeftCount = 0;
    motorData.motorRightCount = 0;
    break;
    
 default:
   /*
    * Here, it's an invalid command
    * 
    * NEED BETTER ERROR HANDLING SOMEHOW
    */
     digitalWrite(LED_BUILTIN, HIGH);      // turn on error light   
//     Serial.print("Bad command "); Serial.println(command);
  }
  // now set the command byte back to NOP
  command = 255;
}

/*
 * Debug routine to print out the motor left and right counts
 */
void printCounts() {
  Serial.print("Pulse counts ["); Serial.print(motorData.motorLeftCount, HEX);
  Serial.print(","); Serial.print(motorData.motorRightCount, HEX);
  Serial.println("]");
}

/*
 * ************************************
 * I2C communication with ROSPI
 * ************************************
 */

 /*
  * Keep track ofthe reads and writes.
  * Probably won't need more than 65535 :)
  */
uint16_t  reads = 0;
uint16_t  writes = 0;

/*
 * This is part of the ReadReg implementation.rTR will hold the
 * number of teh register to be sent to the host on the next read
 * request.
 * 
 * The value 255 is a nonsense value, which indicates that no  
 * register number has been received.
 */
uint8_t  registerToRead = 255;

/*
 * Prepare to read either one or two bytes from the stream
 */
union {
  byte rbyte[2];
  int  received;
} readBuffer;
int readPtr = 0;

/*
 * This responds to the WriteReg8 and WriteReg16 methods in WiringPi.
 * Action is to read the register number (first byte) and then the next
 * one or two bytes depending on whether it's an 8 or 16 bit write from
 * the host.
 * 
 * For the motor control module, we are using the register number as the
 * command, as described above.
 */
void receiveData(int bytecount)
{
  sprintf(debugText, "R: %d: %d:", reads, bytecount);
  Serial.print(debugText);
  uint8_t  reg;
  switch(bytecount) {
  case 3:
    // 16-bit register write.  First byte is the register, next two are the data
    command = Wire.read();
    readBuffer.rbyte[0] = Wire.read();
    readBuffer.rbyte[1] = Wire.read();
    param16 = readBuffer.received;
    sprintf(debugText, "Reg [%d] %x", command, param16);
    Serial.println(debugText);
    break;
   
  case 2:
    // 8-bit register write.  First byte is the register, second is the data.
    command = Wire.read();
    param8 = Wire.read();
    sprintf(debugText, "Reg [%d] %x", command, param8);
    Serial.println(debugText);
    break;

  case 1:
    /*
     * This is a special case;  If the ROSPI issues a ReadReg call, then
     * the master will send a single byte (the register number) before issuing 
     * a read to collect the contents of that register.
     */
    reg = Wire.read();
    registerToRead = reg;
    sprintf(debugText, "Reg [%d] read request?", reg);
    Serial.println(debugText);
    break;
    
  default:
    Serial.print("Bytecount ");Serial.println(bytecount);
    for (int i = 0; i < bytecount; i++) {
      readBuffer.rbyte[readPtr] = Wire.read();
      Serial.println(readBuffer.rbyte[readPtr], HEX);
      readPtr++;
      if (readPtr >= 2) readPtr = 0;
    }
    break;
  }
  reads++;
}
/*
 * On any request, send the current motor state structure to the ROSPI
 */
int wbPtr = 0;
void sendMotorData()
{
  Serial.print(" * "); Serial.println((uint8_t) writeBuffer[wbPtr], HEX);
  delay(100);
  Wire.write(writeBuffer[wbPtr++]);
  if (wbPtr >= sizeof(motorData)) wbPtr = 0;
}

/*
 * *********************
 * Motor Control
 * *********************
 */
/*
 * This section of the program concerns ths motors, and actually
 * sending commands to the motors/handling the interrupts and 
 * other stuff.
 */

volatile byte motorLeftLast = 0;
volatile byte motorRightLast = 0;


void motorSetup() {
  // Setup motor control interfaces
  // make sure motor is OFF on startup
  leftMotor.attach(LEFT_MOTOR);
  leftMotor.write(OFF);
  rightMotor.attach(RIGHT_MOTOR);
  rightMotor.write(OFF);

// reset the counters that are sent back to the host
  motorData.motorLeftCount = 0;
  motorData.motorRightCount = 0;
  motorData.leftThrottle = OFF;
  motorData.rightThrottle = OFF;
  
  // set up to receive the motor interrupts
  pinMode(LEFT_ENCODER, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER), isrRight, CHANGE);
}


/*
 * Useful routine to set both motor throttles and
 * record the throttle setting for trasmission back 
 * to the ROS host
 */
void setSpeeds(const byte left_spd, const byte right_spd) {
  motorData.leftThrottle = left_spd;
  motorData.rightThrottle = right_spd;
  leftMotor.write(left_spd); 
  rightMotor.write(right_spd);
  sprintf(debugText, "Speeds set [%d,%d]", left_spd, right_spd);
  Serial.println(debugText);
}

/*
 * Convenience function to set both motors to the same speed
 */
void setSpeeds(const byte spd) {
  setSpeeds(spd, spd);
}

/*
 * ********************************
 * Interrupt Service Routines
 * ********************************
 */

/*
 * Interrupt service routine for motor 0 (int 0, pin 2)
 * isr0 is for the left motor
 */
void isrLeft()
{
  byte lstate = digitalRead(LEFT_ENCODER);
  if ( (motorLeftLast == LOW)  &&  (lstate == HIGH)) { 
      motorData.motorLeftCount++;
    //  digitalWrite(LED_BUILTIN, HIGH);
  }
  motorLeftLast = lstate;
}

/*
 * Interrupt service routine for motor 1 (int 1, pin 3)
 * isr1 is for the right motor
 */
void isrRight()
{
  byte lstate = digitalRead(RIGHT_ENCODER);
  if ( (motorRightLast == LOW)  &&  (lstate == HIGH)) { 
      motorData.motorRightCount++;
   //   digitalWrite(LED_BUILTIN, HIGH);
  }
  motorRightLast = lstate;
}
