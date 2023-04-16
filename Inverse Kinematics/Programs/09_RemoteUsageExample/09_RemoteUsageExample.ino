/**
 * Program to test out remote usage on hexapod
 * Created:   16.02.2023
 * Modified:  16.02.2023
 * Author:    Philipp Schulz
 */
#include <Wire.h>
// ---------------NRF24L01-----------
#include <SPI.h>
#include <RF24.h>
// ---------------NRF24L01-----------
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
Adafruit_PWMServoDriver pwmBoard = Adafruit_PWMServoDriver(0x41);   //object of the pwm driver board
Servo S1;         // Servo object 1
Servo S2;         // Servo object 2

#define FREQUENCY 50

#define SERVOMIN  697  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  2741 // This is the 'maximum' pulse length count (out of 4096)

// --------------------------used PWM pins on boards----------------------
/* Servo Nr. | Leg Nr. | Location  | Direction | Joint | Connection | Pin Nr.
       1     |    1    |   front   |    left   |  hip  | PWM Board  |    1
       2     |    1    |   front   |    left   | knee  | PWM Board  |    0
       3     |    1    |   front   |    left   | foot  |    MCU     |   PWM1/5
       4     |    2    |   front   |   right   |  hip  | PWM Board  |    15
       5     |    2    |   front   |   right   | knee  | PWM Board  |    14
       6     |    2    |   front   |   right   | foot  |    MCU     |   PWM2/6
       7     |    3    |  middle   |    left   |  hip  | PWM Board  |    4
       8     |    3    |  middle   |    left   | knee  | PWM Board  |    3
       9     |    3    |  middle   |    left   | foot  | PWM Board  |    2
      10     |    4    |  middle   |   right   |  hip  | PWM Board  |    11
      11     |    4    |  middle   |   right   | knee  | PWM Board  |    12
      12     |    4    |  middle   |   right   | foot  | PWM Board  |    13
      13     |    5    |   back    |    left   |  hip  | PWM Board  |    7
      14     |    5    |   back    |    left   | knee  | PWM Board  |    6
      15     |    5    |   back    |    left   | foot  | PWM Board  |    5
      16     |    6    |   back    |   right   |  hip  | PWM Board  |    8
      17     |    6    |   back    |   right   | knee  | PWM Board  |    9
      18     |    6    |   back    |   right   | foot  | PWM Board  |    10
// --------------------------used PWM pins on boards----------------------*/

// servo calibration values
//s:          1  2  3  4  5  6          7  8  9 10 11 12 13 14 15 16 17 18
float m[] = {-1, 1, 1, 1, 1, 0.833333 ,-1, 1, 1, 1, 1, 1,-1, 1, 1, 1, 1, 1};
float b[] = {90,90,45,90,90, 35.5,     90,90,45,90,90,45,90,90,45,90,90,45};
                     // 1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18
float servoValues[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// default translation and rotation for any leg, any additional input will be added to these values
float defaultLegTransform[] = {44,0,-69.5,0,0,0}; // x y z alpha beta gamma

// variables for homing
long startAngle = 0;
int homingDelay = 500;

//variables for movement patterns
int cycleTime = 10;

// for controlling loop times
int servoLoopTime = 100;  // ms
long lastTimestamp = 0;
// variables for IKM
float kartesian[] = {0,0,0,0,0,0};  // x y z alpha beta gamma, transform from global to robot coordinate system!
// servo angles from inverse kinematics 1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18
float kartesianAngles[] =             { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// x coordinates for inverse kinematics 1  2  3  4  5  6
float kartesianX[] =                  { 0, 0, 0, 0, 0, 0};
// y coordinates for inverse kinematics 1  2  3  4  5  6
float kartesianY[] =                  { 0, 0, 0, 0, 0, 0};
// z coordinates for inverse kinematics 1  2  3  4  5  6
float kartesianZ[] =                  { 0, 0, 0, 0, 0, 0};
// ---------------NRF24L01-----------
// variables for all IO
byte leds[] = {0,0,0,0,0}; // led1, led2, led3, led4, led5
byte switches[] = {0,0,0,0,0};   // sw1, sw2, sw3, sw4, sw5
byte buttons[] = {0,0,0,0,0,0};  // L1, L2, StickL, R1, R2, StickR
int axes[] = {0,0,0,0,0,0};      // Lx, Ly, Lz, Rx, Ry, Rz

// structure for data to send to client device
struct SEND_DATA_STRUCTURE_OPTIMIZED{
    byte buttons;
    byte switches;
    int16_t rx_left;
    int16_t ry_left;
    int16_t rz_left;
    int16_t rx_right;
    int16_t ry_right;
    int16_t rz_right;
};
// structure for data to receive from client device
struct RECEIVE_DATA_STRUCTURE_OPTIMIZED{
  byte leds;
};
// create objects of structures
SEND_DATA_STRUCTURE_OPTIMIZED dataToSend;
RECEIVE_DATA_STRUCTURE_OPTIMIZED dataToRead;
// initialize the NRF24L01 chip
RF24 radio(2, 16); // CE, CSN
// addresses for sender and receiver
const byte addresses[][6] = {"01", "02"};
// ---------------NRF24L01-----------
//converts given angle to a number in 4096 for the PWM board
int pulseWidth(int angle){          
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}

// https://stackoverflow.com/questions/9072320/split-string-into-string-array
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

// ---------------NRF24L01-----------
// method for setting the data from global variables to be sent
void setPayloadContent() {
  byte ledsByte = 0;
  for(int i=0;i<6;i++) {
    if(leds[i]) {
      ledsByte |= (1 << (7-i));
    }
  }
  dataToRead.leds = ledsByte;
}

// method for getting the received data to global variables
void getPayloadContent() {
  for(int i=0;i<6;i++) {
    buttons[i] = bitRead(dataToSend.buttons, (7-i));
  }
  for(int i=0;i<5;i++) {
    switches[i] = bitRead(dataToSend.switches, (7-i));
  }
  axes[0] = dataToSend.rx_left;
  axes[1] = dataToSend.ry_left;
  axes[2] = dataToSend.rz_left;
  axes[3] = dataToSend.rx_right;
  axes[4] = dataToSend.ry_right;
  axes[5] = dataToSend.rz_right;
}

// method for handling all radio related code
void handleRadio() {
  uint8_t pipe;
  // check if radio has a new payload
  if (radio.available(&pipe)) { 
    // get dynamic payload size
    uint8_t bytes = radio.getDynamicPayloadSize();
    // read data
    radio.read(&dataToSend, sizeof(dataToSend));
    // handle payload
    getPayloadContent();
    // prepare return payload
    setPayloadContent();
    // send ack ACK payload
    radio.writeAckPayload(1,&dataToRead,sizeof(dataToRead));
  }
}
// ---------------NRF24L01-----------

// method to utilize servo calibration values
float servoCalibration(int servoNumber, float targetPosition) {
  return m[servoNumber-1]*targetPosition + b[servoNumber-1];  // return value based on calibration
}


// method to set a servo by its number to a new targetPosition
void setServoByNumber(int servoNumber, float targetPosition) {
  switch(servoNumber) {                                   // go over all possible servo numbers
    case 1:
      pwmBoard.setPWM(1,0,pulseWidth(servoCalibration(servoNumber,targetPosition)));    // move servo 1 in position
      break;
    case 2:
      pwmBoard.setPWM(0,0,pulseWidth(servoCalibration(servoNumber,targetPosition)));    // move servo 2 in position
      break;
    case 3:
      S1.write(servoCalibration(servoNumber,targetPosition));                           // move servo 3 in position
      break;
    case 4:
      pwmBoard.setPWM(15,0,pulseWidth(servoCalibration(servoNumber,targetPosition)));   // move servo 4 in position
      break;
    case 5:
      pwmBoard.setPWM(14,0,pulseWidth(servoCalibration(servoNumber,targetPosition)));   // move servo 5 in position
      break;
    case 6:
      S2.write(servoCalibration(servoNumber,targetPosition));                           // move servo 6 in position
      break;
    case 7:
      pwmBoard.setPWM(4,0,pulseWidth(servoCalibration(servoNumber,targetPosition)));    // move servo 7 in position
      break;
    case 8:
      pwmBoard.setPWM(3,0,pulseWidth(servoCalibration(servoNumber,targetPosition)));    // move servo 8 in position
      break;
    case 9:
      pwmBoard.setPWM(2,0,pulseWidth(servoCalibration(servoNumber,targetPosition)));    // move servo 9 in position
      break;
    case 10:
      pwmBoard.setPWM(11,0,pulseWidth(servoCalibration(servoNumber,targetPosition)));   // move servo 10 in position
      break;
    case 11:
      pwmBoard.setPWM(12,0,pulseWidth(servoCalibration(servoNumber,targetPosition)));   // move servo 11 in position
      break;
    case 12:
      pwmBoard.setPWM(13,0,pulseWidth(servoCalibration(servoNumber,targetPosition)));   // move servo 12 in position
      break;
    case 13:
      pwmBoard.setPWM(7,0,pulseWidth(servoCalibration(servoNumber,targetPosition)));    // move servo 13 in position
      break;
    case 14:
      pwmBoard.setPWM(6,0,pulseWidth(servoCalibration(servoNumber,targetPosition)));    // move servo 14 in position
      break;
    case 15:
      pwmBoard.setPWM(5,0,pulseWidth(servoCalibration(servoNumber,targetPosition)));    // move servo 15 in position
      break;
    case 16:
      pwmBoard.setPWM(8,0,pulseWidth(servoCalibration(servoNumber,targetPosition)));    // move servo 16 in position
      break;
    case 17:
      pwmBoard.setPWM(9,0,pulseWidth(servoCalibration(servoNumber,targetPosition)));    // move servo 17 in position
      break;
    case 18:
      pwmBoard.setPWM(10,0,pulseWidth(servoCalibration(servoNumber,targetPosition)));   // move servo 18 in position
      break;
  }
  // save current position for movement commands
  servoValues[servoNumber-1] = targetPosition;
}

// method to determine direction of offset for movements
int determineIncrement(int current,int target,int incrementSize) {
  // check if the delta is positive or negative, return the given increment in the fitting direction
  if((current-target) > 0 ) { // if the target is smaller than the current position
    return -incrementSize;
  } else {                    // if the target is larger than the current position
    return incrementSize;
  }
}

//  method to move all servos so that the hexapod stands up
void standUp() {
  // 1. all hips 0°
  // 2. all knees 30°
  // 3. all feet 90°
  // 4. all knees 0°

  // placeholder condition for overall step conditions
  boolean condition = false;

  // phase 1: loop until all hips are in position
  int target = 0;
  while(!condition) {
    // check hip 1 and adjust the saved values for the required movement
    if(servoValues[0]!=target) {
      servoValues[0] = servoValues[0]+determineIncrement(servoValues[0],target,1);
      setServoByNumber(1,servoValues[0]);
    }
    // check hip 2 and adjust the saved values for the required movement
    if(servoValues[3]!=target) {
      servoValues[3] = servoValues[3]+determineIncrement(servoValues[3],target,1);
      setServoByNumber(4,servoValues[3]);
    }
    // check hip 3 and adjust the saved values for the required movement
    if(servoValues[6]!=target) {
      servoValues[6] = servoValues[6]+determineIncrement(servoValues[6],target,1);
      setServoByNumber(7,servoValues[6]);
    }
    // check hip 4 and adjust the saved values for the required movement
    if(servoValues[9]!=target) {
      servoValues[9] = servoValues[9]+determineIncrement(servoValues[9],target,1);
      setServoByNumber(10,servoValues[9]);
    }
    // check hip 5 and adjust the saved values for the required movement
    if(servoValues[12]!=target) {
      servoValues[12] = servoValues[12]+determineIncrement(servoValues[12],target,1);
      setServoByNumber(13,servoValues[12]);
    }
    // check hip 6 and adjust the saved values for the required movement
    if(servoValues[15]!=target) {
      servoValues[15] = servoValues[15]+determineIncrement(servoValues[15],target,1);
      setServoByNumber(16,servoValues[15]);
    }
    // check if the condition is met
    if(servoValues[0]==target && servoValues[3]==target && servoValues[6]==target && servoValues[9]==target && servoValues[12]==target && servoValues[15]==target) {
      condition = true;
    }
    // wait for loop cycle time
    delay(cycleTime);
  }
  // phase 2: loop until all knees are in position
  target = 30;
  condition = false;
  while(!condition) {
    // check knee 1 and adjust the saved values for the required movement
    if(servoValues[1]!=target) {
      servoValues[1] = servoValues[1]+determineIncrement(servoValues[1],target,1);
      setServoByNumber(2,servoValues[1]);
    }
    // check knee 2 and adjust the saved values for the required movement
    if(servoValues[4]!=target) {
      servoValues[4] = servoValues[4]+determineIncrement(servoValues[4],target,1);
      setServoByNumber(5,servoValues[4]);
    }
    // check knee 3 and adjust the saved values for the required movement
    if(servoValues[7]!=target) {
      servoValues[7] = servoValues[7]+determineIncrement(servoValues[7],target,1);
      setServoByNumber(8,servoValues[7]);
    }
    // check knee 4 and adjust the saved values for the required movement
    if(servoValues[10]!=target) {
      servoValues[10] = servoValues[10]+determineIncrement(servoValues[10],target,1);
      setServoByNumber(11,servoValues[10]);
    }
    // check knee 5 and adjust the saved values for the required movement
    if(servoValues[13]!=target) {
      servoValues[13] = servoValues[13]+determineIncrement(servoValues[13],target,1);
      setServoByNumber(14,servoValues[13]);
    }
    // check knee 6 and adjust the saved values for the required movement
    if(servoValues[16]!=target) {
      servoValues[16] = servoValues[16]+determineIncrement(servoValues[16],target,1);
      setServoByNumber(17,servoValues[16]);
    }
    // check if the condition is met
    if(servoValues[1]==target && servoValues[4]==target && servoValues[7]==target && servoValues[10]==target && servoValues[13]==target && servoValues[16]==target) {
      condition = true;
    }
    // wait for loop cycle time
    delay(cycleTime);
  }
  // phase 3: loop until all feet are in position
  target = 90;
  condition = false;
  while(!condition) {
    // check foot 1 and adjust the saved values for the required movement
    if(servoValues[2]!=target) {
      servoValues[2] = servoValues[2]+determineIncrement(servoValues[2],target,1);
      setServoByNumber(3,servoValues[2]);
    }
    // check foot 2 and adjust the saved values for the required movement
    if(servoValues[5]!=target) {
      servoValues[5] = servoValues[5]+determineIncrement(servoValues[5],target,1);
      setServoByNumber(6,servoValues[5]);
    }
    // check foot 3 and adjust the saved values for the required movement
    if(servoValues[8]!=target) {
      servoValues[8] = servoValues[8]+determineIncrement(servoValues[8],target,1);
      setServoByNumber(9,servoValues[8]);
    }
    // check foot 4 and adjust the saved values for the required movement
    if(servoValues[11]!=target) {
      servoValues[11] = servoValues[11]+determineIncrement(servoValues[11],target,1);
      setServoByNumber(12,servoValues[11]);
    }
    // check foot 5 and adjust the saved values for the required movement
    if(servoValues[14]!=target) {
      servoValues[14] = servoValues[14]+determineIncrement(servoValues[14],target,1);
      setServoByNumber(15,servoValues[14]);
    }
    // check foot 6 and adjust the saved values for the required movement
    if(servoValues[17]!=target) {
      servoValues[17] = servoValues[17]+determineIncrement(servoValues[17],target,1);
      setServoByNumber(18,servoValues[17]);
    }
    // check if the condition is met
    if(servoValues[2]==target && servoValues[5]==target && servoValues[8]==target && servoValues[11]==target && servoValues[14]==target && servoValues[17]==target) {
      condition = true;
    }
    // wait for loop cycle time
    delay(cycleTime);
  }
  // phase 4: loop until all knees are in position
  target = 0;
  condition = false;
  while(!condition) {
    // check knee 1 and adjust the saved values for the required movement
    if(servoValues[1]!=target) {
      servoValues[1] = servoValues[1]+determineIncrement(servoValues[1],target,1);
      setServoByNumber(2,servoValues[1]);
    }
    // check knee 2 and adjust the saved values for the required movement
    if(servoValues[4]!=target) {
      servoValues[4] = servoValues[4]+determineIncrement(servoValues[4],target,1);
      setServoByNumber(5,servoValues[4]);
    }
    // check knee 3 and adjust the saved values for the required movement
    if(servoValues[7]!=target) {
      servoValues[7] = servoValues[7]+determineIncrement(servoValues[7],target,1);
      setServoByNumber(8,servoValues[7]);
    }
    // check knee 4 and adjust the saved values for the required movement
    if(servoValues[10]!=target) {
      servoValues[10] = servoValues[10]+determineIncrement(servoValues[10],target,1);
      setServoByNumber(11,servoValues[10]);
    }
    // check knee 5 and adjust the saved values for the required movement
    if(servoValues[13]!=target) {
      servoValues[13] = servoValues[13]+determineIncrement(servoValues[13],target,1);
      setServoByNumber(14,servoValues[13]);
    }
    // check knee 6 and adjust the saved values for the required movement
    if(servoValues[16]!=target) {
      servoValues[16] = servoValues[16]+determineIncrement(servoValues[16],target,1);
      setServoByNumber(17,servoValues[16]);
    }
    // check if the condition is met
    if(servoValues[1]==target && servoValues[4]==target && servoValues[7]==target && servoValues[10]==target && servoValues[13]==target && servoValues[16]==target) {
      condition = true;
    }
    // wait for loop cycle time
    delay(cycleTime);
  }
}

// method to move all servos so that the hexapod sits down
void sitDown() {
  // 1. all hips 0°
  // 2. all knees 30°
  // 3. all feet 0°
  // 4. all knees 0°
  
  // placeholder condition for overall step conditions
  boolean condition = false;
  
  // phase 1: loop until all hips are in position
  int target = 0;
  while(!condition) {
    // check hip 1 and adjust the saved values for the required movement
    if(servoValues[0]!=target) {
      servoValues[0] = servoValues[0]+determineIncrement(servoValues[0],target,1);
      setServoByNumber(1,servoValues[0]);
    }
    // check hip 2 and adjust the saved values for the required movement
    if(servoValues[3]!=target) {
      servoValues[3] = servoValues[3]+determineIncrement(servoValues[3],target,1);
      setServoByNumber(4,servoValues[3]);
    }
    // check hip 3 and adjust the saved values for the required movement
    if(servoValues[6]!=target) {
      servoValues[6] = servoValues[6]+determineIncrement(servoValues[6],target,1);
      setServoByNumber(7,servoValues[6]);
    }
    // check hip 4 and adjust the saved values for the required movement
    if(servoValues[9]!=target) {
      servoValues[9] = servoValues[9]+determineIncrement(servoValues[9],target,1);
      setServoByNumber(10,servoValues[9]);
    }
    // check hip 5 and adjust the saved values for the required movement
    if(servoValues[12]!=target) {
      servoValues[12] = servoValues[12]+determineIncrement(servoValues[12],target,1);
      setServoByNumber(13,servoValues[12]);
    }
    // check hip 6 and adjust the saved values for the required movement
    if(servoValues[15]!=target) {
      servoValues[15] = servoValues[15]+determineIncrement(servoValues[15],target,1);
      setServoByNumber(16,servoValues[15]);
    }
    // check if the condition is met
    if(servoValues[0]==target && servoValues[3]==target && servoValues[6]==target && servoValues[9]==target && servoValues[12]==target && servoValues[15]==target) {
      condition = true;
    }
    // wait for loop cycle time
    delay(cycleTime);
  }
  // phase 2: loop until all knees are in position
  target = 30;
  condition = false;
  while(!condition) {
    // check knee 1 and adjust the saved values for the required movement
    if(servoValues[1]!=target) {
      servoValues[1] = servoValues[1]+determineIncrement(servoValues[1],target,1);
      setServoByNumber(2,servoValues[1]);
    }
    // check knee 2 and adjust the saved values for the required movement
    if(servoValues[4]!=target) {
      servoValues[4] = servoValues[4]+determineIncrement(servoValues[4],target,1);
      setServoByNumber(5,servoValues[4]);
    }
    // check knee 3 and adjust the saved values for the required movement
    if(servoValues[7]!=target) {
      servoValues[7] = servoValues[7]+determineIncrement(servoValues[7],target,1);
      setServoByNumber(8,servoValues[7]);
    }
    // check knee 4 and adjust the saved values for the required movement
    if(servoValues[10]!=target) {
      servoValues[10] = servoValues[10]+determineIncrement(servoValues[10],target,1);
      setServoByNumber(11,servoValues[10]);
    }
    // check knee 5 and adjust the saved values for the required movement
    if(servoValues[13]!=target) {
      servoValues[13] = servoValues[13]+determineIncrement(servoValues[13],target,1);
      setServoByNumber(14,servoValues[13]);
    }
    // check knee 6 and adjust the saved values for the required movement
    if(servoValues[16]!=target) {
      servoValues[16] = servoValues[16]+determineIncrement(servoValues[16],target,1);
      setServoByNumber(17,servoValues[16]);
    }
    // check if the condition is met
    if(servoValues[1]==target && servoValues[4]==target && servoValues[7]==target && servoValues[10]==target && servoValues[13]==target && servoValues[16]==target) {
      condition = true;
    }
    // wait for loop cycle time
    delay(cycleTime);
  }
  // phase 3: loop until all feet are in position
  target = 0;
  condition = false;
  while(!condition) {
    // check foot 1 and adjust the saved values for the required movement
    if(servoValues[2]!=target) {
      servoValues[2] = servoValues[2]+determineIncrement(servoValues[2],target,1);
      setServoByNumber(3,servoValues[2]);
    }
    // check foot 2 and adjust the saved values for the required movement
    if(servoValues[5]!=target) {
      servoValues[5] = servoValues[5]+determineIncrement(servoValues[5],target,1);
      setServoByNumber(6,servoValues[5]);
    }
    // check foot 3 and adjust the saved values for the required movement
    if(servoValues[8]!=target) {
      servoValues[8] = servoValues[8]+determineIncrement(servoValues[8],target,1);
      setServoByNumber(9,servoValues[8]);
    }
    // check foot 4 and adjust the saved values for the required movement
    if(servoValues[11]!=target) {
      servoValues[11] = servoValues[11]+determineIncrement(servoValues[11],target,1);
      setServoByNumber(12,servoValues[11]);
    }
    // check foot 5 and adjust the saved values for the required movement
    if(servoValues[14]!=target) {
      servoValues[14] = servoValues[14]+determineIncrement(servoValues[14],target,1);
      setServoByNumber(15,servoValues[14]);
    }
    // check foot 6 and adjust the saved values for the required movement
    if(servoValues[17]!=target) {
      servoValues[17] = servoValues[17]+determineIncrement(servoValues[17],target,1);
      setServoByNumber(18,servoValues[17]);
    }
    // check if the condition is met
    if(servoValues[2]==target && servoValues[5]==target && servoValues[8]==target && servoValues[11]==target && servoValues[14]==target && servoValues[17]==target) {
      condition = true;
    }
    // wait for loop cycle time
    delay(cycleTime);
  }
  // phase 4: loop until all knees are in position
  target = 0;
  condition = false;
  while(!condition) {
    // check knee 1 and adjust the saved values for the required movement
    if(servoValues[1]!=target) {
      servoValues[1] = servoValues[1]+determineIncrement(servoValues[1],target,1);
      setServoByNumber(2,servoValues[1]);
    }
    // check knee 2 and adjust the saved values for the required movement
    if(servoValues[4]!=target) {
      servoValues[4] = servoValues[4]+determineIncrement(servoValues[4],target,1);
      setServoByNumber(5,servoValues[4]);
    }
    // check knee 3 and adjust the saved values for the required movement
    if(servoValues[7]!=target) {
      servoValues[7] = servoValues[7]+determineIncrement(servoValues[7],target,1);
      setServoByNumber(8,servoValues[7]);
    }
    // check knee 4 and adjust the saved values for the required movement
    if(servoValues[10]!=target) {
      servoValues[10] = servoValues[10]+determineIncrement(servoValues[10],target,1);
      setServoByNumber(11,servoValues[10]);
    }
    // check knee 5 and adjust the saved values for the required movement
    if(servoValues[13]!=target) {
      servoValues[13] = servoValues[13]+determineIncrement(servoValues[13],target,1);
      setServoByNumber(14,servoValues[13]);
    }
    // check knee 6 and adjust the saved values for the required movement
    if(servoValues[16]!=target) {
      servoValues[16] = servoValues[16]+determineIncrement(servoValues[16],target,1);
      setServoByNumber(17,servoValues[16]);
    }
    // check if the condition is met
    if(servoValues[1]==target && servoValues[4]==target && servoValues[7]==target && servoValues[10]==target && servoValues[13]==target && servoValues[16]==target) {
      condition = true;
    }
    // wait for loop cycle time
    delay(cycleTime);
  }
}

// method for implementing the signum function
float sgn(float x) {
  if(x == 0) {
    return 1;
  } else {
    return x/abs(x);
  }
}

// method to calculate all required angles for all servos from kartesian koordinates
void inverseKinematics() {
  // get requested coordinates and angles first
  float x = defaultLegTransform[0]+kartesian[0];
  float y = defaultLegTransform[1]+kartesian[1];
  float z = defaultLegTransform[2]+kartesian[2];
  float a = defaultLegTransform[3]+kartesian[3];
  float b = defaultLegTransform[4]+kartesian[4];
  float g = defaultLegTransform[5]+kartesian[5];
  // set supposed kartesian coordinates for all legs
  // -------------x-------------
  kartesianX[0] = -sgn(x)*sqrt(pow(x,2)+pow(y,2))*sin(135.0*(PI/180.0));
  kartesianX[1] = sgn(x)*sqrt(pow(x,2)+pow(y,2))*sin(45.0*(PI/180.0));
  kartesianX[2] = -x;
  kartesianX[3] = x;
  kartesianX[4] = sgn(x)*sqrt(pow(x,2)+pow(y,2))*sin(-135.0*(PI/180.0));
  kartesianX[5] = -sgn(x)*sqrt(pow(x,2)+pow(y,2))*sin(-45.0*(PI/180.0));
  // -------------y-------------
  kartesianY[0] = -sgn(y)*sqrt(pow(x,2)+pow(y,2))*cos(135.0*(PI/180.0));
  kartesianY[1] = sgn(y)*sqrt(pow(x,2)+pow(y,2))*cos(45.0*(PI/180.0));
  kartesianY[2] = y;
  kartesianY[3] = y;
  kartesianY[4] = sgn(y)*sqrt(pow(x,2)+pow(y,2))*cos(-135.0*(PI/180.0));
  kartesianY[5] = -sgn(y)*sqrt(pow(x,2)+pow(y,2))*cos(-45.0*(PI/180.0));
  // -------------z-------------
  kartesianZ[0] = z;
  kartesianZ[1] = z;
  kartesianZ[2] = z;
  kartesianZ[3] = z;
  kartesianZ[4] = z;
  kartesianZ[5] = z;
  
  // -------------------1. yaw-----------------------
  // front servos
  float dy = 76.15*(1-cos(a*(PI/180.0)));
  float dz = 76.15*sin(a*(PI/180.0));
  kartesianY[0] -= dy;
  kartesianZ[0] -= dz;
  kartesianY[1] -= dy;
  kartesianZ[1] -= dz;
  // back servos
  dy = -76.15*(1-cos(a*(PI/180.0)));
  dz = -76.15*sin(a*(PI/180.0));
  kartesianY[4] -= dy;
  kartesianZ[4] -= dz;
  kartesianY[5] -= dy;
  kartesianZ[5] -= dz;
  
  // -------------------2. pitch---------------------
  // front servos
  float dx = -60*(1-cos(b*(PI/180.0)));
  dz = -60*sin(b*(PI/180.0));
  kartesianX[0] -= dx;
  kartesianZ[0] += dz;
  kartesianX[1] += dx;
  kartesianZ[1] -= dz;
  
  // middle servos
  dx = 55.89*(1-cos(b*(PI/180.0)));
  dz = 55.89*sin(b*(PI/180.0));
  kartesianX[2] += dx;
  kartesianZ[2] -= dz;
  kartesianX[3] -= dx;
  kartesianZ[3] += dz;
  
  // back servos
  dx = 60*(1-cos(b*(PI/180.0)));
  dz = 60*sin(b*(PI/180.0));
  kartesianX[4] += dx;
  kartesianZ[4] -= dz;
  kartesianX[5] -= dx;
  kartesianZ[5] += dz;
  
  // -------------------3. roll----------------------
  // front servos
  dx = -94.46*sin((34.089+g)*(PI/180.0)) + 94.46*sin((34.089)*(PI/180.0));
  dy = -94.46*sin((34.089+g)*(PI/180.0)) + 94.46*sin((34.089)*(PI/180.0));
  //dx = -94.46*sin(g*(PI/180.0));
  //dy = -94.46*sin(g*(PI/180.0));
  kartesianX[0] += dx;
  kartesianY[0] += dy;
  kartesianX[1] += dx;
  kartesianY[1] -= dy;
  // middle servos
  dx = 55.89*sin(g*(PI/180.0));
  dy = -55.89*sin(g*(PI/180.0));
  kartesianX[2] += dx;
  kartesianY[2] += dy;
  kartesianX[3] -= dx;
  kartesianY[3] -= dy;
  // back servos
  dx = 94.46*sin((34.089+g)*(PI/180.0)) - 94.46*sin((34.089)*(PI/180.0));
  dy = 94.46*sin((34.089+g)*(PI/180.0)) - 94.46*sin((34.089)*(PI/180.0));
  //dx = 94.46*sin(g*(PI/180.0));
  //dy = 94.46*sin(g*(PI/180.0));
  kartesianX[4] += dx;
  kartesianY[4] -= dy;
  kartesianX[5] += dx;
  kartesianY[5] += dy;
  
  // -------------------4. leg 1 (front left)--------
  // calculate angles
  float x1 = sqrt(pow(kartesianX[0]-35.74-20.15,2)+pow(kartesianY[0]-56-20.15,2))-32.25;
  float a1 = asin((kartesianY[0]-56-20.15)/(x1+32.25));
  float x2 = sqrt(pow(x1,2)+pow(kartesianZ[0],2));
  float a2 = acos((pow(44,2)+pow(69.5,2)-pow(x2,2))/(2*44*69.5));
  float b1 = acos((pow(44,2)+pow(x2,2)-pow(69.5,2))/(2*44*x2));
  float g1 = asin(kartesianZ[0]/x2);
  float b2 = b1-g1;
  // save angles in array
  kartesianAngles[0] = a1*(180.0/PI);
  kartesianAngles[1] = 90-b2*(180.0/PI);
  kartesianAngles[2] = a2*(180.0/PI);
  // -------------------5. leg 2 (front right)-------
  x1 = sqrt(pow(-kartesianX[1]-35.74-20.15,2)+pow(kartesianY[1]-56-20.15,2))-32.25;
  a1 = asin((kartesianY[1]-56-20.15)/(x1+32.25));
  x2 = sqrt(pow(x1,2)+pow(kartesianZ[1],2));
  a2 = acos((pow(44,2)+pow(69.5,2)-pow(x2,2))/(2*44*69.5));
  b1 = acos((pow(44,2)+pow(x2,2)-pow(69.5,2))/(2*44*x2));
  g1 = asin(kartesianZ[1]/x2);
  b2 = b1-g1;
  // save angles in array
  kartesianAngles[3] = a1*(180.0/PI);
  kartesianAngles[4] = 90-b2*(180.0/PI);
  kartesianAngles[5] = a2*(180.0/PI);
  // -------------------6. leg 3 (middle left)-------
  x1 = sqrt(pow(kartesianX[2]-37.5-28.5,2)+pow(kartesianY[2],2))-32.25;
  a1 = asin(kartesianY[2]/(x1+32.25));
  x2 = sqrt(pow(x1,2)+pow(kartesianZ[2],2));
  a2 = acos((pow(44,2)+pow(69.5,2)-pow(x2,2))/(2*44*69.5));
  b1 = acos((pow(44,2)+pow(x2,2)-pow(69.5,2))/(2*44*x2));
  g1 = asin(kartesianZ[2]/x2);
  b2 = b1-g1;
  // save angles in array
  kartesianAngles[6] = a1*(180.0/PI);
  kartesianAngles[7] = 90-b2*(180.0/PI);
  kartesianAngles[8] = a2*(180.0/PI);
  // -------------------7. leg 4 (middle right)------
  x1 = sqrt(pow(-kartesianX[3]-37.5-28.5,2)+pow(kartesianY[3],2))-32.25;
  a1 = asin(kartesianY[3]/(x1+32.25));
  x2 = sqrt(pow(x1,2)+pow(kartesianZ[3],2));
  a2 = acos((pow(44,2)+pow(69.5,2)-pow(x2,2))/(2*44*69.5));
  b1 = acos((pow(44,2)+pow(x2,2)-pow(69.5,2))/(2*44*x2));
  g1 = asin(kartesianZ[3]/x2);
  b2 = b1-g1;
  // save angles in array
  kartesianAngles[9] = a1*(180.0/PI);
  kartesianAngles[10] = 90-b2*(180.0/PI);
  kartesianAngles[11] = a2*(180.0/PI);
  // -------------------8. leg 5 (back left)---------
  x1 = sqrt(pow(kartesianX[4]-35.74-20.15,2)+pow(-kartesianY[4]-56-20.15,2))-32.25;
  a1 = asin((-kartesianY[4]-56-20.15)/(x1+32.25));
  x2 = sqrt(pow(x1,2)+pow(kartesianZ[4],2));
  a2 = acos((pow(44,2)+pow(69.5,2)-pow(x2,2))/(2*44*69.5));
  b1 = acos((pow(44,2)+pow(x2,2)-pow(69.5,2))/(2*44*x2));
  g1 = asin(kartesianZ[4]/x2);
  b2 = b1-g1;
  // save angles in array
  kartesianAngles[12] = -a1*(180.0/PI);
  kartesianAngles[13] = 90-b2*(180.0/PI);
  kartesianAngles[14] = a2*(180.0/PI);
  // -------------------9. leg 6 (back right)--------
  x1 = sqrt(pow(-kartesianX[5]-35.74-20.15,2)+pow(-kartesianY[5]-56-20.15,2))-32.25;
  a1 = asin((-kartesianY[5]-56-20.15)/(x1+32.25));
  x2 = sqrt(pow(x1,2)+pow(kartesianZ[5],2));
  a2 = acos((pow(44,2)+pow(69.5,2)-pow(x2,2))/(2*44*69.5));
  b1 = acos((pow(44,2)+pow(x2,2)-pow(69.5,2))/(2*44*x2));
  g1 = asin(kartesianZ[5]/x2);
  b2 = b1-g1;
  // save angles in array
  kartesianAngles[15] = -a1*(180.0/PI);
  kartesianAngles[16] = 90-b2*(180.0/PI);
  kartesianAngles[17] = a2*(180.0/PI);
}

// method for setting all servos according to IKM angles
void setIKMServos() {
  // currently for debugging, real code comes later
  Serial.println("X: "+String(kartesianX[0])+" "+String(kartesianX[1])+" "+String(kartesianX[2])+" "+String(kartesianX[3])+" "+String(kartesianX[4])+" "+String(kartesianX[5]));
  Serial.println("Y: "+String(kartesianY[0])+" "+String(kartesianY[1])+" "+String(kartesianY[2])+" "+String(kartesianY[3])+" "+String(kartesianY[4])+" "+String(kartesianY[5]));
  Serial.println("Z: "+String(kartesianZ[0])+" "+String(kartesianZ[1])+" "+String(kartesianZ[2])+" "+String(kartesianZ[3])+" "+String(kartesianZ[4])+" "+String(kartesianZ[5]));
  // print servo angles
  for(int i=0;i<6;i++) {
    Serial.println("leg "+String(i+1)+" angles: "+String(kartesianAngles[(i*3)])+" "+String(kartesianAngles[(i*3)+1])+" "+String(kartesianAngles[(i*3)+2]));
  }
  
  // set servo angles
  for(int i=1;i<19;i++) {
    setServoByNumber(i,kartesianAngles[i-1]);
  }
  
}

// method to home all servos based on the initial target values
void homeServos() {
  setServoByNumber(2,startAngle);                            // home knee joint front left
  setServoByNumber(3,startAngle);                            // home foot joint front left
  delay(homingDelay);
  setServoByNumber(5,startAngle);                            // home knee joint front right
  setServoByNumber(6,startAngle);                            // home foot joint front right
  delay(homingDelay);
  setServoByNumber(8,startAngle);                            // home knee joint middle left
  setServoByNumber(9,startAngle);                            // home foot joint middle left
  delay(homingDelay);
  setServoByNumber(11,startAngle);                           // home knee joint middle right
  setServoByNumber(12,startAngle);                           // home foot joint middle right
  delay(homingDelay);
  setServoByNumber(14,startAngle);                           // home knee joint back left
  setServoByNumber(15,startAngle);                           // home foot joint back left
  delay(homingDelay);
  setServoByNumber(17,startAngle);                           // home knee joint back right
  setServoByNumber(18,startAngle);                           // home foot joint back right
  delay(homingDelay);
  setServoByNumber(1,startAngle);                            // home hip joint front left
  delay(homingDelay);
  setServoByNumber(4,startAngle);                            // home hip joint front right
  delay(homingDelay);
  setServoByNumber(7,startAngle);                            // home hip joint middle left
  delay(homingDelay);
  setServoByNumber(10,startAngle);                           // home hip joint middle right
  delay(homingDelay);
  setServoByNumber(13,startAngle);                           // home hip joint back left
  delay(homingDelay);
  setServoByNumber(16,startAngle);                           // home hip joint back right
  delay(homingDelay);
}

// method for controlling overall behavior
void processVariables() {
  // set targets for first leg
  setServoByNumber(1,map(axes[0],-1024,1024,-90,90));
  setServoByNumber(2,map(axes[1],-1024,1024,-90,90));
  if(axes[2]<=0) {
    setServoByNumber(3,map(axes[2],-1024,0,-45,0));
  } else {
    setServoByNumber(3,map(axes[2],0,1024,0,135));
  }
  for(int i=0;i<5;i++) {
    leds[i] = switches[i];
  }
  // check if second leg should be used
  if(switches[0]) {
    setServoByNumber(4,map(axes[3],-1024,1024,-90,90));
    setServoByNumber(5,map(axes[4],-1024,1024,-90,90));
    if(axes[5]<=0) {
      setServoByNumber(6,map(axes[5],-1024,0,-45,0));
    } else {
      setServoByNumber(6,map(axes[5],0,1024,0,135));
    }
  }
  // check if third leg should be used
  if(switches[1]) {
    setServoByNumber(7,map(axes[3],-1024,1024,-90,90));
    setServoByNumber(8,map(axes[4],-1024,1024,-90,90));
    if(axes[5]<=0) {
      setServoByNumber(9,map(axes[5],-1024,0,-45,0));
    } else {
      setServoByNumber(9,map(axes[5],0,1024,0,135));
    }
  }
  // check if fourth leg should be used
  if(switches[2]) {
    setServoByNumber(10,map(axes[0],-1024,1024,-90,90));
    setServoByNumber(11,map(axes[1],-1024,1024,-90,90));
    if(axes[2]<=0) {
      setServoByNumber(12,map(axes[2],-1024,0,-45,0));
    } else {
      setServoByNumber(12,map(axes[2],0,1024,0,135));
    }
  }
  // check if fifth leg should be used
  if(switches[3]) {
    setServoByNumber(13,map(axes[0],-1024,1024,-90,90));
    setServoByNumber(14,map(axes[1],-1024,1024,-90,90));
    if(axes[2]<=0) {
      setServoByNumber(15,map(axes[2],-1024,0,-45,0));
    } else {
      setServoByNumber(15,map(axes[2],0,1024,0,135));
    }
  }
  // check if sixth leg should be used
  if(switches[4]) {
    setServoByNumber(16,map(axes[3],-1024,1024,-90,90));
    setServoByNumber(17,map(axes[4],-1024,1024,-90,90));
    if(axes[5]<=0) {
      setServoByNumber(18,map(axes[5],-1024,0,-45,0));
    } else {
      setServoByNumber(18,map(axes[5],0,1024,0,135));
    }
  }
}

void setup() {
  Serial.begin(115200);
  // ---------------NRF24L01-----------
  radio.begin();
  radio.setPALevel(RF24_PA_LOW); //RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.openWritingPipe(addresses[0]); // 02
  radio.openReadingPipe(1, addresses[1]); // 01
  setPayloadContent();
  // load the payload for the first received transmission on pipe 0
  radio.writeAckPayload(1, &dataToRead, sizeof(dataToRead));
  radio.startListening();  // put radio in RX mode
  // ---------------NRF24L01-----------
  pwmBoard.begin();
  pwmBoard.setPWMFreq(FREQUENCY);
  S1.attach(15,SERVOMIN,SERVOMAX); // WAS 14, THEN 9, NOW 2
  S2.attach(0,SERVOMIN-200,SERVOMAX); // WAS 12, THEN 10, NOW 15
  Serial.println("start homing, please wait.");
  homeServos();
  lastTimestamp = millis();
}

void loop() {
  // get next set of values
  handleRadio();
  long currentTimestamp = millis();
  if(currentTimestamp-lastTimestamp>servoLoopTime) {
    // save current timestamp for next loop
    //lastTimestamp = currentTimestamp;
    // process the values
    processVariables();
  }
}
