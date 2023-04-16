/**
 * Program to test out servo calibration and standing up / sitting down routines
 * Created:   28.06.2022
 * Modified:  29.06.2022
 * Author:    Philipp Schulz
 */
#include <Wire.h>
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
//s:          1  2  3  4  5  6-        7  8  9 10 11 12 13 14 15 16 17 18 6+
float m[] = {-1, 1, 1, 1, 1,1.16667  ,-1, 1, 1, 1, 1, 1,-1, 1, 1, 1, 1, 1,0.833333};
float b[] = {90,90,45,90,90,30,       90,90,45,90,90,45,90,90,45,90,90,45,30};
                     // 1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18
float servoValues[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// variables for homing
long startAngle = 0;
int homingDelay = 1000;

//variables for movement patterns
int cycleTime = 10;

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

// method to utilize servo calibration values
float servoCalibration(int servoNumber, float targetPosition) {
  // initialize return value
  float newPosition = 0;
  // check servoNumber
  if(servoNumber == 6) {    // servo number 6 needs more complex calibration
    if(targetPosition < 75) { // handle negative angles, use "6-" calibration values
      newPosition = m[servoNumber-1]*targetPosition + b[servoNumber-1];
    } else {                // handle positive angles, use "6+" calibration values
      newPosition = m[18]*targetPosition + b[18];
    }
  } else {                  // rest of the servos can be done with calibration parameter arrays
    newPosition = m[servoNumber-1]*targetPosition + b[servoNumber-1];
  }
  return newPosition;
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
  // 2. all knees 60°
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
  target = 60;
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
  // 2. all knees 60°
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
  target = 60;
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


// method to home all servos based on the initial target values
void homeServos() {
  Serial.println("starting homing");
  setServoByNumber(2,startAngle);                            // home knee joint front left
  setServoByNumber(3,startAngle);                            // home foot joint front left
  delay(homingDelay);
  setServoByNumber(4,startAngle);                            // home knee joint front right
  setServoByNumber(5,startAngle);                            // home foot joint front right
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

void setup() {
  pwmBoard.begin();
  pwmBoard.setPWMFreq(FREQUENCY);
  S1.attach(9,SERVOMIN,SERVOMAX);
  S2.attach(10,SERVOMIN-200,SERVOMAX);
  Serial.begin(9600);
  homeServos();
  Serial.println("ready");
  Serial.println("set calibrated servo angle by the following notation: [servo number] [angle]");
  Serial.println("For setting all servos, use servo number 19 for hips, 20 for knees, 21 for feet");
  Serial.println("For standing up, use number 22 / for sitting down, use number 23");
}

void loop() {
  if(Serial.available()) {
    String serialData = Serial.readString();
    if(getValue(serialData,' ',0).toInt()<=18) {    // if a single servo should be moved
      setServoByNumber(getValue(serialData,' ',0).toInt(),getValue(serialData,' ',1).toInt());
    } else if(getValue(serialData,' ',0).toInt()==19) {  // if the hips should be moved
      setServoByNumber(1,getValue(serialData,' ',1).toInt());
      setServoByNumber(4,getValue(serialData,' ',1).toInt());
      setServoByNumber(7,getValue(serialData,' ',1).toInt());
      setServoByNumber(10,getValue(serialData,' ',1).toInt());
      setServoByNumber(13,getValue(serialData,' ',1).toInt());
      setServoByNumber(16,getValue(serialData,' ',1).toInt());
    } else if(getValue(serialData,' ',0).toInt()==20) {  // if the knees should be moved
      setServoByNumber(2,getValue(serialData,' ',1).toInt());
      setServoByNumber(5,getValue(serialData,' ',1).toInt());
      setServoByNumber(8,getValue(serialData,' ',1).toInt());
      setServoByNumber(11,getValue(serialData,' ',1).toInt());
      setServoByNumber(14,getValue(serialData,' ',1).toInt());
      setServoByNumber(17,getValue(serialData,' ',1).toInt());
    } else if(getValue(serialData,' ',0).toInt()==21) {  // if the feet should be moved
      setServoByNumber(3,getValue(serialData,' ',1).toInt());
      setServoByNumber(6,getValue(serialData,' ',1).toInt());
      setServoByNumber(9,getValue(serialData,' ',1).toInt());
      setServoByNumber(12,getValue(serialData,' ',1).toInt());
      setServoByNumber(15,getValue(serialData,' ',1).toInt());
      setServoByNumber(18,getValue(serialData,' ',1).toInt());
    } else if(getValue(serialData,' ',0).toInt()==22) { // if the robot should stand up
      standUp();
    } else if(getValue(serialData,' ',0).toInt()==23) { // if the robot should sit down
      sitDown();
    }
    Serial.println(getValue(serialData,' ',0)+" : "+getValue(serialData,' ',1));
  }
}
