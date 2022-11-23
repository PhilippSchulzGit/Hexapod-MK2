/**
 * Program to calibrate positions of all servos
 * Created:   08.05.2022
 * Modified:  28.05.2022
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

//TODO: IMPLEMENT SERVO MODELS FROM CALIBRATION

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

// variables for homing
long startAngle = 0;
int homingDelay = 1000;

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

// method to set a servo by its number to a new targetPosition
void setServoByNumber(int servoNumber, float targetPosition) {
  //TODO: CHANGE SERVO CONNECTIONS (CURRENTLY ALL SET TO 9)
  switch(servoNumber) {                                   // go over all possible servo numbers
    case 1:
      pwmBoard.setPWM(1,0,pulseWidth(targetPosition));    // move servo 1 in position
      break;
    case 2:
      pwmBoard.setPWM(0,0,pulseWidth(targetPosition));    // move servo 2 in position
      break;
    case 3:
      S1.write(targetPosition);                           // move servo 3 in position
      break;
    case 4:
      pwmBoard.setPWM(15,0,pulseWidth(targetPosition));   // move servo 4 in position
      break;
    case 5:
      pwmBoard.setPWM(14,0,pulseWidth(targetPosition));   // move servo 5 in position
      break;
    case 6:
      S2.write(targetPosition);                           // move servo 6 in position
      break;
    case 7:
      pwmBoard.setPWM(4,0,pulseWidth(targetPosition));    // move servo 7 in position
      break;
    case 8:
      pwmBoard.setPWM(3,0,pulseWidth(targetPosition));    // move servo 8 in position
      break;
    case 9:
      pwmBoard.setPWM(2,0,pulseWidth(targetPosition));    // move servo 9 in position
      break;
    case 10:
      pwmBoard.setPWM(11,0,pulseWidth(targetPosition));   // move servo 10 in position
      break;
    case 11:
      pwmBoard.setPWM(12,0,pulseWidth(targetPosition));   // move servo 11 in position
      break;
    case 12:
      pwmBoard.setPWM(13,0,pulseWidth(targetPosition));   // move servo 12 in position
      break;
    case 13:
      pwmBoard.setPWM(7,0,pulseWidth(targetPosition));    // move servo 13 in position
      break;
    case 14:
      pwmBoard.setPWM(6,0,pulseWidth(targetPosition));    // move servo 14 in position
      break;
    case 15:
      pwmBoard.setPWM(5,0,pulseWidth(targetPosition));    // move servo 15 in position
      break;
    case 16:
      pwmBoard.setPWM(8,0,pulseWidth(targetPosition));    // move servo 16 in position
      break;
    case 17:
      pwmBoard.setPWM(9,0,pulseWidth(targetPosition));    // move servo 17 in position
      break;
    case 18:
      pwmBoard.setPWM(10,0,pulseWidth(targetPosition));   // move servo 18 in position
      break;
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
  S1.attach(14,SERVOMIN,SERVOMAX);
  S2.attach(12,SERVOMIN-200,SERVOMAX);
  Serial.begin(9600);
  //homeServos();
  Serial.println("ready");
  Serial.println("set servo angle by the following notation: [servo number] [angle]");
}

void loop() {
  if(Serial.available()) {
    String serialData = Serial.readString();
    setServoByNumber(getValue(serialData,' ',0).toInt(),getValue(serialData,' ',1).toInt());
    Serial.println(getValue(serialData,' ',0)+" : "+getValue(serialData,' ',1));
  }
}
