/**
 * Program to test out inverse kinematics, second iteration after first had problems
 * Created:   05.10.2022
 * Modified:  05.10.2022
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
//s:          1  2  3  4  5  6          7  8  9 10 11 12 13 14 15 16 17 18
float m[] = {-1, 1, 1, 1, 1, 0.833333 ,-1, 1, 1, 1, 1, 1,-1, 1, 1, 1, 1, 1};
float b[] = {90,90,45,90,90, 35.5,     90,90,45,90,90,45,90,90,45,90,90,45};
                     // 1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18
float servoValues[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// default translation and rotation for any leg, any additional input will be added to these values
// these values are set starting from the first axis of each leg
float defaultLegTransform[] = {44,0,-69.5,0,0,0}; // x y z alpha beta gamma

// variables for homing
long startAngle = 0;
int homingDelay = 500;

//variables for movement patterns
int cycleTime = 10;

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
  // temporary print to terminal for debugging
  for(int i=0;i<6;i++) {
    Serial.println(String(i+1)+" "+String(kartesianX[i])+", "+String(kartesianY[i])+", "+String(kartesianZ[i]));
  }
  /**
  // -------------------1. yaw-----------------------
  // front servos
  float dy = 76.15*(1-cos(a*(PI/180.0)));
  float dz = 76.15*sin(a*(PI/180.0));
  kartesianY[0] -= dy;
  kartesianZ[0] -= dz;
  kartesianY[1] -= dy;
  kartesianZ[1] -= dz;
  // back servos
  kartesianY[4] += dy;
  kartesianZ[4] += dz;
  kartesianY[5] += dy;
  kartesianZ[5] += dz;
  
  // -------------------2. pitch---------------------
  // front servos
  float dx = -60*(1-cos(b*(PI/180.0)));
  dz = -60*sin(b*(PI/180.0));
  kartesianX[0] -= dx;
  kartesianZ[0] += dz;
  kartesianX[1] += dx;
  kartesianZ[1] -= dz;
  // back servos
  kartesianX[4] -= dx;
  kartesianZ[4] += dz;
  kartesianX[5] += dx;
  kartesianZ[5] -= dz;
  // middle servos
  dx = 55.89*(1-cos(b*(PI/180.0)));
  dz = 55.89*sin(b*(PI/180.0));
  kartesianX[2] += dx;
  kartesianZ[2] -= dz;
  kartesianX[3] -= dx;
  kartesianZ[3] += dz;
  
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
  **/
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
  x1 = sqrt(pow(-kartesianX[2]-37.5-28.5,2)+pow(kartesianY[2],2))-32.25;
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
  x1 = sqrt(pow(kartesianX[3]-37.5-28.5,2)+pow(kartesianY[3],2))-32.25;
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
void setIKMServos(int legNumber) {
  // currently for debugging, real code comes later
  Serial.println("X: "+String(kartesianX[legNumber-1]));
  Serial.println("Y: "+String(kartesianY[legNumber-1]));
  Serial.println("Z: "+String(kartesianZ[legNumber-1]));
  // print servo angles
  Serial.println("leg "+String(legNumber)+" angles: "+String(kartesianAngles[((legNumber-1)*3)])+" "+String(kartesianAngles[((legNumber-1)*3)+1])+" "+String(kartesianAngles[((legNumber-1)*3)+2]));
  // set servo angles
  setServoByNumber((legNumber-1)*3,kartesianAngles[(legNumber-1)*3-1]);
  setServoByNumber((legNumber-1)*3+1,kartesianAngles[(legNumber-1)*3]);
  setServoByNumber((legNumber-1)*3+2,kartesianAngles[(legNumber-1)*3+1]);
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

void setup() {
  pwmBoard.begin();
  pwmBoard.setPWMFreq(FREQUENCY);
  S1.attach(2,SERVOMIN,SERVOMAX); // WAS 14, THEN 9, NOW 2
  S2.attach(15,SERVOMIN-200,SERVOMAX); // WAS 12, THEN 10, NOW 15
  Serial.begin(9600);
  Serial.println("start homing, please wait.");
  homeServos();
  Serial.println("ready");
  Serial.println("set calibrated servo angle by the following notation: [servo number] [angle]");
  Serial.println("For setting all servos, use servo number 19 for hips, 20 for knees, 21 for feet");
  Serial.println("For standing up, use number 22 / for sitting down, use number 23");
  Serial.println("For IKM, use the following notation: 24 [legNumber] [X] [Y] [Z] [alpha] [beta] [gamma]");
  Serial.println("for homing, use number 25");
}

void loop() {
  if(Serial.available()) {
    String serialData = Serial.readString();
    Serial.print("command: "+serialData);
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
    } else if(getValue(serialData,' ',0).toInt()==24) { // if the robot should move according to IKM
      int legNumber = getValue(serialData,' ',1).toFloat();
      kartesian[0] = getValue(serialData,' ',2).toFloat();
      kartesian[1] = getValue(serialData,' ',3).toFloat();
      kartesian[2] = getValue(serialData,' ',4).toFloat();
      kartesian[3] = getValue(serialData,' ',5).toFloat();
      kartesian[4] = getValue(serialData,' ',6).toFloat();
      kartesian[5] = getValue(serialData,' ',7).toFloat();
      inverseKinematics();
      setIKMServos(legNumber);
    } else if(getValue(serialData,' ',0).toInt()==25) {  // if the servos should be homed
      homeServos();
    }
    //Serial.println(getValue(serialData,' ',0)+" : "+getValue(serialData,' ',1));
    Serial.println("done");
  }
}
