#include <AutoPID.h>
#include <Encoder.h>

#define Kp 0
#define Ki 0
#define Kd 0
#define OUTMAX 200
#define OUTMIN -200
// values for converting the millimeters to encoder amounts
#define WHEEL_DIAMETER 64   // ALL MEASUREMENTS IN MILLIMETERS
#define ENC_COUNTS_PER_TURN 255 // NEED TO MEASURE
#define WHEEL_TO_CENTERLINE 20 // NEED TO MEASURE
// Pins to drive the motors MUST BE PWM
#define MOT1_A 4
#define MOT1_B 5
#define MOT2_A 6
#define MOT2_B 7

double input_L, setpoint_L, output_L; // left Data
double input_R, setpoint_R, output_R; // Right Data
//  Variables for the encoder update function, made global for debugging purposes
long old_L = -999;
long old_R = -999;
long newencL = 0;
long newencR = 0;

bool sensorFlag = false;
byte Position = 0;
//  Encoder declaration
Encoder encL(2, 3);   // left encoder interuptable pins
Encoder encR(20, 21); // right encoder interuptable pins

//  PID declaration
AutoPID pidCont_L(&input_L, &setpoint_L, &output_L, OUTMAX, OUTMIN, Kp, Ki, Kd);
AutoPID pidCont_R(&input_R, &setpoint_R, &output_R, OUTMAX, OUTMIN, Kp, Ki, Kd);

void setup() {
  attachInterrupt(digitalPinToInterrupt(18), pong(), RISING);
  // put your setup code here, to run once:
  //  FIRST SET IS THE PINS TO RUN THE LEFT MOT MUST BE PWM
  pinMode(MOT1_A, OUTPUT);
  pinMode(MOT1_B, OUTPUT);
  //  SECOND SET IS THE PINS TO RUN THE RIGHT MOT MUST BE PWM
  pinMode(MOT2_A, OUTPUT);
  pinMode(MOT2_B, OUTPUT);

  
  //  Setting up the Serial and stuff
  Serial.begin(9600);
}
int toggle = 0;
void loop() {
  if (Position > 13);;
  //  Case Code for updating PID Values once the correct location is reached
  switch (Position) {
    case (0):   //turn 45deg
      if (!toggle) {
        turnAngleDeg(-45); // calc setpoints to turn 45 deg CW
        toggle++;
      }
      if (pidCont_L.atSetPoint(1) && pidCont_R.atSetPoint(1)) {
        Position = 1;
        toggle = 0;
      }
      break;
    case (1):   //drive forward to approx the centerline
      if (!toggle) {
        setpoint_L = old_L + computeDist(1860);
        setpoint_R = old_R + computeDist(1860);
      }
      if (pidCont_L.atSetPoint(1) && pidCont_R.atSetPoint(1)) {
        Position = 2;
        toggle = 0;
      }
      break;
    case (2):   // turn other way 45 deg
      if (!toggle) {
        turnAngleDeg(45);
        toggle++;
      }
      if (pidCont_L.atSetPoint(1) && pidCont_R.atSetPoint(1)) {
        Position = 3;
        toggle = 0;
      }
      break;
    case (3):   // drive to around the half way point (Just a correction amount)
      if (!toggle) {
        setpoint_L = old_L + computeDist(50);
        setpoint_R = old_R + computeDist(50);
      }
      if (pidCont_L.atSetPoint(1) && pidCont_R.atSetPoint(1)) {
        Position = 4;
        toggle = 0;
      }
      break;
    case (4):   // sweeeeeeeeeeeeeeeep till detect
//atach
// search for code
//detach
      break;
    case (5):   // drive until it touches

      break;
    case (6):   // drive back until where 5 started

      break;
    case (7):   // unturn the till the angle the 4 detected

      break;
    case (8):   // 180

      break;
    case (9):   // sweep again

      break;
    case (10):  // drive again at that new angle until hit

      break;
    case (11):  // un-drive that distance that case 10 drove

      break;
    case (12):  // un turn the angle that case 9 found

      break;
    case (13):  // end

      break;
  }
  updateEncoders();
  updateMotors();

}

/*
    Update motors is used to figure out which way the motor is spinning to determine which of the
    two motor wires needs to be set to low.
*/
void updateMotors() {
  //  LEFT
  if (output_L < 0) {
    analogWrite(MOT1_A, output_L);
    digitalWrite(MOT1_B, LOW);
  }
  else if (output_L >= 0) {
    analogWrite(MOT1_B, output_L);
    digitalWrite(MOT1_A, LOW);
  }
  // RIGHT
  if (output_R < 0) {
    analogWrite(MOT2_A, output_R);
    digitalWrite(MOT2_B, LOW);
  }
  else if (output_R >= 0) {
    analogWrite(MOT2_B, output_R);
    digitalWrite(MOT2_A, LOW);
  }
}

void updateEncoders() {
  newencL = encL.read();
  newencR = encR.read();
  if (newencL != old_L) {
    old_L = newencL;
    input_L = (double)old_L;
  }
  if (newencR != old_R) {
    old_R = newencR;
    input_R = (double)old_R;
  }
}

/*
   Returns the number of encoder counts required to move the given number of millimeters
*/
long computeDist(int millimeters) {
  return (millimeters / ((3.14159) * WHEEL_DIAMETER)) * ENC_COUNTS_PER_TURN;
}

/*
    Follows the formula
    (New set point) = (Old set point) +- (arclength made by that degree turn)
    where the arclength = ((circumference)*percentage of 360 turned))

    The code will auto make the setpoints so no need to return stuff <3
*/
void turnAngleDeg(int deg) {
  if (deg > 0) { // assume positive turns to be CCW
    setPoint_R = old_R + computeDist((2 * WHEEL_TO_CENTERLINE * 3.14159) * (deg / 360));
    setPoint_L = old_L - computeDist((2 * WHEEL_TO_CENTERLINE * 3.14159) * (deg / 360));
  }
  if (deg < 0) { // assume negative to be CW
    setPoint_R = old_R - computeDist((2 * WHEEL_TO_CENTERLINE * 3.14159) * (deg / 360));
    setPoint_L = old_L + computeDist((2 * WHEEL_TO_CENTERLINE * 3.14159) * (deg / 360));
  }
}
void pong(){
  if(Sensor_Active){
    sensorFlag = true;
  }
}

