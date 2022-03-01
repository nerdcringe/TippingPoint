/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*
STATES:
 - Full row w/ goal   wip
 - Left Side Fake (looks like center)       ok
 - 




FAIRGROUNDS
  - rightside         alright
  - fullrow           alright
  - skills            working on it
  - rightSide center  eh
  - leftSide 
  - leftSide center
  - Don't let go in tug of war in auton
*/


// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
using namespace vex;
// A global instance of competition
competition Competition;


#define PI 3.14159265

int i;
const double WHEEL_CIRCUMFERENCE = 3.25 * PI;

bool conveyorOnDriver = false;

// DriveStraight variables
double desiredDegree = 0.0;
/*
double KPC = 50.0;
double KDC = 0.12;
double correctionL = 0;
double correctionR = 0;
double cSpeed = 0;*/



// DEVICES //////////////////////////////////
controller controllerPrim(primary);

// MOTORS //////////////////////////////////////////
motor LFBASE(PORT15, true); // Left front
motor LMBASE(PORT4); // Left mid
motor LBBASE(PORT9, true); // Left back

motor RFBASE(PORT2); // Right front
motor RMBASE(PORT7, true); // Right mid
motor RBBASE(PORT6); // Right back  

// 13 bad

/* REVERSED
motor LFBASE(PORT13); // Left front
motor LMBASE(PORT14, true); // Left mid
motor LBBASE(PORT8); // Left back

motor RFBASE(PORT3, true); // Right front
motor RMBASE(PORT2); // Right mid
motor RBBASE(PORT1, true); // Right back
*/

motor ARM(PORT16, true);
motor CONVEYOR(PORT5, true);

triport threeWirePort(PORT22); // Use to reference all three-wire ports

digital_out LCLAMP(threeWirePort.F); // Pneumatic device
digital_out RCLAMP(threeWirePort.G); // Pneumatic device

digital_out SHIELD(threeWirePort.D); // Pneumatic device

digital_out LFORKLIFT(threeWirePort.H); // Pneumatic device
digital_out RFORKLIFT(threeWirePort.E); // Pneumatic device


// SENSORS //////////////////////////////////
inertial Inertial(PORT20);
line CLAMPLINE(threeWirePort.A);
line WALLLINE(threeWirePort.B);



bool clampMogoDetected()
{
  return CLAMPLINE.value(pct) <= 62;
}

bool wallDetected()
{
  return WALLLINE.value(pct) <= 62;
}



// MATH FUNCTIONS /////////////////////////////////////////////

double inchesToTicks(double inches) {
  // Derived from: length/circumference = deg/360
  double ticks = (inches / WHEEL_CIRCUMFERENCE) * 360;
  return ticks;
}

double ticksToInches(double ticks) {
  double inches = (ticks * WHEEL_CIRCUMFERENCE) / 360;
  return inches;
}

double keepInRange(double number, double bottom, double top)
{
  if (number < bottom)
  {
    number = bottom;
  }
  if (number > top)
  {
    number = top;
  }
  return number;
}


// BASE MOVEMENT //////////////////////////////

void leftDrive(double power)
{
  LFBASE.spin(fwd, power, pct);
  LMBASE.spin(fwd, power, pct);
  LBBASE.spin(fwd, power, pct);
}

void rightDrive(double power)
{
  RFBASE.spin(fwd, power, pct);
  RMBASE.spin(fwd, power, pct);
  RBBASE.spin(fwd, power, pct);
}


void stopBase()
{
  LFBASE.stop(coast);
  LMBASE.stop(coast);
  LBBASE.stop(coast);

  RFBASE.stop(coast);
  RMBASE.stop(coast);
  RBBASE.stop(coast);
}

double encoderAverage()
{
  double sum = LFBASE.rotation(deg)
             + LMBASE.rotation(deg)
             + LBBASE.rotation(deg)
             + RFBASE.rotation(deg)
             + RMBASE.rotation(deg)
             + RBBASE.rotation(deg);
  return sum / 6;
}

void clearEncoders()
{
  RFBASE.resetRotation();
  RMBASE.resetRotation();
  RBBASE.resetRotation();

  LFBASE.resetRotation();
  LMBASE.resetRotation();
  LBBASE.resetRotation();
}

double getRotation()
{
  return Inertial.rotation(deg); // IMPORTANT that value is NEGATIVE
}



void simpleForward(double inches, double speed)
{
  double ticks = inchesToTicks(inches);
  clearEncoders();
  double error = ticks;
  while ((inches > 0 && error > 0) || (inches < 0 && error < 0))
  {
    error = ticks - encoderAverage(); // desired - actual
    leftDrive(speed);
    rightDrive(speed);
  }
  stopBase();
}


void smoothSpeed(double inches, double startSpeed, double endSpeed)
{
  double setpoint = inchesToTicks(inches);
  clearEncoders();
  double error = setpoint;
  while (fabs(error) < 4)
  {
    error = setpoint - encoderAverage(); // desired - actual
    double proportion = fabs(encoderAverage())/fabs(setpoint); // 0 at start and 1 at end
    
    // speed goes from startSpeed -> endSpeed proportional to distance made
    double speed = proportion * (endSpeed - startSpeed);
    speed += startSpeed;

    if (proportion > 1)
    {
      speed = endSpeed;
    }
    
    /*if ((setpoint > 0 && error < 0)
    ||  (setpoint < 0 && error > 0 )) // if going past setpoint
    {
      speed = endSpeed;
    }*/
    
    if (error < 0)
    {
      speed *= -1;
    }
    leftDrive(speed);
    rightDrive(speed);
  }
  stopBase();
}


// no time to correct
void smoothSpeedNoCorrect(double inches, double startSpeed, double endSpeed)
{
  double ticks = inchesToTicks(inches);
  clearEncoders();
  double error = ticks;
  while ((inches > 0 && error > 0) || (inches < 0 && error < 0))
  {
    error = ticks - encoderAverage(); // desired - actual
    double proportion = encoderAverage()/ticks; // 0 at start and 1 at end
    
    // speed goes from startSpeed -> endSpeed proportional to distance made
    double speed = proportion * (endSpeed - startSpeed);
    speed += startSpeed;
    leftDrive(speed);
    rightDrive(speed);
  }
  stopBase();
}


// P controller with a trapezoidal motion profile
// Use negative distance to go backwards
// More info: https://www.vexforum.com/t/advanced-pid-and-motion-profile-control/28400/3
void forwardInches(double inches, int maxSpeed)
{
  clearEncoders();
  maxSpeed = keepInRange(maxSpeed, 0, 100);

  //MinSpeed : replacement for I in PID. Ensures the movement overcomes friction.
  const double minSpeed = 2;      // Lowest speed the motors will go; Turning is more precise but jankier when lower.

  // Acceleration rates: replacement for D in PID, changes rate of speed up/slow down
  // Gradually speeds up for the first half, slows down for second half
  const double accelRate = 30;    // Speed multiplier while speeding up (slope at start).
  const double deaccelRate = 8;  // Speed multiplier while slowing down (slope at end). Starts slowing down later when higher

  double targetDistance = inchesToTicks(inches);  // How far the robot should travel
  double targetRange = 2;                         // Distance from the desired distance the robot has to be to stop.
  double error = targetDistance;                  // Distance from the desired distance
  double speed;                                   // Actual speed value of the motors

  while (fabs(error) > targetRange) {

    error = targetDistance - encoderAverage(); // Error = desired - actual

    // First half: accelerate to max. Second half, deccelerate to min
      // Speed is proportional to the error/target distance.
      // Since error is always decreasing, the speed proportion is inverted (subtracted from 1) for the first half of distance
      // travelled so speed increases proportionally instead of decreasing. 
      // fabs() (absolute value) is used because the direction is determined by the sign of the error, not of the speed.
    if (fabs(error) > fabs(targetDistance/2))
    {
      speed = (1 - (fabs(error) / fabs(targetDistance))) * accelRate; // Speeds up as error decreases
    }
    else
    {
      speed = (fabs(error) / fabs(targetDistance)) * deaccelRate;  // Slows down as error increases
    }
    speed = fabs(speed);
    speed *= maxSpeed;
    speed = keepInRange(speed, minSpeed, maxSpeed);

    // Go forward if positive error and backwards if negative error
    if (error < 0)
    {
      speed *= -1;
    }
    leftDrive(speed);
    rightDrive(speed);

    Brain.Screen.printAt(1, 60, "Target Dist: %.2f  ", ticksToInches(targetDistance));
    Brain.Screen.printAt(1, 80, "Progress:    %.2f  ", ticksToInches(encoderAverage()));
    Brain.Screen.printAt(1, 120,"Error:       %.2f ticks, %.2f \"inches\"     ", error, ticksToInches(error));
    Brain.Screen.printAt(1, 100,"Speed:       %.2f  ", speed);
  }

  stopBase();
  task::sleep(15);
}

/*
// Go a certain distance by speeding up and slowing down smoothly
// More advanced info about motion profiling: https://www.vexforum.com/t/advanced-pid-and-motion-profile-control/28400/3
void forwardInchesNew(double desiredInches, int maxSpeed)
{
  clearEncoders();
  maxSpeed = keepInRange(maxSpeed, 0, 100);

  const double minSpeed = 2;      // Minimum speed needed to overcome friction

  // Gradually speeds up for the first half, slows down for second half
  const double accelRate = 30;    // Speed multiplier while speeding up (How fast it accelerates).
  const double deaccelRate = 3;  // Speed multiplier while slowing down (How fast it deaccelerates).

  double targetDistance = inchesToTicks(desiredInches);  // How far the robot should travel
  double targetRange = 2;                         // Distance from the desired distance the robot has to be to stop.
  double error = targetDistance;                  // Distance from the desired distance
  
  while (fabs(error) > targetRange)
  {
    // Error is the current distance required to reach the target distance
    // Error is always decreasing because robot is getting closer to target
    error = targetDistance - encoderAverage(); // Error = desired - actual

    // Ratio of distance to go out of the initial distance.
    // Use absolute value (fabs) to simplify the calculation
    double proportion = fabs(error) / fabs(targetDistance); // Goes from 1 -> 0

    double speed;
    if (proportion > 0.5) // First half, speed up
    {
      speed = (1 - proportion) * accelRate; // 1 minus proportion makes it go from 0 -> 1
    }
    else // Second half, slow down
    {
      speed = proportion * deaccelRate;  // Slows down as error increases
    }

    speed = fabs(speed); // Speed is positive to keep calculations simple

    // Speed is kept below maximum speed. Speed stays constant when at maxSpeed.
    speed *= 100;//maxSpeed;
    speed = keepInRange(speed, minSpeed, maxSpeed);

    // Set speed to negative when error is negative to go backwards
    if (error < 0)
    {
      speed *= -1;
    }
    leftDrive(speed);
    rightDrive(speed);

    Brain.Screen.printAt(1, 60, "Target Dist: %.2f", ticksToInches(targetDistance));
    Brain.Screen.printAt(1, 80, "Progress:    %.2f", ticksToInches(encoderAverage()));
    Brain.Screen.printAt(1, 120,"Error:       %.2f ticks, %.2f \"inches\"", error, ticksToInches(error));
    Brain.Screen.printAt(1, 100,"Speed:       %.2f", speed);
  }

  stopBase();
  task::sleep(15);
}*/





void forwardInchesTimed(double inches, int maxSpeed, int maxTimeMs)
{
  clearEncoders();
  maxSpeed = keepInRange(maxSpeed, 0, 100);

  //MinSpeed : replacement for I in PID. Ensures the movement overcomes friction.
  const double minSpeed = 2;      // Lowest speed the motors will go; Turning is more precise but jankier when lower.

  // Acceleration rates: replacement for D in PID, changes rate of speed up/slow down
  // Gradually speeds up for the first half, slows down for second half
  const double accelRate = 30;    // Speed multiplier while speeding up (slope at start).
  const double deaccelRate = 8;  // Speed multiplier while slowing down (slope at end). Starts slowing down later when higher

  double targetDistance = inchesToTicks(inches);  // How far the robot should travel
  double targetRange = 2;                         // Distance from the desired distance the robot has to be to stop.
  double error = targetDistance;                  // Distance from the desired distance
  double speed;                                   // Actual speed value of the motors

  timer Timer;
  Timer.clear();

  while (fabs(error) > targetRange && Timer.time(msec) <= maxTimeMs) {

    error = targetDistance - encoderAverage(); // Error = desired - actual

    // First half: accelerate to max. Second half, deccelerate to min
      // Speed is proportional to the error/target distance.
      // Since error is always decreasing, the speed proportion is inverted (subtracted from 1) for the first half of distance
      // travelled so speed increases proportionally instead of decreasing. 
      // fabs() (absolute value) is used because the direction is determined by the sign of the error, not of the speed.
    if (fabs(error) > fabs(targetDistance/2))
    {
      speed = (1 - (fabs(error) / fabs(targetDistance))) * accelRate; // Speeds up as error decreases
    }
    else
    {
      speed = (fabs(error) / fabs(targetDistance)) * deaccelRate;  // Slows down as error increases
    }
    speed = fabs(speed);
    speed *= maxSpeed;
    speed = keepInRange(speed, minSpeed, maxSpeed);

    // Go forward if positive error and backwards if negative error
    if (error < 0)
    {
      speed *= -1;
    }
    leftDrive(speed);
    rightDrive(speed);

    Brain.Screen.printAt(1, 60, "Target Dist: %.2f", ticksToInches(targetDistance));
    Brain.Screen.printAt(1, 80, "Progress:    %.2f", ticksToInches(encoderAverage()));
    Brain.Screen.printAt(1, 120,"Error:       %.2f ticks, %.2f \"inches\"", error, ticksToInches(error));
    Brain.Screen.printAt(1, 100,"Speed:       %.2f", speed);
  }

  stopBase();
  task::sleep(15);
}

// Moves forward at least a certain distance but doesn't correct if it overshoots
// As far as I know it only works for fwd, not backward
void forwardInchesFast(double inches, int maxSpeed, int msTimeout = -1, int deaccelRate=4)
{
  clearEncoders();
  maxSpeed = keepInRange(maxSpeed, 0, 100);

  //MinSpeed : replacement for I in PID. Ensures the movement overcomes friction.
  const double minSpeed = 12;      // Lowest speed the motors will go; Turning is more precise but jankier when lower.

  // Acceleration rates: replacement for D in PID, changes rate of speed up/slow down
  // Gradually speeds up for the first half, slows down for second half
  const double accelRate = 20;    // Speed multiplier while speeding up (slope at start).
  //const double deaccelRate = 4;  // Speed multiplier while slowing down (slope at end). Starts slowing down later when higher

  double targetDistance = inchesToTicks(inches);  // How far the robot should travel
  //double targetRange = 2;                         // Distance from the desired distance the robot has to be to stop.
  double error = targetDistance;                  // Distance from the desired distance
  double speed;                                   // Actual speed value of the motors

  desiredDegree = getRotation();
  Brain.resetTimer();

  while (((inches > 0 && error > 0) || (inches < 0 && error < 0))  && 
  (Brain.timer(vex::timeUnits::msec) < msTimeout || msTimeout == -1))
  {
    error = targetDistance - encoderAverage(); // Error = desired - actual

    // First half: accelerate to max. Second half, deccelerate to min
      // Speed is proportional to the error/target distance.
      // Since error is always decreasing, the speed proportion is inverted (subtracted from 1) for the first half of distance
      // travelled so speed increases proportionally instead of decreasing. 
      // fabs() (absolute value) is used because the direction is determined by the sign of the error, not of the speed.
    if (fabs(error) > fabs(targetDistance/2))
    {
      speed = (1 - (fabs(error) / fabs(targetDistance))) * accelRate; // Speeds up as error decreases
    }
    else
    {
      speed = (fabs(error) / fabs(targetDistance)) * deaccelRate;  // Slows down as error increases
    }
    speed = fabs(speed);
    speed *= maxSpeed;
    speed = keepInRange(speed, minSpeed, maxSpeed);

    // Go forward if positive error and backwards if negative error
    if (error < 0)
    {
      speed *= -1;
    }
    leftDrive(speed/* + correctionL*/);
    rightDrive(speed/* + correctionR*/);

    // Keep speed either above maxSpeed or below negative maxSpeed so it's fast enough to overcome friction
    /*if (speed > 0) {
      speed = keepInRange(speed, minSpeed, maxSpeed);//fmax(speed, minSpeed);  //  Larger than + minSpeed
    } else {
      speed = keepInRange(speed, -maxSpeed, -minSpeed);//
      //speed = //fmin(speed, -minSpeed); // Smaller than - minSpeed
    }*/


    Brain.Screen.printAt(1, 60, "Target Dist: %.2f", ticksToInches(targetDistance));
    Brain.Screen.printAt(1, 80, "Progress:    %.2f", ticksToInches(encoderAverage()));
    Brain.Screen.printAt(1, 120,"Error:       %.2f ticks, %.2f \"inches\"", error, ticksToInches(error));
    Brain.Screen.printAt(1, 120,"Error:       %.2f", error);
    Brain.Screen.printAt(1, 100,"Speed:       %.2f", speed);
  }

  stopBase();
  task::sleep(15);
}


// Moves forward at least a certain distance but doesn't correct if it overshoots
// As far as I know it only works for fwd, not backward
void forwardInchesSpeedUp(double inches, int maxSpeed)
{
  clearEncoders();
  maxSpeed = keepInRange(maxSpeed, 0, 100);

  //MinSpeed : replacement for I in PID. Ensures the movement overcomes friction.
  const double minSpeed = 20;      // Lowest speed the motors will go; Turning is more precise but jankier when lower.

  // Acceleration rates: replacement for D in PID, changes rate of speed up/slow down
  // Gradually speeds up for the first half, slows down for second half
  const double accelRate = 20;    // Speed multiplier while speeding up (slope at start).
  //const double deaccelRate = 3;  // Speed multiplier while slowing down (slope at end). Starts slowing down later when higher

  double targetDistance = inchesToTicks(inches);  // How far the robot should travel
  //double targetRange = 2;                         // Distance from the desired distance the robot has to be to stop.
  double error = targetDistance;                  // Distance from the desired distance
  double speed;                                   // Actual speed value of the motors

  desiredDegree = getRotation();

  while ((inches > 0 && error > 0) || (inches < 0 && error < 0))
  {
    error = targetDistance - encoderAverage(); // Error = desired - actual

    // First half: accelerate to max. Second half, deccelerate to min
      // Speed is proportional to the error/target distance.
      // Since error is always decreasing, the speed proportion is inverted (subtracted from 1) for the first half of distance
      // travelled so speed increases proportionally instead of decreasing. 
      // fabs() (absolute value) is used because the direction is determined by the sign of the error, not of the speed.
    /*if (fabs(error) > fabs(targetDistance/2))
    {*/
      speed = (1 - (fabs(error) / fabs(targetDistance))) * accelRate; // Speeds up as error decreases
    /*}
    else
    {
      speed = (fabs(error) / fabs(targetDistance)) * deaccelRate;  // Slows down as error increases
    }*/
    speed = fabs(speed);
    speed *= maxSpeed;
    speed = keepInRange(speed, minSpeed, maxSpeed);

    // Go forward if positive error and backwards if negative error
    if (error < 0)
    {
      speed *= -1;
    }
    leftDrive(speed/* + correctionL*/);
    rightDrive(speed/* + correctionR*/);

    // Keep speed either above maxSpeed or below negative maxSpeed so it's fast enough to overcome friction
    /*if (speed > 0) {
      speed = keepInRange(speed, minSpeed, maxSpeed);//fmax(speed, minSpeed);  //  Larger than + minSpeed
    } else {
      speed = keepInRange(speed, -maxSpeed, -minSpeed);//
      //speed = //fmin(speed, -minSpeed); // Smaller than - minSpeed
    }*/


    Brain.Screen.printAt(1, 60, "Target Dist: %.2f", ticksToInches(targetDistance));
    Brain.Screen.printAt(1, 80, "Progress:    %.2f", ticksToInches(encoderAverage()));
    Brain.Screen.printAt(1, 120,"Error:       %.2f ticks, %.2f \"inches\"", error, ticksToInches(error));
    Brain.Screen.printAt(1, 120,"Error:       %.2f", error);
    Brain.Screen.printAt(1, 100,"Speed:       %.2f", speed);
  }

  stopBase();
  task::sleep(15);
}



// Stops when detects mobile goal on clamp line sensor
void forwardInchesDetect(double maxInches, int maxSpeed)
{
  clearEncoders();
  maxSpeed = keepInRange(maxSpeed, 0, 100);

  //MinSpeed : replacement for I in PID. Ensures the movement overcomes friction.
  const double minSpeed = 12;      // Lowest speed the motors will go; Turning is more precise but jankier when lower.

  // Acceleration rates: replacement for D in PID, changes rate of speed up/slow down
  // Gradually speeds up for the first half, slows down for second half
  const double accelRate = 15;    // Speed multiplier while speeding up (slope at start).
  const double deaccelRate = 5;  // Speed multiplier while slowing down (slope at end). Starts slowing down later when higher

  double targetDistance = inchesToTicks(maxInches);  // How far the robot should travel
  //double targetRange = 2;                         // Distance from the desired distance the robot has to be to stop.
  double error = targetDistance;                  // Distance from the desired distance
  double speed;                                   // Actual speed value of the motors

  desiredDegree = getRotation();

  while (error > 0 and !clampMogoDetected()) {

    error = targetDistance - encoderAverage(); // Error = desired - actual

    // First half: accelerate to max. Second half, deccelerate to min
      // Speed is proportional to the error/target distance.
      // Since error is always decreasing, the speed proportion is inverted (subtracted from 1) for the first half of distance
      // travelled so speed increases proportionally instead of decreasing. 
      // fabs() (absolute value) is used because the direction is determined by the sign of the error, not of the speed.
    if (fabs(error) > fabs(targetDistance/2))
    {
      speed = (1 - (fabs(error) / fabs(targetDistance))) * accelRate; // Speeds up as error decreases
    }
    else
    {
      speed = (fabs(error) / fabs(targetDistance)) * deaccelRate;  // Slows down as error increases
    }
    speed = fabs(speed);
    speed *= maxSpeed;
    speed = keepInRange(speed, minSpeed, maxSpeed);

    // Go forward if positive error and backwards if negative error
    if (error < 0)
    {
      speed *= -1;
    }
    leftDrive(speed/* + correctionL*/);
    rightDrive(speed/* + correctionR*/);


    Brain.Screen.printAt(1, 60, "Target Dist: %.2f", ticksToInches(targetDistance));
    Brain.Screen.printAt(1, 80, "Progress:    %.2f", ticksToInches(encoderAverage()));
    Brain.Screen.printAt(1, 120,"Error:       %.2f ticks, %.2f \"inches\"", error, ticksToInches(error));
    Brain.Screen.printAt(1, 120,"Error:       %.2f", error);
    Brain.Screen.printAt(1, 100,"Speed:       %.2f", speed);
  }

  stopBase();
  task::sleep(15);
}



// Absolute turn in degrees
void gyroTurn(double targetAngle, int maxSpeed) {

  desiredDegree = targetAngle;
  double initialAngle = getRotation();
  //          <---------------------------|------------------->
  // Negative Degrees (Counterclockwise),    Positive Degrees (Clockwise)
  double relativeAngle = targetAngle - getRotation();

  double targetRange = 2; // Distance from the desired angle that is allowed
  double error = relativeAngle; // Distance from the desired range
  double progress;              // Degrees the robot has turned already
  double minSpeed = 5; // Lowest speed the motors will go; Turning is generally more precise when lower, but slower.
  double deaccelRate = 2.5;//3; // 2.4

  double speed; // Actual speed value of the motors

  maxSpeed = keepInRange(maxSpeed, 0, 100);

  while (fabs(error) > targetRange) {
    progress = getRotation() - initialAngle;
    error = progress - relativeAngle;

    // Speed starts at maximum and approaches minimum as the gyro value
    // approaches the desired angle. It deccelerates for precision.
    speed = fabs(error / relativeAngle); // Speed is absolute valued so that it can be kept in range properly
    speed *= maxSpeed * deaccelRate;
    speed = keepInRange(speed, minSpeed, maxSpeed);
    
    // Keep speed either above maxSpeed or below negative maxSpeed so it's fast enough to overcome friction
    if (error > 0) {
      leftDrive(-speed);
      rightDrive(speed);
    } else {
      leftDrive(speed);
      rightDrive(-speed);
    }

    /*controllerPrim.Screen.clearScreen();
    task::sleep(10);
    controllerPrim.Screen.setCursor(1, 1);
    task::sleep(10);
    controllerPrim.Screen.print("%f", getRotation());*/

     Brain.Screen.printAt(1, 120, "Desired angle: %.2f, Relative angle: %.2f",
     degrees, relativeAngle);
     Brain.Screen.printAt(1, 140, "Speed: %.2f, Error: %.2f", speed, error);
     Brain.Screen.printAt(1, 180, "Gyro: %.2f", getRotation());
  }

  stopBase();
  task::sleep(15);
}




// Relative turn in degrees
void relativeTurn(double relativeAngle, int maxSpeed) {

  // new angle is added to desired degree
  desiredDegree += relativeAngle;
  //double initialAngle = getRotation();
  //          <---------------------------|------------------->
  // Negative Degrees (Counterclockwise),    Positive Degrees (Clockwise)
  double targetRange = 2; // Distance from the desired angle that is allowed
  double error = desiredDegree - getRotation(); // Distance from the desired range
  //double progress;              // Degrees the robot has turned already
  double minSpeed = 5; // Lowest speed the motors will go; Turning is generally more precise when lower, but slower.
  double deaccelRate = 2.5;//3; // 2.4

  double speed; // Actual speed value of the motors

  maxSpeed = keepInRange(maxSpeed, 0, 100);

  while (fabs(error) > targetRange) {
    error = desiredDegree - getRotation();
    //progress = getRotation() - initialAngle;
    //error = progress - relativeAngle;

    // Speed starts at maximum and approaches minimum as the gyro value
    // approaches the desired angle. It deccelerates for precision.
    speed = fabs(error / relativeAngle); // Speed is absolute valued so that it can be kept in range properly
    speed *= maxSpeed * deaccelRate;
    speed = keepInRange(speed, minSpeed, maxSpeed);
    
    // Keep speed either above maxSpeed or below negative maxSpeed so it's fast enough to overcome friction
    if (error > 0) {
      leftDrive(-speed);
      rightDrive(speed);
    } else {
      leftDrive(speed);
      rightDrive(-speed);
    }

    /*controllerPrim.Screen.clearScreen();
    task::sleep(10);
    controllerPrim.Screen.setCursor(1, 1);
    task::sleep(10);
    controllerPrim.Screen.print("%f", getRotation());*/

     Brain.Screen.printAt(1, 120, "Desired angle: %.2f, Relative angle: %.2f",
     degrees, relativeAngle);
     Brain.Screen.printAt(1, 140, "Speed: %.2f, Error: %.2f", speed, error);
     Brain.Screen.printAt(1, 180, "Gyro: %.2f", getRotation());
  }

  stopBase();
  task::sleep(15);
}



// Absolute turn in degrees
void oneWheelTurn(double targetAngle, int maxSpeed, char wheel) {

  double initialAngle = getRotation();
  //          <---------------------------|------------------->
  // Negative Degrees (Counterclockwise),    Positive Degrees (Clockwise)
  double relativeAngle = targetAngle - getRotation();

  double targetRange = 2; // Distance from the desired angle that is allowed
  double error = relativeAngle; // Distance from the desired range
  double progress;              // Degrees the robot has turned already
  double minSpeed = 5; // Lowest speed the motors will go; Turning is generally more precise when lower, but slower.
  double deaccelRate = 2.5;//3; // 2.4

  double speed; // Actual speed value of the motors

  maxSpeed = keepInRange(maxSpeed, 0, 100);

  while (fabs(error) > targetRange) {
    progress = getRotation() - initialAngle;
    error = progress - relativeAngle;

    // Speed starts at maximum and approaches minimum as the gyro value
    // approaches the desired angle. It deccelerates for precision.
    speed = fabs(error / relativeAngle); // Speed is absolute valued so that it can be kept in range properly
    speed *= maxSpeed * deaccelRate;
    speed = keepInRange(speed, minSpeed, maxSpeed);
    
    // Keep speed either above maxSpeed or below negative maxSpeed so it's fast enough to overcome friction

    if (error > 0) {
      if (wheel == 'L')
        leftDrive(-speed);
      if (wheel == 'R')
        rightDrive(speed);
    } else {
      if (wheel == 'L')
        leftDrive(speed);
      if (wheel == 'R')
        rightDrive(-speed);
    }

    /*controllerPrim.Screen.clearScreen();
    task::sleep(10);
    controllerPrim.Screen.setCursor(1, 1);
    task::sleep(10);
    controllerPrim.Screen.print("%f", getRotation());*/

     Brain.Screen.printAt(1, 120, "Desired angle: %.2f, Relative angle: %.2f",
     degrees, relativeAngle);
     Brain.Screen.printAt(1, 140, "Speed: %.2f, Error: %.2f", speed, error);
     Brain.Screen.printAt(1, 180, "Gyro: %.2f", getRotation());
  }

  stopBase();
  task::sleep(15);
}



// Goes forward and turns to a certain angle at the same time
void curveTurn(double inches, double targetAngle, int maxFwdSpeed, int maxTurnSpeed) {

  clearEncoders();
  maxFwdSpeed = keepInRange(maxFwdSpeed, 0, 100);
  // MinSpeed is the lowest speed the motors will go. Turning is more precise when lower but it can be less fluid.
  // MinSpeed ensures the speed is enough to overcome friction. It replaces the I in PID.
  const double minFwdSpeed = 2; 
  double fwdSpeed;

  double targetFwdDistance = inchesToTicks(inches);  // How far the robot should travel
  double targetFwdRange = 2;                         // Distance from the desired distance the robot has to be to stop.
  double fwdError = targetFwdDistance;               // Current distance from the desired distance

  // Acceleration rates: replacement for D in PID, changes rate of speed up/slow down
  // Gradually speeds up for the first half, slows down for second half
  const double fwdAccelRate = 30;    // Speed multiplier while speeding up (slope at start).
  const double fwdDeaccelRate = 12;  // Speed multiplier while slowing down (slope at end). Starts slowing down later when higher


  double initialAngle = getRotation();
  // positive degrees == right; negative degrees == left // actually this is
  // wrong cuz counter-clockwise (left) is positive but go off
  double relativeAngle = targetAngle - getRotation();

  maxTurnSpeed = keepInRange(maxTurnSpeed, 0, 100);
  double minTurnSpeed = 5; // Lowest speed the motors will go; Turning is generally more precise when lower, but slower.
  double turnSpeed; // Actual speed value of the motors
  
  double targetAngleRange = 2; // Distance from the desired angle that is allowed
  double turnError = relativeAngle; // Distance from the desired range
  double turnProgress;              // Degrees the robot has turned already
  double turnDeaccelRate = 2.5;
 
  // fabs is absolute value for floating point numbers (float)
  while (fabs(turnError) > targetAngleRange && fabs(fwdError) > targetFwdRange) {

    fwdError = targetFwdDistance - encoderAverage(); // Error = desired - actual
    // First half: accelerate to max. Second half, deccelerate to min
    if (fabs(fwdError) > fabs(targetFwdDistance/2))
    {
      fwdSpeed = (1 - (fabs(fwdError) / fabs(targetFwdDistance))) * fwdAccelRate; // Speeds up as error decreases
    }
    else
    {
      fwdSpeed = (fabs(fwdError) / fabs(targetFwdDistance)) * fwdDeaccelRate;  // Slows down as error increases
    }
    fwdSpeed = fabs(fwdSpeed);
    fwdSpeed *= maxFwdSpeed;
    fwdSpeed = keepInRange(fwdSpeed, minFwdSpeed, maxFwdSpeed);

    // Go forward if positive error and backwards if negative error
    if (fwdError < 0)
    {
      fwdSpeed *= -1;
    }


    turnProgress = getRotation() - initialAngle;
    turnError = turnProgress - relativeAngle;

    // Speed starts at maximum and approaches minimum as the gyro value
    // approaches the desired angle. It deccelerates for precision.
    turnSpeed = fabs(turnError / relativeAngle); // Speed is absolute valued so that it can be kept in range properly
    turnSpeed *= maxTurnSpeed * turnDeaccelRate;
    turnSpeed = keepInRange(turnSpeed, minTurnSpeed, maxTurnSpeed);
    
    if (turnError < 0)
    {
      turnSpeed *= -1; // Make turnspeed negative to turn the other way if error is negative
    }
    // Sum up speeds to move forward and turn at same time
    leftDrive(fwdSpeed + turnSpeed); // Turn left when turnSpeed is -
    rightDrive(fwdSpeed - turnSpeed); // Turn right when turnSpeed is +

    Brain.Screen.printAt(1, 60, "Fwd Target Dist: %.2f in", ticksToInches(targetFwdDistance));
    Brain.Screen.printAt(1, 80, "Fwd Progress:    %.2f in", ticksToInches(encoderAverage()));
    Brain.Screen.printAt(1, 100,"Fwd Error:       %.2f ticks, %.2f \"inches\"", fwdError, ticksToInches(fwdError));
    Brain.Screen.printAt(1, 120,"Fwd Speed:       %.2f %", fwdSpeed);

    Brain.Screen.printAt(1, 160, "Desired angle: %.2f,  Relative angle: %.2f", degrees, relativeAngle);
    Brain.Screen.printAt(1, 180, "Turn Speed: %.2f,  Turn Error: %.2f", turnSpeed, turnError);
    Brain.Screen.printAt(1, 200, "Gyro: %.2f", getRotation());
  }

  stopBase();
  task::sleep(15);
}



/*
// Stay moving in the same direction.
// Set desiredDegree, then correctionL and correctionR are continuously updated in a separate task/thread
// Add correctionL and correctionR to corresponding base side speed for functions you want to drive straight
int driveStraight()
{
  while( true )
  {
    double Error = fabs( desiredDegree - getRotation()); // Error is desired - actual value
    double Derivative = KPC * exp(-KDC * Error); // Set the Derivative equal to exponential of the error (e^error)
    double  Proportion = 200.0 / (1 + Derivative) - 4; // Sets Proportion to the desired speed over the derivative

    cSpeed = (Proportion * 1.1);
    // Make turning speed a little faster than proportion

    if (Error != 0) // TODO: Add range for the error to be within (probably +- 1 degree)
    {
      // If error is not completely eliminated, make sure speed is at least 1 (or -1) to ensure speed is enough to overcome friction/weight of robot.
      cSpeed += 1 ;
    }
    Brain.Screen.printAt(140, 200, "Rotation: %.2f deg", getRotation());
    if (getRotation() < desiredDegree)
    {
      correctionL = cSpeed;
      correctionR = -cSpeed;
    }
    else if (getRotation() > desiredDegree )
    {
      correctionL = -cSpeed;
      correctionR = cSpeed;
    }
    else
    {
      correctionL = 0;
      correctionR = 0;
    }

    task::sleep(20);
    //Brain.Screen.printAt(0, 120, "Gyro: %.2f", Gyro1.value(rotationUnits::deg));
  }
  return(0);
}
*/

/*
// Stay moving in the same direction and on the line.
int driveStraightLine()
{
  while( true )
{
double turnError = fabs( desiredDegree - Gyro1.value(rotationUnits::deg) );
    double xError = fabs ( desiredX - xTracker.rotation(rotationUnits::deg) );

    // Not sure if this is the way to do it, but the error is the average of the
turn error and x error to account for both angle and horizontal position. double
Error = (turnError + xError)/2;

double Derivative = KPC * exp(-KDC * Error); //Sets the Derivative equal to
exponential of the error double  Proportion = 200.0 / (1 + Derivative)-4; //sets
Proportion to the desired speed over the derivative cSpeed = Proportion*1.34;
        //Brain.Screen.printAt(140, 55, "Gyro: %.3f",
Gyro1.value(rotationUnits::deg)); if( Gyro1.value(rotationUnits::deg) <
desiredDegree )
    {
correctionL = cSpeed;
correctionR = -cSpeed;
}
else if ( Gyro1.value(rotationUnits::deg) > desiredDegree )
    {
correctionL = -cSpeed;
correctionR = cSpeed;
}
else
    {
correctionL = 0;
correctionR = 0;
}
        task::sleep(20);
        //Brain.Screen.printAt(0, 120, "Gyro: %.2f",
Gyro1.value(rotationUnits::deg));
}
    return(0);
}*/




// PID FUNCTIONS ////////////////


void forwardPID(float targetInches, float maxPower, float msTimeout) {
  float change = 1.34;
  float Kp = 0.3031; // you need to tune these value manually. 0.305
  float Ki = 0.008;// //.03; // they are just arbitrary constants so you need to
                     //test     0.0145
  float Kd = 0.53231; //.18922175;    //until they work. 0.529

  float error =
      inchesToTicks(targetInches * change) - encoderAverage(); // desired - actual
  float lastError;
  float integral;
  float derivative;

  float integralPowerLimit =
      40 / Ki; // little less than half power in pct (percent)
  float integralActiveZone = 15;
  // explaining integral active zone
  // area where proportion is effective is represented with >>/<<
  // area where proportion is not strong enough to move base rep with ----
  // 0 is the target where error is equal to 0
  // only want integral during ineffective zone to prevent windup of power
  //>>>>>>>>>>>>-----------0---------------<<<<<<<<<<<<<<<<

  float exitThreshold = 0.5; // Exit loop when error is less than this
  float finalPower;

  clearEncoders();
  Brain.resetTimer();

  while (fabs(error) > exitThreshold &&
         Brain.timer(vex::timeUnits::msec) < msTimeout) {
    // Brain.Screen.printAt(140, 85,"ROTATION: %.3f deg", getRotation());
    error = inchesToTicks(targetInches * change) - encoderAverage();

    if (fabs(error) < integralActiveZone && error != 0) {
      integral = integral + error;
    } else {
      integral = 0;
    }
    integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit);

    derivative = error - lastError;
    lastError = error;
    /*
     if (error == 0)
     {
       derivative = 0;
     }*/

    finalPower = ((Kp * error) + (Ki * integral) + (Kd * derivative));
    finalPower = keepInRange(finalPower, -maxPower, maxPower);

    leftDrive(finalPower);
    rightDrive(finalPower);
    Brain.Screen.printAt(140, 25, "P: %.2f, I: %.2f, D: %.2f", (Kp * error),
                         (Ki * integral), (Kd * derivative));
    Brain.Screen.printAt(
        140, 50, "Dist: %.2f, Error: %.2f", ticksToInches(encoderAverage()),
        error - encoderAverage());
    vex::task::sleep(40);
  }
  stopBase();
}

void backwardPID(float targetInches, float maxPower, float msTimeout) {
  float change = 1.2;
  float Kp = 0.303; // you need to tune these value manually. 0.305
  float Ki = 0.008;// //.03; // they are just arbitrary constants so you need to
                     //test     0.0145
  float Kd = 0.53232; //.18922175;    //until they work. 0.529

  float error =
      inchesToTicks(-targetInches * change) - encoderAverage(); // desired - actual
  float lastError;
  float integral;
  float derivative;

  float integralPowerLimit =
      40 / Ki; // little less than half power in pct (percent)
  float integralActiveZone = 15;
  // explaining integral active zone
  // area where proportion is effective is represented with >>/<<
  // area where proportion is not strong enough to move base rep with ----
  // 0 is the target where error is equal to 0
  // only want integral during ineffective zone to prevent windup of power
  //>>>>>>>>>>>>-----------0---------------<<<<<<<<<<<<<<<<

  float exitThreshold = 0.5; // Exit loop when error is less than this
  float finalPower;

  clearEncoders();
  Brain.resetTimer();

  while (fabs(error) > exitThreshold &&
         Brain.timer(vex::timeUnits::msec) < msTimeout) {
    // Brain.Screen.printAt(140, 85,"ROTATION: %.3f deg", getRotation());
    error = inchesToTicks(-targetInches * change) - encoderAverage();

    if (fabs(error) < integralActiveZone && error != 0) {
      integral = integral + error;
    } else {
      integral = 0;
    }
    integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit);

    derivative = error - lastError;
    lastError = error;
    /*
     if (error == 0)
     {
       derivative = 0;
     }*/

    finalPower = ((Kp * error) + (Ki * integral) + (Kd * derivative));
    finalPower = keepInRange(finalPower, -maxPower, maxPower);

    leftDrive(finalPower);
    rightDrive(finalPower);
    Brain.Screen.printAt(140, 25, "P: %.2f, I: %.2f, D: %.2f", (Kp * error),
                         (Ki * integral), (Kd * derivative));
    Brain.Screen.printAt(140, 50, "Dist: %.2f, Error: %.2f",
                         ticksToInches(encoderAverage()), ticksToInches(error));
    vex::task::sleep(40);
  }
  stopBase();
}

void turnPID(float target, float maxPower, float msTimeout) {
  float change = 1.0;
  /*
    float Kp = 0.5105;   //getting to target
    float Ki = 0.00825; // increases speed (builds up over time) before: 0.008
    float Kd = 0.0111;*/    //slow down

  /*float Kp = 0.6;    // 0.508497;
  float Ki = 0.0001; // 0.007;
  float Kd = 0.0504; // 051;//.09;*/

  // new constants for Kalahari
  /*float Kp = 0.48;       // 0.508497;
  float Ki = 0.005;//75;//19;      // 11; //0.007;
  float Kd = 0.499;//3; // 0.0504;//051;//.09;*/
  
  float Kp = 0.6;       // 0.508497;
  float Ki = 0.03;//75;//19;      // 11; //0.007;
  float Kd = 0.1;//3; // 0.0504;//051;//.09;

  desiredDegree = target;
  float error = (target * change) - getRotation();
  float lastError;
  float integral;
  float derivative;

  float integralPowerLimit =
      40 / Ki;                   // little less than half power in pct (percent)
  float integralActiveZone = 15; // degrees b/c its a gyro turn doesnt use ticks
  // explaining integral active zone
  // area where proportion is effective is represented with >>/<<
  // area where proportion is not strong enough to move base rep with ----
  // 0 is the target where error is equal to 0
  // only want integral during ineffective zone to prevent windup of power
  //>>>>>>>>>>>>-----------0---------------<<<<<<<<<<<<<<<<

  float exitThreshold = 0.75; // Exit loop when error is less than this
  float finalPower;

  clearEncoders();
  Brain.resetTimer();

  while (fabs(error) > exitThreshold &&
         Brain.timer(vex::timeUnits::msec) < msTimeout) {
    Brain.Screen.printAt(140, 95, "ROTATION: %.3f deg", getRotation());
    error = (target * change) - getRotation();

    if (fabs(error) < integralActiveZone && error != 0) {
      integral = integral + error;
    } else {
      integral = 0;
    }
    integral = keepInRange(integral, -integralPowerLimit, integralPowerLimit);

    derivative = error - lastError;
    lastError = error;
    /*
     if (error == 0)
     {
       derivative = 0;
     }*/

    finalPower = ((Kp * error) + (Ki * integral) + (Kd * derivative));
    finalPower = keepInRange(finalPower, -maxPower, maxPower);

    leftDrive(finalPower);
    rightDrive(-finalPower);
    // Brain.Screen.printAt(140, 25,"P: %.2f, I: %.2f, D: %.2f", (Kp * error),
    // (Ki * integral), (Kd * derivative));
    Brain.Screen.printAt(140, 65, "P: %.2f, I: %.2f, D: %.2f", (Kp * error),
                         (Ki * integral), (Kd * derivative));
    Brain.Screen.printAt(140, 45, "INERTIAL: %.2f", Inertial.value());
    Brain.Screen.printAt(140, 25, "INERTIAL2: %.2f", getRotation());

    vex::task::sleep(40);
  }
  stopBase();
}



// MISC MOVEMENT FUNCTIONS ///////////////////

void forkliftDown()
{
  LFORKLIFT.set(1);
  RFORKLIFT.set(1);
}

void forkliftUp()
{
  LFORKLIFT.set(0);
  RFORKLIFT.set(0);
}

void toggleForklift()
{
  if (LFORKLIFT.value())
    {
      forkliftUp();
    }
    else
    {
      forkliftDown();
    }
}


void clampDown()
{
  LCLAMP.set(1);
  RCLAMP.set(1);
}

void clampUp()
{
  LCLAMP.set(0);
  RCLAMP.set(0);
}

void toggleClamp()
{
  if (LCLAMP.value())
    {
      clampUp();
    }
    else
    {
      clampDown();
    }
}


void shieldOff()
{
  SHIELD.set(0);
}

void shieldOn()
{
  SHIELD.set(1);
}

void toggleShield()
{
  if (SHIELD.value() == 1)
  {
    shieldOff();
  }
  else
  {
    shieldOn();
  }
}


void conveyorOn()
{
  CONVEYOR.spin(fwd, 100, pct);
}

void conveyorOff()
{
  CONVEYOR.stop();
}


void toggleConveyor()
{
  conveyorOnDriver = !conveyorOnDriver;
}

/*
void toggleConveyor()
{
  if (CONVEYOR.isSpinning())
  {
    ConveyorOff();
    Brain.Screen.printAt(60, 80, "not spinning: %d", i);
  }
  else
  {
    ConveyorOn();
    Brain.Screen.printAt(60, 60, "spinning: %d", i);
  }
}

void toggleConveyorReverse()
{
  if (CONVEYOR.isSpinning())
  {
    ConveyorOff();
    Brain.Screen.printAt(60, 80, "Reverse not spinning: %d", i);
  }
  else
  {
    ConveyorReverse();
    Brain.Screen.printAt(60, 60, "Reverse spinning: %d", i);
  }
}*/



/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  //shieldOn();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {}



int flipoutDelay()
{
  task::sleep(400);
  forkliftDown();
  return 0;
}

int delayLift()  
{
  task::sleep(650);
  ARM.startRotateTo(750, deg, 80, velocityUnits::pct);
  return 0;
}


int clampDelayAuton()
{
  while(ticksToInches(encoderAverage()) > -20) {
    task::sleep(5);
  }
  clampUp();
  return 0;
}


int deployShieldDelay()
{
  while(ticksToInches(encoderAverage()) < 22) {
    task::sleep(5);
  }
  shieldOff();
  return 0;
}


int dropMogoEndMove()
{
  task::sleep(14600);
  /*leftDrive(-100);
  rightDrive(-100);*/
  conveyorOff();
  task::sleep(200);
  forkliftUp();
  /*leftDrive(100);
  rightDrive(100);*/
  return 0;
}


void leftSide()
{
  // grab closest yellow mogo
  shieldOn();
  clampUp();
  
  //clearEncoders();
  //task a(deployShieldDelay);
  forwardInchesSpeedUp(28, 100);
  //task::sleep(75);
  clampDown();
  
  //forwardInchesFast(5, 100);
  task::sleep(75);
  leftDrive(-100);
  rightDrive(-100);
}

// shift robot left and tilt right a little
void rightSide()
{
  shieldOn();
  // grab closest yellow mogo
  clampUp();
  forwardInchesSpeedUp(26.5, 100);
  //task::sleep(75);
  clampDown();
  
  //forwardInchesFast(5, 100);
  task::sleep(75);
  leftDrive(-100);
  rightDrive(-100);
}


void leftSideRings()
{
  // grab closest yellow mogo
  shieldOn();
  clampUp();
  /*forwardInchesFast(30, 100);
  task::sleep(75);
  clampDown();
  task::sleep(75);*/
  
  //clearEncoders();
  //task a(deployShieldDelay);

  
  forwardInchesSpeedUp(28, 100);
  //task::sleep(75);
  clampDown();
  forwardInchesFast(5, 100);
/*
  smoothSpeed(4, 10, 50);
  smoothSpeed(4, 50, 100);
  smoothSpeed(20, 50, 100);*/

  // bring close yellow mogo and drop it in zone
  //clearEncoders();
  //task b(clampDelayAuton);
  forwardInchesTimed(-39, 100, 3000);
  
  shieldOff();
  forwardInches(2, 28);

  // turn towards ramp mogo and push it up ramp.
  gyroTurn(-100, 40);
  ARM.startRotateTo(1000, deg, 80, velocityUnits::pct);
  forwardInches(-9, 35);
  forkliftDown();
  //gyroTurn(75-180, 30);
  task::sleep(50);
  conveyorOn();
  // forwardInches(21, 20);
  forwardInchesFast(19, 22);
  forwardInchesFast(-4, 35);
  forkliftUp();

/*
  forwardInches(20, 30);
  forwardInches(-20, 30);
  forwardInches(20, 30);
  forwardInches(-20, 30);
  forwardInches(20, 30);
  forwardInches(-20, 30);
  //forwardInches(20, 30);
  */
}


// shift robot left and tilt right a little
void rightSideRings()
{
  // grab closest yellow mogo
  clampUp();
  shieldOn();
  /*forwardInchesSpeedUp(27, 100);
  //task::sleep(75);
  clampDown();
  forwardInchesFast(5, 100);
  //task::sleep(75);

  // bring close yellow mogo and drop it in zone
  forwardInches(-24, 40);*/
  
  forwardInchesSpeedUp(26, 100);
  //task::sleep(75);
  clampDown();
  
  //forwardInchesFast(5, 100);
  task::sleep(75);

  // bring close yellow mogo and drop it in zone
  forwardInchesFast(-24, 100); //-23 dist
  //inertialForward(-24, 100);


  task::sleep(75);
  clampUp();
  //gyroTurn(-45, 27);
  task::sleep(75);
  //conveyorOn();
  
  gyroTurn(-98, 34);   // -95 too right
  forwardInches(-8, 34); // into wall
  task::sleep(300);
  forkliftDown(); // forkliftDown means bite down, not place down

  forwardInches(3, 18); // line up with rings, 6.75, 6.25
  ARM.startRotateTo(300, deg);
  gyroTurn(-1.75, 25); //-173 deg turn towards rings, -179 too left
  conveyorOn();
  forwardInches(25, 19);
  //gyroTurn(15, 28);
  forwardInchesFast(-28, 100);
  /*
  task::sleep(25);
  ARM.rotateFor(-800, rotationUnits::deg, 75, velocityUnits::pct);
  //forkliftUp();
  task::sleep(25);
  conveyorOn();
  forwardInches(4, 23); // line up with rings
  gyroTurn(-172, 25); //-173 deg turn towards rings, -179 too left
  forwardInches(-24, 30);
  
  task::sleep(400);
  forwardInches(30, 55);*/
}




void rightCenter()
{
  clampUp();
  shieldOn();
  // bring center yellow mogo and drop it in zone
  forwardInchesSpeedUp(34.5, 100);
  clampDown();
  forwardInchesFast(5, 100);
  // bring close yellow mogo and drop it in zone
  leftDrive(-100);
  rightDrive(-100);
}





int raiseArmDelay()
{
  ARM.startRotateTo(200, deg, 75, velocityUnits::pct);
  while(ticksToInches(encoderAverage()) < 20) {
    task::sleep(5);
  }
  ARM.rotateTo(0, deg, 75, velocityUnits::pct);

  return 0;
}


void leftCenter()
{
  clampUp();
  //shieldOn();
  // bring center yellow mogo and drop it in zone
  
  clearEncoders();
  task a(raiseArmDelay);
  
  conveyorOn();
  forwardInchesSpeedUp(37, 100);
  clampDown();
  forwardInchesFast(5, 100);
  // bring close yellow mogo and drop it in zone
  leftDrive(-100);
  rightDrive(-100);
}



// pretend to go middle
void leftSideFake()
{
  // grab closest yellow mogo
  //shieldOn();
  gyroTurn(-22, 30);
  clampUp();
  
  //clearEncoders();
  //task a(deployShieldDelay);
  forwardInchesSpeedUp(28, 100);
  //task::sleep(75);
  clampDown();
  
  //forwardInchesFast(5, 100);
  task::sleep(75);
  leftDrive(-100);
  rightDrive(-100);
}


void rightSideWP()
{
  forkliftUp();
  forwardInches(-15, 30);
  forkliftDown();
  task::sleep(100);
  conveyorOn();
  gyroTurn(90, 30);
  //forwardInches(50, 75);
}


int dropMogoDelay()
{
  /*while(ticksToInches(encoderAverage()) < 45) {
    task::sleep(5);
  }
  conveyorOff();*/
  while(ticksToInches(encoderAverage()) < 50) {
    task::sleep(5);
  }
  conveyorOff();
  task::sleep(250);
  forkliftUp();
  return 0;
}


int dropMogoEnd()
{
  task::sleep(14750);
  conveyorOff();
  task::sleep(150);
  forkliftUp();
  return 0;
}


void fullRow()
{
  task a(dropMogoEnd);
  // Pick up mogo
  forkliftUp();
  forwardInchesFast(-9, 32);
  forkliftDown();

  // Score rings on mogo
  task::sleep(500);
                        //forwardInches(-5, 25);
  gyroTurn(88, 30);
  conveyorOn();
  ARM.startRotateTo(890, deg, 50, velocityUnits::pct);

  clearEncoders();
  task b(dropMogoDelay); // drop the mobile goal after some encoder value is reached
  forwardInchesFast(65, 100); // go across home row

  //forkliftUp();
  gyroTurn(180, 30);
  forwardInchesFast(-26, 40, 1600);  
  forwardInches(2, 22);
  gyroTurn(92, 30);

  forwardInchesSpeedUp(-14, 30);
  forkliftDown();
  conveyorOn();

  forwardInches(21, 22);
  /*task::sleep(100);
  conveyorOff();
  forkliftUp();*/
  //forwardInchesFast(5, 35);
  /*forwardInches(12, 30);
  forwardInches(-12, 30);
  forwardInches(12, 30);
  forwardInches(-12, 30);
  //forwardInches(12, 30);*/
  /*forwardInchesFast(15, 30);
  forwardInchesFast(-15, 30);*/

  //forwardInchesSpeedUp(23, 50); // don't overcorrect due to speed
  // slow to pick up rings
  //forwardInchesContinued(50, 20); // remember the previous encoder values to correct the combined end position of both movements
  /*
  conveyorOff();
  task::sleep(50);
  forkliftUp();
  forwardInches(16, 40);
  gyroTurn(143, 32); // 135 too little

  forwardInches(-23, 45);
  forkliftDown();
  task::sleep(75);
  gyroTurn(90, 30);
  */

}



// LEFT SIDE
void fullRowCenter()
{
  
  /*
  clampUp();
  shieldOn();
  // bring center yellow mogo and drop it in zone
  forwardInchesSpeedUp(34.5, 100);
  clampDown();
  forwardInchesFast(5, 100);
  // bring close yellow mogo and drop it in zone
  //leftDrive(-100);
  //rightDrive(-100);

  forwardInches(20, 65);
  
  task a(dropMogoEnd);
  // Pick up mogo
  forkliftUp();
  forwardInchesFast(-9, 32);
  forkliftDown();

  // Score rings on mogo
  task::sleep(500);
                        //forwardInches(-5, 25);
  gyroTurn(88, 30);
  conveyorOn();
  ARM.startRotateTo(890, deg, 50, velocityUnits::pct);

  clearEncoders();
  task b(dropMogoDelay); // drop the mobile goal after some encoder value is reached
  forwardInchesFast(65, 100); // go across home row

  //forkliftUp();
  gyroTurn(180, 30);
  forwardInchesFast(-26, 40, 1600);  
  forwardInches(2, 22);
  gyroTurn(92, 30);

  forwardInchesSpeedUp(-14, 30);
  forkliftDown();
  conveyorOn();

  forwardInches(21, 22);*/
}



int dropMogoDelaySkills()
{
  while(ticksToInches(encoderAverage()) < 25) {
    task::sleep(5);
  }
  forkliftUp();
  return 0;
}


////// SKILLS ////////////////////////////////////////


void setonSkills()
{
  forwardInches(2.15, 30); // go towards mogo on platform. // used to be 2 in
    task::sleep(40);
  clampDown();
    task::sleep(200);
  //ARM.rotateFor(-800, deg, 75, velocityUnits::pct);
  //forkliftUp(); don't
  gyroTurn(8, 18);
  task::sleep(40);
  forwardInches(-6, 26);
  task::sleep(40);
  gyroTurn(-60, 30); // WEIRD ONE, turns between neutral mogos used to be -64 deg

  forwardInches(61, 60);
  task::sleep(25);
  forwardInches(-1, 14);
  clampUp();
  task::sleep(25);
  forwardInches(-3.5, 26);
  gyroTurn(-161, 34); // turn towards blue win point mogo (WEIRD ONE II)
  task::sleep(25);
  forwardInches(25.75, 40);

  task::sleep(75);
  clampDown(); 
  task::sleep(50);

  gyroTurn(-266, 35);
  forwardInches(38, 55); // go 'cross field
  gyroTurn(-255, 30);
  task::sleep(25);
  clampUp();
  task::sleep(60);
  
  // zig zag to center neutral mogo
  forwardInches(-38, 55);
  gyroTurn(-275, 30);
  forwardInches(32, 50); // push tall one
  gyroTurn(-257, 30);
  forwardInches(-33, 50);

  gyroTurn(-340, 27); // turn and get on outside of neut mogo
  task::sleep(50);
  forwardInches(22, 40);
  task::sleep(50);
  gyroTurn(-240, 28);
  task::sleep(50);
  forwardInches(32, 50);

  // back up, turn 90ish to the left,  pick up red mogo on blue win point line
  forwardInches(-3, 26);
  gyroTurn(-340.5, 28);
  forwardInches(20.1, 32);//used to be 21 in sus stops at wall

  task::sleep(50);
  clampDown();
  task::sleep(100);

  // drag red mogo to red zone
  forwardInches(-4, 24);
  gyroTurn(-442, 28);
  forwardInches(48, 60); // across field

  task::sleep(25);
  clampUp();  // release red mogo
  task::sleep(50);
  forwardInches(-3, 25);
  gyroTurn(-476, 31);// Towards platform,  Used to be -470  -360-112  maybe -478
  forwardInches(17.5,30); 

  task::sleep(50);
  clampDown(); // pick up blue mogo on platform
  task::sleep(75);

  forwardInches(-6, 20);
  task::sleep(25);
  gyroTurn(-475+45, 32);// Used to be -560
  task::sleep(75);
  forwardInches(-55, 55);
}



// BOTB Skills
void skillsPlatform()
{
  //hookDown();
  forkliftDown();
  task::sleep(75);
  ARM.rotateFor(-25, rotationUnits::deg, 25, velocityUnits::pct);
  ARM.resetRotation();

  ARM.rotateFor(650, deg, 75, velocityUnits::pct);
    task::sleep(25);
  forwardInches(-10,28);
  ARM.startRotateFor(-650, deg, 75, velocityUnits::pct);

  task::sleep(75);
  gyroTurn(10, 18); // turn and back up
  forwardInches(10.5, 26);
    task::sleep(100);
  gyroTurn(103, 26); // WEIRD ONE, turn towards mogo used to be 104 deg
  //gyroTurn(103, 23); // WEIRD ONE, turn towards mogo used to be 104 deg, 99, 27
// UNCOMMENT
// Pick up close neutral mogo
  //task::sleep(275); waiting may jerk too mucuh
  task::sleep(50);
  forwardInches(28.5, 28);
  task::sleep(50);
  
  clampDown();
  task::sleep(150);
  //ARM.startRotateTo(800, deg, 75, velocityUnits::pct);
  //task::sleep(100);
  
  gyroTurn(118, 22); // turn towards center of red plat, used to be 119.5 deg, 113
  task::sleep(50);

  //forwardInches(8, 43); //33.25 in
  ARM.rotateTo(750, deg, 80, velocityUnits::pct); // lift arm to drop mogo while moving (may need to stop in between)
  //task liftMid(delayLift); // lift while going forward
  //ARM.startRotateTo(800, deg, 75, velocityUnits::pct);
  forwardInches(30, 42); //33.25 in, 30 in
  task::sleep(50);
  //forwardInches(33.25, 45); //33.25 in
  //gyroTurn(97, 20);
  //task::sleep(10);
  gyroTurn(92.5, 24); // turn towards platform, used to be 94 deg
  task::sleep(100);
  
  // drop in center of red platform.
  forwardInches(4, 22);
  task::sleep(100);
  ARM.rotateFor(-175, deg, 70, velocityUnits::pct);
  task::sleep(50);
  clampUp(); // let go,
  task::sleep(175);
  ARM.rotateTo(750, deg, 70, velocityUnits::pct);
  task::sleep(50);
  gyroTurn(86, 24); // turns to line up mogo with back
  task::sleep(75);
  forwardInches(-17, 25); // pick up mid neutral
  task::sleep(100);
  ARM.rotateTo(0, deg, 70, velocityUnits::pct);
  task::sleep(100);
  forwardInches(-20, 26);
  ARM.rotateTo(7-50, deg, 26, velocityUnits::pct);
  forwardInches(5, 26);
  gyroTurn(0, 26);
  //gyroTurn(-25, 26); // turn towards dropped red mogo (sketchy). hits rings
  //forwardInches(26, 34); // goes extra forward to conform to mogo angle
  task::sleep(50);
  /*clampDown();
  task::sleep(75);

  forwardInches(-19, 37);
  task::sleep(50);
  gyroTurn(90, 25);

  ARM.rotateTo(800, deg, 75, velocityUnits::pct);
  forwardInches(23, 40);

  // drop red mogo on left of tower
  task::sleep(75);
  ARM.rotateFor(-175, deg, 70, velocityUnits::pct);
  task::sleep(50);
  clampUp(); // let go,
  task::sleep(175);
  ARM.rotateTo(750, deg, 70, velocityUnits::pct);
  task::sleep(100);
  forwardInches(-15, 30);
*/
  /*task::sleep(120);
  ARM.rotateFor(300, deg, 60, velocityUnits::pct);  
  ARM.startRotateTo(0, deg, 20, velocityUnits::pct); // go down as its moving back
  forwardInches(-10, 26);*/
}



// left side, straight setup
void greenBaySkills()
{
  // GET LEFT RED
  forkliftUp();
  task::sleep(75);
  forwardInches(-2, 15);
  task::sleep(75);
  forkliftDown();
  task::sleep(75);
  conveyorOn();
  //forwardInches(1, 20);
  forwardInches(3, 16);

  task::sleep(75);
  gyroTurn(15, 18); // turn and back up
  task::sleep(100);
  forwardInches(3, 16);
    task::sleep(100);
  gyroTurn(97, 26);
  
  //gyroTurn(87, 30);
  forwardInches(29, 42);
  conveyorOff();
  task::sleep(75);
  clampDown();
  task::sleep(100);
  gyroTurn(112, 17);
  task::sleep(100);
  ARM.startRotateTo(1050, deg, 75, velocityUnits::pct);

  //clearEncoders();
  //task t1(dropMogoDelaySkills); // causes friction from placing while moving
  
  // DROP LEFT RED
  forwardInches(19, 40);
  task::sleep(150);
  forkliftUp();
  task::sleep(250);
  forwardInches(10, 32);
  task::sleep(100);
  
  // SCORE CLOSE NEUTRAL -> RED PLAT
  ARM.rotateTo(650, deg, 35, velocityUnits::pct);
  gyroTurn(109, 14);
  task::sleep(100);
  clampUp();
  task::sleep(150);
  ARM.rotateTo(700, deg, 35, velocityUnits::pct);

  // SCORE LEFT RED
  forwardInches(-5, 26);
  gyroTurn(-47, 37); // sketchy turn
  ARM.rotateTo(-50, deg, 75, velocityUnits::pct);
  forwardInches(15, 30);
  clampDown();
  task::sleep(100);
  gyroTurn(120, 39);
  ARM.startRotateTo(800, deg, 60, velocityUnits::pct);
  forwardInches(23, 30);
  task::sleep(150);
  ARM.rotateTo(650, deg, 60, velocityUnits::pct);
  clampUp();
  task::sleep(150);
  ARM.rotateTo(800, deg, 60, velocityUnits::pct);


  // GET RIGHT BLUE
  forwardInches(-8.5, 18);
  gyroTurn(180, 27);
  ARM.startRotateTo(-50, deg, 30, velocityUnits::pct);
  forwardInches(-20.5, 40); // 35 speed;
  forkliftDown();
  task::sleep(200);
  conveyorOn();

  // PUSH CENTER NEUTRAL -> BLUE ZONE
  forwardInches(15, 30);
  gyroTurn(223, 20); // 226
  forwardInches(20, 38);
  clampDown(); // secure neutral
  forwardInches(20, 50);
  forkliftUp(); // drop right blue
  forwardInches(30, 40);
  clampUp();
  forwardInches(-10, 28);
  gyroTurn(44, 35); // does 180, used to be 46 deg
  forwardInches(20, 35);
  clampDown();
  task::sleep(100);
  gyroTurn(-80, 33); // turn towards blue platform

  
  // SCORE RIGHT BLUE
  ARM.startRotateTo(900, deg, 60, velocityUnits::pct);
  forwardInches(24, 37);
  conveyorOff();
  ARM.rotateTo(700, deg, 40, velocityUnits::pct);
  clampUp();
  task::sleep(100);
  ARM.startRotateTo(900, deg, 60, velocityUnits::pct);

  // GET RIGHT RED
  forwardInches(-8, 24);
  gyroTurn(0, 30);
  task::sleep(50);
  ARM.startRotateTo(-50, deg, 60, velocityUnits::pct);
  forwardInches(-29, 32); // back into mogo against wall
  forkliftDown();
  task::sleep(100);
  conveyorOn();

  // SCORE FAR NEUTRAL + RIGHT RED -> BLUE SIZE
  gyroTurn(51, 30); // 55
  forwardInches(18, 29);
  clampDown();
  task::sleep(100);
  //ARM.startRotateTo(700, deg, 60, velocityUnits::pct);
  forwardInches(12, 36);
  conveyorOff();
  //gyroTurn(0, 30); // turn so right red is in zone if it gets picked up
}



// left side, straight setup
void fairgroundsSkills()
{
  // GET LEFT RED
  forkliftUp();
  task::sleep(75);
  forwardInches(-2, 15);
  task::sleep(75);
  forkliftDown();
  task::sleep(75);
  conveyorOn();
  //forwardInches(1, 20);
  forwardInches(3, 16);

  task::sleep(75);
  gyroTurn(15, 18); // turn and back up
  task::sleep(100);
  forwardInches(3, 16);
    task::sleep(100);
  gyroTurn(97, 26);
  
  //gyroTurn(87, 30);
  forwardInches(29, 42);
  conveyorOff();
  task::sleep(75);
  clampDown();
  task::sleep(100);
  gyroTurn(112, 17);
  task::sleep(100);
  ARM.startRotateTo(1050, deg, 75, velocityUnits::pct);

  //clearEncoders();
  //task t1(dropMogoDelaySkills); // causes friction from placing while moving
  
  // DROP LEFT RED
  forwardInches(19, 40);
  task::sleep(150);
  forkliftUp();
  task::sleep(200);
  forwardInches(10, 35);
  task::sleep(100);
  
  // SCORE CLOSE NEUTRAL -> RED PLAT
  ARM.rotateTo(650, deg, 35, velocityUnits::pct);
  //gyroTurn(109, 14);
  task::sleep(100);
  clampUp();
  task::sleep(150);
  ARM.rotateTo(700, deg, 35, velocityUnits::pct);

  // SCORE LEFT RED
  forwardInches(-5, 26);
  ARM.startRotateTo(-50, deg, 75, velocityUnits::pct);
  gyroTurn(-60, 42); // sketchy turn, -53 too positive
  forwardInches(15, 30);
  clampDown();
  task::sleep(100);
  ARM.rotateTo(50, deg, 75, velocityUnits::pct);
  gyroTurn(120, 42);
  ARM.startRotateTo(850, deg, 60, velocityUnits::pct);
  forwardInches(23, 30);
  task::sleep(150);
  ARM.rotateTo(650, deg, 60, velocityUnits::pct);
  clampUp();
  task::sleep(150);
  ARM.rotateTo(850, deg, 60, velocityUnits::pct);


  // GET RIGHT BLUE
  forwardInches(-8.5, 18);
  gyroTurn(180, 27);
  ARM.startRotateTo(-25, deg, 30, velocityUnits::pct);
  forwardInchesFast(-20.5, 40); // 35 speed;
  forkliftDown();
  task::sleep(150);
  conveyorOn();

  // PUSH CENTER NEUTRAL -> BLUE ZONE
  forwardInches(15, 32);
  gyroTurn(223, 20); // 226
  conveyorOff();
  forwardInches(20, 38);
  clampDown(); // secure neutral
  forwardInchesFast(32, 50); // 20 in
  forkliftUp(); // drop right blue
  forwardInches(18, 40); // 30 in
  clampUp();
  forwardInches(-10, 28);
  gyroTurn(44, 36); // does 180, used to be 46 deg
  forwardInches(10, 28);
  clampDown();
  forwardInches(10, 28);
  task::sleep(100);
  gyroTurn(-80, 34); // turn towards blue platform

  
  // SCORE RIGHT BLUE
  ARM.startRotateTo(900, deg, 80, velocityUnits::pct);
  forwardInches(20, 37);
  ARM.rotateTo(700, deg, 40, velocityUnits::pct);
  clampUp();
  task::sleep(100);
  ARM.startRotateTo(900, deg, 60, velocityUnits::pct);

  // GET RIGHT RED
  forwardInches(-4, 24);
  gyroTurn(0, 30);
  task::sleep(50);
  ARM.startRotateTo(150, deg, 60, velocityUnits::pct);
  forwardInches(-28.5, 32); // back into mogo against wall -29
  forkliftDown();
  task::sleep(100);
  conveyorOn();

  // SCORE FAR NEUTRAL + RIGHT RED -> RED SIDE
  gyroTurn(51, 30); // 55
  forwardInches(18, 30);
  clampDown();
  task::sleep(75);
  //ARM.startRotateTo(700, deg, 60, velocityUnits::pct);
  forwardInchesFast(29, 65);
  conveyorOff();
  conveyorOff();
  //gyroTurn(0, 30); // turn so right red is in zone if it gets picked up
  gyroTurn(0, 38);
  forkliftUp();
  //gyroTurn(0, 30); // turn so right red is in zone if it gets picked up
}


void statesSkills()
{
  // GET LEFT RED
  shieldOff();
  forkliftUp();
  task::sleep(75);
  forwardInches(-2, 16);
  task::sleep(75);
  forkliftDown();
  task::sleep(75);
  conveyorOn();
  //forwardInches(1, 20);
  forwardInches(3, 17);

  task::sleep(75);
  gyroTurn(15, 19); // turn and back up
  task::sleep(100);
  forwardInches(3, 16);
    task::sleep(100);
  conveyorOff();
  gyroTurn(100, 32);
  
  //gyroTurn(87, 30);
  forwardInches(29, 45); // go to neutral mobile goal
  //simpleForward(29, 45);
  task::sleep(75);
  clampDown();
  task::sleep(100);
  gyroTurn(117, 18);
// UNCOMMENT
  ARM.startRotateTo(1050, deg, 80, velocityUnits::pct);
  task::sleep(150);
  forwardInches(28.5, 40); // go into platform
  
  // SCORE CLOSE NEUTRAL -> RED PLAT
  ARM.rotateTo(700, deg, 65, velocityUnits::pct);

  gyroTurn(100, 17);
  //ARM.startRotateTo(575, deg, 45, velocityUnits::pct);
  forwardInches(2, 16);

  //task::sleep(100);
  clampUp();
  task::sleep(50);
  ARM.rotateTo(750, deg, 45, velocityUnits::pct);
  
  // GET RIGHT BLUE
  forwardInches(-4, 23);

  gyroTurn(0, 28);
  forkliftUp();
  task::sleep(75);
  ARM.startRotateTo(-75, deg, 70, velocityUnits::pct);
  
  forwardInchesFast(23.5, 55);
  clampDown();
  //forkliftDown();
  task::sleep(75);
  //conveyorOn();

  // PUSH CENTER NEUTRAL -> BLUE ZONE
  forwardInches(-9, 32);
  // UNCOMMENT
  gyroTurn(40, 24); // 226
 
  //forwardInchesFast(-29, 60); 
  //forkliftDown();
  //forwardInchesFast(-34, 70);
  forwardInchesFast(-64, 100); // push tall neutral into blue zone
  forkliftUp();
  task::sleep(75);
  
  forwardInches(11, 36);
   
  //clampDown();
  task::sleep(100);
  
  gyroTurn(-50, 30); // turn towards blue platform


  // SCORE RIGHT BLUE
  ARM.startRotateTo(900, deg, 75, velocityUnits::pct);
  task::sleep(300);
  forwardInches(10, 30);
  conveyorOff();
  ARM.rotateTo(700, deg, 50, velocityUnits::pct);
  clampUp();
  task::sleep(75);
  ARM.startRotateTo(900, deg, 60, velocityUnits::pct);

  // GET RIGHT RED
  forwardInches(-8, 32);
  gyroTurn(0, 29);
  task::sleep(50);
  ARM.startRotateTo(-50, deg, 60, velocityUnits::pct);
  forwardInchesTimed(-26, 35, 2150); // back into mogo against wall
  forkliftDown();
  task::sleep(50);
  conveyorOn();
  gyroTurn(51, 28);


  // PICK UP FAR NEUTRAL
  ARM.startRotateTo(150, deg, 40, velocityUnits::pct); // lift arm so conveyor can intake rings
  forwardInches(17, 37);
  ARM.rotateTo(-50, deg, 40, velocityUnits::pct); // lower arm to pick up mobile goal
  forwardInches(5, 22);
  clampDown();
  task::sleep(25);
  
  // SCORE FAR NEUTRAL -> BLUE PLATFORM
  ARM.startRotateTo(850, deg, 65, velocityUnits::pct);
  forwardInches(26, 46);
  clampUp();

  gyroTurn(0, 29);
  forwardInches(-2, 18);
  ARM.rotateTo(-50, deg, 75, velocityUnits::pct);
  forwardInches(4, 19);
  clampDown();

  ARM.rotateTo(850, deg, 75, velocityUnits::pct);
  gyroTurn(90, 30);

  clampUp();
}


void test()
{
  relativeTurn(90, 25);
}

void reveal()
{
  
}



/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void draw()
{
  Brain.Screen.setFillColor(color(0, 60, 5)); // green, in rgb `
  Brain.Screen.setPenWidth(0);
  Brain.Screen.drawRectangle(0, 0, 480, 272); // fill entire screen
  

  Brain.Screen.setPenColor(color(255, 235, 0)); // yellow
  Brain.Screen.setPenWidth(100);
  // draw Y shape
  Brain.Screen.drawLine(380, 120, 220, 120);
  Brain.Screen.drawLine(220, 120, 90, 210);
  Brain.Screen.drawLine(220, 120, 90, 30);

  Brain.Screen.setPenColor(color(255, 255, 255)); // white
  //Brain.Screen.setPenColor(color(104, 158, 49)); // light green
}


void usercontrol(void) {
  // User control code here, inside the loop
  draw();
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.


    leftDrive(controllerPrim.Axis3.value() * 1.0);
    rightDrive(controllerPrim.Axis2.value() * 1.0);

    if (controllerPrim.ButtonL1.pressing())
    {
      ARM.spin(fwd, 50, pct);
    }
    else if (controllerPrim.ButtonL2.pressing())
    {
      ARM.spin(reverse, 50, pct);
    }
    else
    {
      ARM.stop(brakeType::hold);
    }

  
    // Reverse override if pressing button
    if (controllerPrim.ButtonB.pressing())
    {
      CONVEYOR.spin(reverse, 100, pct);
    } 
    else
    {
      // If no reverse override, toggle on/off with button X (set up in callbacks)
      if (conveyorOnDriver)
      {
        conveyorOn();
      }
      else
      {
        CONVEYOR.stop();
      }
    }
    // Callbacks for other functions set up in main

    /*
    Brain.Screen.printAt(0, 20, "R: %.2f, L: %.2f", RFBASE.velocity(pct), LFBASE.velocity(pct));
    Brain.Screen.printAt(20, 40, "%.2f deg", getRotation());
    Brain.Screen.printAt(20, 60, "ARM rot: %.2f", ARM.rotation(deg));
    Brain.Screen.printAt(20, 80, "CONVEYOR vel: %.2f", CONVEYOR.velocity(pct));
    Brain.Screen.printAt(20, 100, "LINE: %d, Detected: %d", CLAMPLINE.value(pct), clampMogoDetected());
    */

    //Brain.Screen.printAt(0, 20, "%.1f\%", Brain.Battery.capacity());
    Brain.Screen.printAt(220, 40, "R: %.1f, L: %.1f    ", RFBASE.velocity(pct), LFBASE.velocity(pct));
    Brain.Screen.printAt(220, 60, "Robot rot: %.2f deg  ", getRotation());
    Brain.Screen.printAt(220, 80, "ARM rot: %.1f deg  ", ARM.rotation(deg));

    Brain.Screen.printAt(220, 190, "CONVEYOR vel: %.1f  ", CONVEYOR.velocity(pct));
    Brain.Screen.printAt(220, 210, "CLAMPLINE: %d, Detected: %d    ", CLAMPLINE.value(pct), clampMogoDetected());
    Brain.Screen.printAt(220, 230, "WALLLINE: %d, Detected: %d    ", WALLLINE.value(pct), wallDetected());
  
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}


bool holdMode = false;

void toggleHold(){
  holdMode = !holdMode;

  controllerPrim.Screen.setCursor(1,1);
    controllerPrim.Screen.clearLine();

  if (holdMode == true){
    controllerPrim.Screen.print("Hold");
    LBBASE.setStopping(hold);
    LMBASE.setStopping(hold);
    LFBASE.setStopping(hold);
    RBBASE.setStopping(hold);
    RMBASE.setStopping(hold);
    RFBASE.setStopping(hold);

  } else {
    controllerPrim.Screen.print("Coast");
    LBBASE.setStopping(coast);
    LMBASE.setStopping(coast);
    LFBASE.setStopping(coast);
    RBBASE.setStopping(coast);
    RMBASE.setStopping(coast);
    RFBASE.setStopping(coast);
  }
}



//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  // fairgroundsSkills
  Competition.autonomous(statesSkills);
  Competition.drivercontrol(usercontrol);

  ARM.setStopping(hold);

  // Custom callbacks
  controllerPrim.ButtonR2.pressed(toggleForklift);
  controllerPrim.ButtonR1.pressed(toggleClamp);
  controllerPrim.ButtonLeft.pressed(toggleShield);
  controllerPrim.ButtonDown.pressed(toggleConveyor);
  //controllerPrim.ButtonX.pressed(toggleHold);
  draw();
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
    
    if (Inertial.isCalibrating())
    {
      Brain.Screen.setPenColor(red);
      Brain.Screen.printAt(220, 20, "Calibrating          ");
      Brain.Screen.setPenColor(white);
    }
    else
    {
      Brain.Screen.printAt(220, 20, "Press to recalibrate?");
      if (Brain.Screen.pressing())// and Brain.Screen.xPosition() < width and Brain.Screen.yPosition() < height)
      {
        Inertial.calibrate();
        ARM.resetRotation();
      }
    }
    i++;
  }
}