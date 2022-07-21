#ifndef DRIVEBASEMODULE_H
#define DRIVEBASEMODULE_H

#include <vector>
#include <math.h> 

#include <iostream>
#include <fstream>
#include <string>
#include <list>
#include "Macros.h"

#include <rev/CANSparkMax.h>
#include <frc/Joystick.h>

#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/PIDController.h>
#include <frc/ADIS16448_IMU.h>

#define driverStickPort 0
#define operatorStickPort 1

// Values that are assigned on init 
#define PIDProportional 1
#define PIDIntegral 0
#define PIDDerivative 0

#define motorInitMaxCurrent 100 // The initial max current setting
#define motorInitRatedCurrent 60 // The inital rated current settings
#define motorInitLimitCycles 2000 // The inital number of allowed ms at peak current

#define lInvert true // Inversion setings for sides
#define rInvert false 

#define xDeadband 0.025
#define yDeadband 0.025

#define centerToWheel 1.041667 //Center of the robot to outer wheel or .994... for to inner wheel or 1.08333
#define PI 3.141592654

class DriveBaseModule{
  float rMotorSetpoint; // Current Motor Setpoints
  float lMotorSetpoint;

  float robotProportional; // PID values, for dynamic assignment
  float robotIntegral;
  float robotDerivative;

  bool pressed = false;
  bool intakeOn = false;
  bool moveFlag = true;

  const double deadband = 1e-5;
  float prevTime;
  float prev_value_speed;
  float prev_value_turn;
  
  typedef struct {
    float speed;
    float sensitivity;

  } inputPoint;
  std::list<inputPoint> inputs;

  //frc::Joystick* driverStick = new frc::Joystick(driverStickPort);
  frc::Joystick* operatorStick = new frc::Joystick(operatorStickPort);

  rev::CANSparkMax* lMotor = new rev::CANSparkMax(lMotorLeaderID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* lMotorFollower = new rev::CANSparkMax(lMotorFollowerID, rev::CANSparkMax::MotorType::kBrushless);

  rev::CANSparkMax* rMotor = new rev::CANSparkMax(rMotorLeaderID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* rMotorFollower = new rev::CANSparkMax(rMotorFollowerID, rev::CANSparkMax::MotorType::kBrushless);
  //if you don't include getEncoder here, it doesn't build?
  rev::SparkMaxRelativeEncoder lEncoder = lMotor->GetEncoder();
  rev::SparkMaxRelativeEncoder rEncoder = rMotor->GetEncoder();

  rev::SparkMaxPIDController lPID = lMotor->GetPIDController();
  rev::SparkMaxPIDController rPID = rMotor->GetPIDController();

  bool initDriveMotor(rev::CANSparkMax* motor, rev::CANSparkMax* follower, bool invert); //loads initial values into motors such as current limit and phase direction
  bool setPowerBudget(rev::CANSparkMax* motor, float iPeak, float iRated, int limitCycles); //changes the current limits on the motors 
  
  float gyroInitVal = 0.0f;
  
  
  // frc::PIDOutput output; //make a decorator class
  // frc::PIDController* rightStickPID = new frc::PIDController(1.0, 0.0, 0.0, m_imu, &output); //Can you initialize like this

  public:

  frc::ADIS16448_IMU m_imu{};
  std::vector<uint8_t> getConstructorArgs();
  void periodicInit();
  void periodicRoutine();
 
  bool setDriveCurrLimit(float iPeak, float iRated, int limitCycles);
  void arcadeDrive(double vel, double dir); //takes two values from the joystick and converts them into motor output %
  bool PIDDrive(float totalFeet, float maxAcc, float maxVelocity);
  bool PIDDriveSimpleTick(float totalFeet);
  bool PIDDriveTick(float totalFeet, float maxAcc, float maxVelocity);
  bool PIDTurn(float angle, float radius, float maxAcc, float maxVelocity);
  bool PIDGyroTurn(float angle, float radius, float maxAcc, float maxVelocity);
  bool PIDGyroTurnTick(float angle, float radius, float maxAcc, float maxVelocity);
  void LimitRate(double& s, double& t);
  float getGyroAngle();
  void InitGyro();
  void GyroTurn(float theta);
  bool GyroTurnTick(float theta);
  float TurningSensitivity(float rightStick, float leftStick);
  void alignToGoal();
  float getDistanceTraversed();
  void adjustedArcadeDrive(double x, double y);
  float LinearInterpolation(double x, double y, double x2, double y2, double input);
  void addSensitivityPoint(float speed, float sensitivity);


  float sliderValue = 1;
  float adjustSpeed = 0.01;

  bool tested = false;
  bool index = false;
  float pidprevTime;
  float pidprevVelocity;
  float pidprevPosition;

  bool encoderZeroed = false;




};


#endif