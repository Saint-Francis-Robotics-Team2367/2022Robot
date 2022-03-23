#ifndef DRIVEBASEMODULE_H
#define DRIVEBASEMODULE_H

#include <vector>
#include <math.h> 

#include <iostream>
#include <fstream>
#include <string>

#define PATHS_FILE "paths.txt"

#include "Macros.h"

#include <rev/CANSparkMax.h>
#include <frc/Joystick.h>

#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/ADIS16448_IMU.h>
#include <frc/PIDController.h>

#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPipelineResult.h>
#include <photonlib/PhotonTrackedTarget.h>

#define driverStickPort 0
#define operatorStickPort 1

// Values that are assigned on init 
#define PIDProportional 1
#define PIDIntegral 0
#define PIDDerivative 0

#define motorInitMaxCurrent 60 // The initial max current setting
#define motorInitRatedCurrent 40 // The inital rated current settings
#define motorInitLimitCycles 50  // The inital number of allowed ms at peak current

#define lInvert true // Inversion setings for sides
#define rInvert false 

#define xDeadband 0.08
#define yDeadband 0.08

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

  const double deadband = 0.08;
  float prevTime;
  float prev_value_speed;
  float prev_value_turn;

  frc::Joystick* driverStick = new frc::Joystick(driverStickPort);
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

  //frc::PIDController g;
  bool initDriveMotor(rev::CANSparkMax* motor, rev::CANSparkMax* follower, bool invert); //loads initial values into motors such as current limit and phase direction
  bool setPowerBudget(rev::CANSparkMax* motor, float iPeak, float iRated, int limitCycles); //changes the current limits on the motors 
  

  float gyroInitVal = 0.0f;
  frc::ADIS16448_IMU m_imu{};

  public:

  std::vector<uint8_t> getConstructorArgs();
  void periodicInit();
  void periodicRoutine();
 
  bool setDriveCurrLimit(float iPeak, float iRated, int limitCycles);
  void arcadeDrive(float vel, float dir); //takes two values from the joystick and converts them into motor output %
  bool PIDDrive(float totalFeet, float maxAcc, float maxVelocity);
  bool PIDDriveTick(float totalFeet, float maxAcc, float maxVelocity);
  bool PIDTurn(float angle, float radius, float maxAcc, float maxVelocity);
  bool PIDGyroTurn(float angle, float radius, float maxAcc, float maxVelocity);
  bool PIDGyroTurnTick(float angle, float radius, float maxAcc, float maxVelocity);
  void LimitRate(float&s, float&t);
  float getGyroAngle();
  void InitGyro();
  void GyroTurn(float theta);
  bool GyroTurnTick(float theta);
  float TurningSensitivity(float rightStick, float leftStick);
  void alignToGoal();
  float sliderValue = 0.43;

  bool tested = false;
  bool index = false;
  float pidprevTime;
  float pidprevVelocity;
  float pidprevPosition;

  
  photonlib::PhotonCamera cam{"guccicam"};
};

#endif
