#ifndef DRIVEBASEMODULE_H
#define DRIVEBASEMODULE_H

#include <vector>
#include <math.h> 

#include "Macros.h"
#include "ModuleBase.h"
#include "GenericPipe.h"

#include "rev/CANSparkMax.h"
#include <frc/Joystick.h>

#include <frc/SmartDashboard/SmartDashboard.h>

#define driverStickPort 0
#define operatorStickPort 1

#define lMotorLeaderID 12 // Change these according to hardware
#define lMotorFollowerID 13
#define rMotorLeaderID 15
#define rMotorFollowerID 14

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

#define centerToWheel 1.06
#define PI 3.141592654

class DriveBaseModule : public ModuleBase {
  float rMotorSetpoint; // Current Motor Setpoints
  float lMotorSetpoint;

  float robotProportional; // PID values, for dynamic assignment
  float robotIntegral;
  float robotDerivative;

  bool pressed = false;
  
  const double deadband = 0.08;
  float prevTime;
  float prev_value_speed;
  float prev_value_turn;

  GenericPipe* ErrorModulePipe;
  GenericPipe* BrownoutModulePipe;
  GenericPipe* AutonomousModulePipe;

  frc::Joystick* driverStick;
  frc::Joystick* operatorStick;


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
  
  public:

  std::vector<uint8_t> getConstructorArgs();
  void periodicInit();
  void periodicRoutine();
 
  bool setDriveCurrLimit(float iPeak, float iRated, int limitCycles);
  void arcadeDrive(float vel, float dir); //takes two values from the joystick and converts them into motor output %
  bool PIDDrive(float totalFeet, float maxAcc, float maxVelocity);
  bool PIDTurn(float angle, float totalFeet, float maxAcc, float maxVelocity);
  void LimitRate(float&s, float&t);
};

#endif
