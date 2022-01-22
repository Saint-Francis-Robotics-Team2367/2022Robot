#ifndef DRIVEBASEMODULE_H
#define DRIVEBASEMODULE_H

#include <vector>
#include <math.h> 

#include "Macros.h"
#include "ModuleBase.h"
#include "GenericPipe.h"

#include "rev/CANSparkMax.h"
#include <frc/Joystick.h>

#define driverStickPort 0
#define operatorStickPort 1

#define lMotorLeaderID 13 // Change these according to hardware
#define lMotorFollowerID 12 
#define rMotorLeaderID 3
#define rMotorFollowerID 50

// Values that are assigned on init 
#define PIDProportional 1
#define PIDIntegral 0
#define PIDDerivative 0

#define motorInitMaxCurrent 60 // The initial max current setting
#define motorInitRatedCurrent 40 // The inital rated current settings
#define motorInitLimitCycles 50  // The inital number of allowed ms at peak current

#define lInvert true // Inversion setings for sides
#define rInvert false 

#define xDeadband 0.1
#define yDeadband 0.1

class DriveBaseModule : public ModuleBase {
  float rMotorSetpoint; // Current Motor Setpoints
  float lMotorSetpoint;

  float robotProportional; // PID values, for dynamic assignment
  float robotIntegral;
  float robotDerivative;

  GenericPipe* ErrorModulePipe;
  GenericPipe* BrownoutModulePipe;
  GenericPipe* AutonomousModulePipe;

  frc::Joystick* driverStick;
  frc::Joystick* operatorStick;

  rev::CANSparkMax* lMotor;
  rev::CANSparkMax* lMotorFollower;
  rev::CANSparkMax* rMotorFollower;
  rev::CANSparkMax* rMotor;
  
  bool initDriveMotor(rev::CANSparkMax* motor, rev::CANSparkMax* follower, bool invert); //loads initial values into motors such as current limit and phase direction
  bool setPowerBudget(rev::CANSparkMax* motor, float iPeak, float iRated, int limitCycles); //changes the current limits on the motors 
  
  public:

  std::vector<uint8_t> getConstructorArgs();
  void periodicInit();
  void periodicRoutine();
 
  bool setDriveCurrLimit(float iPeak, float iRated, int limitCycles);
  void arcadeDrive(float vel, float dir); //takes two values from the joystick and converts them into motor output %

};

#endif
