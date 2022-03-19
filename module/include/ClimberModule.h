#ifndef CLIMBERMODULE_H
#define CLIMBERMODULE_H

#include "Macros.h"
#include "ModuleBase.h"
#include "GenericPipe.h"

#include <rev/CANSparkMax.h>
#include <frc/Solenoid.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/PneumaticsModuleType.h>

#define c_rightMotorID 0
#define c_leftMotorID 0

class ClimberModule : public ModuleBase
{
  bool activateClimber = false;

  frc::Solenoid *leftSolenoid = new frc::Solenoid::Solenoid(frc::PneumaticsModuleType::CTREPCM, 0);
  frc::Solenoid *rightSolenoid = new frc::Solenoid::Solenoid(frc::PneumaticsModuleType::CTREPCM, 1);
//   frc::Solenoid *leftSolenoid = new frc::Solenoid::Solenoid(0, frc::PneumaticsModuleType::CTREPCM, 0);
//   frc::Solenoid *rightSolenoid = new frc::Solenoid::Solenoid(0, frc::PneumaticsModuleType::CTREPCM, 1);

  rev::CANSparkMax *c_rightMotor = new rev::CANSparkMax(c_rightMotorID, rev::CANSparkMax::CANSparkMaxLowLevel::MotorType::kBrushless);
  rev::CANSparkMax *c_leftMotor = new rev::CANSparkMax(c_leftMotorID, rev::CANSparkMax::CANSparkMaxLowLevel::MotorType::kBrushless);

public:
  std::vector<uint8_t> getConstructorArgs();
  void periodicInit();
  void periodicRoutine();

  void climber(int joystickInput, rev::CANSparkMax *c_leftMotor, rev::CANSparkMax *c_rightMotor, frc::Solenoid *leftSolenoid, frc::Solenoid *rightSolenoid);
};

#endif
