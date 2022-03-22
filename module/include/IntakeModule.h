#ifndef INTAKEMODULE_H
#define INTAKEMODULE_H

#include "Macros.h"

#include "rev/CANSparkMax.h"
#include <frc/Joystick.h>

class IntakeModule{
private:
  bool intakeActive = false;

  rev::CANSparkMax* intakeAction = new rev::CANSparkMax(intakeActionID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* intakeRoller = new rev::CANSparkMax(intakeRollerID, rev::CANSparkMax::MotorType::kBrushless);

  frc::Joystick* driverStick = new frc::Joystick(0);
  frc::Joystick* operatorStick = new frc::Joystick(1);
  double basePos = 0.0;
  double outPos = 2.5;


  rev::CANSparkMax* indexMotors[3];



public:
  void periodicInit();
  void periodicRoutine();
};

#endif
