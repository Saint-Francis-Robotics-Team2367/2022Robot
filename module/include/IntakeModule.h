#ifndef INTAKEMODULE_H
#define INTAKEMODULE_H

#include "Macros.h"
#include "ModuleBase.h"
#include "GenericPipe.h"

#include "rev/CANSparkMax.h"
#include <frc/Joystick.h>

#define intakeActionID 16
#define intakeRollerID 17

class IntakeModule : public ModuleBase {
private:
  bool intakeActive = false;

  rev::CANSparkMax* intakeAction = new rev::CANSparkMax(intakeActionID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* intakeRoller = new rev::CANSparkMax(intakeRollerID, rev::CANSparkMax::MotorType::kBrushless);

  rev::SparkMaxRelativeEncoder iActEncoder = intakeAction->GetEncoder();

public:
  std::vector<uint8_t> getConstructorArgs();
  void periodicInit();
  void periodicRoutine();

  void intake(frc::Joystick* driverStick, frc::Joystick* operatorStick);
};

#endif