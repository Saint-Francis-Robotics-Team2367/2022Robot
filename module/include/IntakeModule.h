#ifndef INTAKEMODULE_H
#define INTAKEMODULE_H

#include "Macros.h"
#include "ModuleBase.h"
#include "GenericPipe.h"

#include "rev/CANSparkMax.h"
#include <frc/Joystick.h>

#define intakeActionID 11
#define intakeRollerID 10
#define indexID0 1
#define indexID1 2
#define indexID2 69

class IntakeModule : public ModuleBase {
private:
  bool intakeActive = false;

  rev::CANSparkMax* intakeAction = new rev::CANSparkMax(intakeActionID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* intakeRoller = new rev::CANSparkMax(intakeRollerID, rev::CANSparkMax::MotorType::kBrushless);

  //rev::CANSparkMax* intakeRoller;
  //rev::CANSparkMax* intakeAction;

  rev::SparkMaxRelativeEncoder iActEncoder = intakeAction->GetEncoder();
  double basePos = 0.0;
  double outPos = 2.5;


  rev::CANSparkMax* indexMotors[3];



public:
  std::vector<uint8_t> getConstructorArgs();
  void periodicInit();
  void periodicRoutine();
};

#endif
