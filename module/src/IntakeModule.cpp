#include "IntakeModule.h"


void IntakeModule::periodicInit() {
  this->msInterval = IntakeModuleRunInterval;
  indexMotors[0] = new rev::CANSparkMax(indexID0, rev::CANSparkMax::MotorType::kBrushed);
  indexMotors[1] = new rev::CANSparkMax(indexID1, rev::CANSparkMax::MotorType::kBrushed);
  indexMotors[2] = new rev::CANSparkMax(indexID2, rev::CANSparkMax::MotorType::kBrushed);
}

void IntakeModule::periodicRoutine() {
  // Use mode of robo
  Message* m = nullptr;
  if (stateRef->IsAutonomousEnabled())
    m = pipes[1]->popQueue();
  else if (stateRef->IsTeleopEnabled()) 
    m = pipes[0]->popQueue();

  if (!m) {
    if (m->str == "disable")
    {
      intakeAction->Set(-1.0);
      intakeRoller->StopMotor();
    }
    if (m->str == "activate")
    {
      intakeAction->Set(1.0);
      intakeRoller->Set(1.0);
      std::cout << "activating shooter" << std::endl;
    }
    if (m->str == "outtake")
    {
      if (intakeActive)
        intakeRoller->Set(-1.0);
    }

    if (m->str == "shooting") {
      if (m->vals[0])
        for (int i = 0;i < 3; i++) {
          indexMotors[i]->Set(1.0);
        }
      else
        for (int i = 0;i < 3; i++) {
          indexMotors[i]->Set(0.0);
        }
    }

    if (m->str == "index") {
      if (m->vals[0]) {
        indexMotors[0]->Set(1.0);
        indexMotors[1]->Set(1.0);
      }
      else {
        indexMotors[0]->Set(0.0);
        indexMotors[1]->Set(0.0);
      }
    }
  }

  if ((intakeActive && iActEncoder.GetPosition() >=  outPos) || (!intakeActive && iActEncoder.GetPosition() <= basePos))
    intakeAction->StopMotor();
    
}

std::vector<uint8_t> IntakeModule::getConstructorArgs() { return std::vector<uint8_t> {DriveBaseModuleID, AutonomousModuleID}; }