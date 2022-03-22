#include "IntakeModule.h"


void IntakeModule::periodicInit() {
  indexMotors[0] = new rev::CANSparkMax(indexID0, rev::CANSparkMax::MotorType::kBrushed);
  indexMotors[1] = new rev::CANSparkMax(indexID1, rev::CANSparkMax::MotorType::kBrushed);
  indexMotors[2] = new rev::CANSparkMax(indexID2, rev::CANSparkMax::MotorType::kBrushed);
  iActEncoder.SetPosition(0);
  iActPIDController.SetP(0.2);
  intakeAction->SetSmartCurrentLimit(5);
  //iActEncoder.SetPositionConversionFactor(0.04);
}

void IntakeModule::periodicRoutine() {
  // Use mode of robo
/*
    if (m) {
      if (m->str == "disable")
      {
        iActPIDController.SetReference(25, rev::CANSparkMax::ControlType::kPosition);
        intakeRoller->Set(0);
      }
      if (m->str == "activate")
      {
        iActPIDController.SetReference(-25, rev::CANSparkMax::ControlType::kPosition);
        intakeRoller->Set(-1.0);
        std::cout << "activating intake" << std::endl;
      }
      if (m->str == "outtake")
      {
        if (intakeActive)
          intakeRoller->Set(1.0);

        for (int i = 0;i < 3; i++) {
          indexMotors[i]->Set(-1.0);
        }
      }

      if ((sm) && (sm->str == "shooting")) {
        if (sm->vals[0])
          for (int i = 0;i < 3; i++) {
            indexMotors[i]->Set(1.0);
          }
        else
          for (int i = 0;i < 3; i++) {
            indexMotors[i]->Set(0.0);
          }
      }

      if (m->str == "index") {
        if (m->vals[0] == 1) {
          indexMotors[0]->Set(1.0);
          indexMotors[1]->Set(1.0);
        }
        else {
          indexMotors[0]->Set(0.0);
          indexMotors[1]->Set(0.0);
        }
      }
    }
    if (stateRef->IsAutonomousEnabled())
      m = pipes[1]->popQueue();
    else if (stateRef->IsTeleopEnabled()) 
      m = pipes[0]->popQueue();
  }
*/    
}