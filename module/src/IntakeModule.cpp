#include "IntakeModule.h"
#include "frc/SmartDashboard/SmartDashboard.h"
#include <iostream>
void IntakeModule::periodicInit() {
  indexMotors[0] = new rev::CANSparkMax(indexID0, rev::CANSparkMax::MotorType::kBrushed);
  indexMotors[1] = new rev::CANSparkMax(indexID1, rev::CANSparkMax::MotorType::kBrushed);
  indexMotors[2] = new rev::CANSparkMax(indexID2, rev::CANSparkMax::MotorType::kBrushed);
  indexMotors[0]->GetDeviceId();
  frc::SmartDashboard::PutNumber("intakeSpeed", -0.5);
  frc::SmartDashboard::PutNumber("armCurrent", 4);
  frc::SmartDashboard::PutNumber("armSpeedUp", .5);
  frc::SmartDashboard::PutNumber("armSpeedDown", .5);
  intakeAction->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  //iActEncoder.SetPositionConversionFactor(0.04);
}

void IntakeModule::periodicRoutine() {  frc::SmartDashboard::PutNumber("DevID", indexMotors[0]->GetDeviceId());
  frc::SmartDashboard::PutNumber("IsBrushed", indexMotors[0]->GetInitialMotorType() == rev::CANSparkMaxLowLevel::MotorType::kBrushed);
  if(driverStick->GetRawAxis(2) > 0.05)
  {
    intakeAction->SetSmartCurrentLimit(frc::SmartDashboard::GetNumber("armCurrent", 4));
    intakeAction->Set( driverStick->GetRawAxis(2) * frc::SmartDashboard::GetNumber("armSpeedUp", .5) );
  }
  else if (driverStick->GetRawAxis(3) > 0.05)
  {
    intakeAction->SetSmartCurrentLimit(frc::SmartDashboard::GetNumber("armCurrent", 4));
    intakeAction->Set( -1.0 * frc::SmartDashboard::GetNumber("armSpeedDown", .5) * driverStick->GetRawAxis(3) );
  }
  else
  {
    intakeAction->StopMotor();
  }

  if(driverStick->GetRawButton(1))
  {
    indexMotors[0]->Set(-0.5);
    indexMotors[1]->Set(-0.5);
    indexMotors[2]->Set(-1.0);
  }
  else if(! (driverStick->GetRawButton(5) | driverStick->GetRawButton(6)))
  {
    indexMotors[0]->StopMotor();
    indexMotors[1]->StopMotor();
    indexMotors[2]->StopMotor();
  }

  if(driverStick->GetRawButton(5))
  {
    indexMotors[0]->Set(-1.0);
    indexMotors[1]->Set(-1.0);
    intakeRoller->Set(frc::SmartDashboard::GetNumber("intakeSpeed", -0.5));
  }
  else
  {
    if(!driverStick->GetRawButton(1)) {
      indexMotors[0]->StopMotor();
      indexMotors[1]->StopMotor();
    }
    intakeRoller->StopMotor();
  }

  if(driverStick->GetRawButton(6))
  {
    indexMotors[1]->Set(-0.7);
    indexMotors[2]->Set(-1.0);
  }
  else
  {
    if(! (driverStick->GetRawButton(1) | driverStick->GetRawButton(5)))
    {
    indexMotors[1]->StopMotor();
    indexMotors[2]->StopMotor();
    }
  }
  

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

void IntakeModule::enable() {
   //indexMotors[0]->Set(-0.8);
   //indexMotors[1]->Set(-0.8);
   //indexMotors[2]->Set(-1.0);
   //intakeRoller->Set(frc::SmartDashboard::GetNumber("intakeSpeed", -0.5));
}

void IntakeModule::disable() {
   //std::cout << "disabling intake" << std::endl;
   //indexMotors[0]->StopMotor();
   //indexMotors[1]->StopMotor();
   //indexMotors[2]->StopMotor();
   //intakeRoller->StopMotor();
}