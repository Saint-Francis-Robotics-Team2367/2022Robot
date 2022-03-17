// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>


void Robot::RobotInit() {
  rev::SparkMaxPIDController johnsonMotorPID = johnsonMotor->GetPIDController();
  johnsonMotorPID.SetP(0.5);
  johnsonMotorPID.SetReference(7, rev::CANSparkMax::ControlType::kPosition);

}

void Robot::RobotPeriodic() {
  neo->Set(1);
  //johnsonMotor->Set(1)
}
 
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  
}

void Robot::TeleopPeriodic() {
  
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
 }

void Robot::TestPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

