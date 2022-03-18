// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>


void Robot::RobotInit() {
  // rev::SparkMaxPIDController johnsonMotorPID = johnsonMotor->GetPIDController();
  // johnsonMotorPID.SetP(0.5);
  // johnsonMotorPID.SetReference(7, rev::CANSparkMax::ControlType::kPosition);

}

void Robot::RobotPeriodic() {
  neo->Set(-0.33);
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
 std::cout << "TestInit: Entering init" << std::endl;
 tested_motors = false;
 // Robot::checkMotorIDs();
 std::cout << "TestInit: Exiting function" << std::endl;
}
 
void Robot::TestPeriodic() {
 std::cout << "TestPeriodic: Entering periodic" << std::endl;
 if (tested_motors == false) {
   std::cout << "-------------ENTERING FUNCTION------------" << std::endl;
   TestFunctions->checkMotorIDs();
   std::cout << "-------------EXITING FUNCTION------------" << std::endl;
   tested_motors = true;
 } else {
   std::cout << "-------------EXITING CODE------------" << std::endl;
   exit(0);
 }
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

