// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constructor.h"
#include "ModuleBase.h"

// // All Module Includes
#include "DriveBaseModule.h"
#include "ErrorModule.h"
#include "AutonomousModule.h"

void Robot::RobotInit() {
  //   if (!Constructor::constructThreadedRobot(std::vector<ModuleBase*> {new ErrorModule, new DriveBaseModule, new AutonomousModule}, this)) { // Pass a reference of this object to all modules
  //   // frc::DriverStation::ReportError("[Constructor] Web Construction has failed; ensure it is acyclic and constructable");
  //   return;
  // }
}

void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutBoolean("btn pressed", operatorStick->GetRawAxis(3));
}
 
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  std::cout << "Teleop Initi" << std::endl;
}

void Robot::TeleopPeriodic() {

  std::cout << "Entering Teleop Periodic" << std::endl;
  // 2 neo 550s 
  // solenoid at the last five seconds

  if (operatorStick->GetRawButton(5)) { // left black button
    //c_leftMotor->Set(1.0);
    frc::SmartDashboard::PutBoolean("test", true);
    std::cout << "left button pressed" << std::endl;
    frc::SmartDashboard::PutBoolean("right btn", operatorStick->GetRawButton(5));

  }  
  
  if (operatorStick->GetRawButton(6)) { // right black button
    //c_rightMotor->Set(1.0);
    std::cout << "right button pressed" << std::endl;
    frc::SmartDashboard::PutBoolean("right btn", operatorStick->GetRawButton(6));

  } 
  if (operatorStick->GetRawAxis(2)){ // left black button
    //c_leftMotor->Set(1.0);
    frc::SmartDashboard::PutBoolean("left trigger", operatorStick->GetRawAxis(2));
    std::cout << "left trigger pressed" << std::endl;
  } 
  if (operatorStick->GetRawAxis(3)) { // right black button
    //c_rightMotor->Set(1.0);
    frc::SmartDashboard::PutBoolean("right trigger", operatorStick->GetRawAxis(3));
    std::cout << "right trigger pressed" << std::endl;
  } 
  if (operatorStick->GetRawButtonPressed(4)) { // Y, solenoid
    //solenoid_valve->Set(true);
  }
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
  //solenoid_valve->Set(false);
}

void Robot::TestPeriodic() {
 
  
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif