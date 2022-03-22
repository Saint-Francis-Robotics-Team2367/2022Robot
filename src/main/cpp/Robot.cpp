// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constructor.h"
#include "ModuleBase.h"

// // All Module Includes
#include "DriveBaseModule.h"
#include "IntakeModule.h"
#include "ErrorModule.h"
#include "AutonomousModule.h"
#include "ShooterModule.h"
void Robot::RobotInit() {
    if (!Constructor::constructThreadedRobot(std::vector<ModuleBase*> {new ErrorModule, new DriveBaseModule, new AutonomousModule, new IntakeModule, new ShooterModule}, this)) { // Pass a reference of this object to all modules
     std::cout << "poopoo" << std::endl;
     return;
  }
}

void Robot::RobotPeriodic() {
  
}
 
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  //Move this somewhere else later

  //Shooter Motor inits
 
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

