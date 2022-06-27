// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Solenoid.h>
#include <frc/PowerDistribution.h>
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsControlModule.h>

// // All Module Includes
#include "DriveBaseModule.h"
#include "DriveBaseModulePID.h"

//DriveBaseModule compRobotDrive;
DriveBaseModulePID GyroPIDDrive;

void Robot::RobotInit()
{
  //compRobotDrive.periodicInit();
 
}

void Robot::RobotPeriodic()
{
  
}
void Robot::AutonomousInit()
{
}
void Robot::AutonomousPeriodic()
{

}

void Robot::TeleopInit()
{
   GyroPIDDrive.periodicInit();

}

void Robot::TeleopPeriodic()
{
    float rightStickOutput = -1.0 * driverStick->GetRawAxis(4);
  

  GyroPIDDrive.arcadeDrive(driverStick->GetRawAxis(1), rightStickOutput - GyroPIDDrive.CalculatePID());
 
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit()
{
}

void Robot::TestPeriodic()
{
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
