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
  // testLeftMotor->Set(0.2);
  // testRightMotor->Set(0.2);
}

void Robot::TeleopInit()
{
   GyroPIDDrive.periodicInit();
  frc::SmartDashboard::PutNumber("Pd", GyroPIDDrive.rightStickPID.GetP());
  frc::SmartDashboard::PutNumber("Dee", GyroPIDDrive.rightStickPID.GetD());
}

void Robot::TeleopPeriodic()
{
  float rightStickOutput = -1.0 * driverStick->GetRawAxis(4);

  //check if gyro fell off *******
  GyroPIDDrive.rightStickPID.SetSetpoint(rightStickOutput);
  GyroPIDDrive.arcadeDrive(driverStick->GetRawAxis(1),  GyroPIDDrive.GetOutput());
  frc::SmartDashboard::PutNumber("output", GyroPIDDrive.GetOutput());
  frc::SmartDashboard::PutNumber("gyro", GyroPIDDrive.m_imu.GetRate().value());
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit()
{
  GyroPIDDrive.periodicInit();
  frc::SmartDashboard::PutNumber("Pd", GyroPIDDrive.rightStickPID.GetP());
  frc::SmartDashboard::PutNumber("Dee", GyroPIDDrive.rightStickPID.GetD());
}

void Robot::TestPeriodic()
{
   double m_P = frc::SmartDashboard::GetNumber("Pd", GyroPIDDrive.rightStickPID.GetP());
  frc::SmartDashboard::PutNumber("Pd", m_P);
  GyroPIDDrive.rightStickPID.SetP(m_P);

  double Dee = frc::SmartDashboard::GetNumber("Dee", GyroPIDDrive.rightStickPID.GetD());
  frc::SmartDashboard::PutNumber("Dee", Dee);
  GyroPIDDrive.rightStickPID.SetD(Dee);

  if(driverStick->GetRawButton(1)) {
    frc::SmartDashboard::PutBoolean("Button Pressed", true);
    GyroPIDDrive.rightStickPID.SetSetpoint(0.2);
    GyroPIDDrive.arcadeDrive(0.3, GyroPIDDrive.GetOutput());
    frc::SmartDashboard::PutNumber("output", GyroPIDDrive.GetOutput());
  } else {
    frc::SmartDashboard::PutBoolean("Button Pressed", false);
    GyroPIDDrive.rightStickPID.SetSetpoint(0);
    GyroPIDDrive.arcadeDrive(0, 0);
    frc::SmartDashboard::PutNumber("output", GyroPIDDrive.GetOutput());
  }
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
