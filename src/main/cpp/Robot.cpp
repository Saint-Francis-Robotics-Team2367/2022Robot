// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Solenoid.h>
#include <frc/PowerDistribution.h>
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsControlModule.h>
//#include "DriveBaseModule.h"

#include<mutex>
#include <atomic>

std::atomic<bool> shouldRun(true); //initialize to true in teleop init, to false in disable init
GyroDrivePID drive;


void Robot::gyroDriving() {
      float rightStickOutput = -1.0 * driverStick->GetRawAxis(4);
      frc::SmartDashboard::PutNumber("in thread right stick output", rightStickOutput);
      drive.rightStickPID.SetSetpoint(rightStickOutput);
      //GyroPIDDrive.arcadeDrive(driverStick->GetRawAxis(1),  GyroPIDDrive.GetOutput());
      drive.arcadeDrive(driverStick->GetRawAxis(1),  drive.GetOutput());
      frc::SmartDashboard::PutNumber("output", drive.GetOutput());
      frc::SmartDashboard::PutNumber("gyro", drive.gyroSource.ahrs->GetRate());
}

void Robot::initThread() {
    //need init here?
    int counter = 0;
    while(true) {
      //I don't this this works lmao
      auto nextRun = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
      frc::SmartDashboard::PutNumber("timesRun", ++counter);
      if(!shouldRun.load()) {
        break;
      }
      frc::SmartDashboard::PutBoolean("atomic bool lock", shouldRun.load());
      gyroDriving();
      std::this_thread::sleep_until(nextRun);
    }
    frc::SmartDashboard::PutBoolean("yo", true);
}


void Robot::RobotInit()
{

}

void Robot::RobotPeriodic()
{
  
  frc::SmartDashboard::PutNumber("angle", drive.gyroSource.ahrs->GetAngle()); 
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

  drive.periodicInit();
  std::thread th(&Robot::initThread, this);
  th.detach(); //do I get a memory error or am I good?
  shouldRun = true; //needs to be assigned here or drive thread won't run~!, called from other thread...

}

void Robot::TeleopPeriodic()
{
  
}

void Robot::DisabledInit() {
  shouldRun = false;
}
void Robot::DisabledPeriodic() {
}

void Robot::TestInit()
{
  // GyroPIDDrive.periodicInit();
  // frc::SmartDashboard::PutNumber("Pd", GyroPIDDrive.rightStickPID.GetP());
  // frc::SmartDashboard::PutNumber("Dee", GyroPIDDrive.rightStickPID.GetD());
}

void Robot::TestPeriodic()
{
  //  double m_P = frc::SmartDashboard::GetNumber("Pd", GyroPIDDrive.rightStickPID.GetP());
  // frc::SmartDashboard::PutNumber("Pd", m_P);
  // GyroPIDDrive.rightStickPID.SetP(m_P);

  // double Dee = frc::SmartDashboard::GetNumber("Dee", GyroPIDDrive.rightStickPID.GetD());
  // frc::SmartDashboard::PutNumber("Dee", Dee);
  // GyroPIDDrive.rightStickPID.SetD(Dee);

  // if(driverStick->GetRawButton(1)) {
  //   frc::SmartDashboard::PutBoolean("Button Pressed", true);
  //   GyroPIDDrive.rightStickPID.SetSetpoint(0.3);
  //   GyroPIDDrive.arcadeDrive(0, GyroPIDDrive.GetOutput());
  //   frc::SmartDashboard::PutNumber("output", GyroPIDDrive.GetOutput());
  // } else {
  //   frc::SmartDashboard::PutBoolean("Button Pressed", false);
  //   GyroPIDDrive.rightStickPID.SetSetpoint(0);
  //   GyroPIDDrive.arcadeDrive(0, 0);
  //   frc::SmartDashboard::PutNumber("output", GyroPIDDrive.GetOutput());
  // }
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
