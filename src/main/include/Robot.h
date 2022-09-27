// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <math.h>
#include <rev/CANSparkMax.h>
//#include "DriveBaseModulePID.h"
#include <thread>
#include <chrono>
#include "AHRS.h"
#include "frc/SPI.h"
#include "frc/SerialPort.h"

#include "frc/PIDController.h"

//module includes
#include "GyroDrivePID.h"




class Robot : public frc::TimedRobot {

   struct pathPoint {
    float x;
    float y;
  };


 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  //Auto Stuff
  void initializePaths();

  //DriveBaseModulePID GyroPIDDrive;
  frc::Joystick* driverStick = new frc::Joystick(0);

  // void gyroDriving() {
  //     float rightStickOutput = -1.0 * driverStick->GetRawAxis(4);
  //     frc::SmartDashboard::PutNumber("in thread right stick output", rightStickOutput);
  //     GyroPIDDrive.rightStickPID.SetSetpoint(rightStickOutput);
  //     //GyroPIDDrive.arcadeDrive(driverStick->GetRawAxis(1),  GyroPIDDrive.GetOutput());
  //     GyroPIDDrive.arcadeDrive(driverStick->GetRawAxis(1),  GyroPIDDrive.GetOutput());
  //     frc::SmartDashboard::PutNumber("output", GyroPIDDrive.GetOutput());
  //     frc::SmartDashboard::PutNumber("gyro", GyroPIDDrive.m_imu.GetRate().value());
  // }

  void gyroDriving();

  void initThread();

  // void initThread() { //removed ref to Joystick?
  // //need init here?
  // int hi = 1;
  //   while(true) {
  //     //I don't this this works lmao
  //     auto nextRun = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
  //     frc::SmartDashboard::PutNumber("bool", ++hi);
  //     gyroDriving();
  //     std::this_thread::sleep_until(nextRun);
  //   }
  //   frc::SmartDashboard::PutBoolean("yo", true);
  // };



  float goalPosition = 90; //if we have a 90 degree gyro thing to face towards the goal, else use photon....needs testing either way
  pathPoint robPos;
  float robTheta;
  std::vector<pathPoint> path;
  int pathi = 0;
  bool isTrue = false;
  bool resetPath = false;
  float coordOffset = 0; // how much the robot should go past the path point to pick up the bal
  float shootingDistance = 3;
  bool shootingPoints[7] = {0};
  bool tested = false;

  float d = 0;
  float theta = 0;
  bool moveFlag = false;
  bool turnFlag = false;
  bool shootFlag = true;
  float delays;
  float shoottimestart;
  
  // rev::CANSparkMax* testRightMotor = new rev::CANSparkMax(1, rev::CANSparkMax::MotorType::kBrushless);
  // rev::CANSparkMax* testLeftMotor = new rev::CANSparkMax(14, rev::CANSparkMax::MotorType::kBrushless);
  double n = 0.3;

  
};


