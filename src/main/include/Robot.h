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
#include <frc/Solenoid.h>
//#include <frc/ADIS16448_IMU.h>

class Robot : public frc::TimedRobot {
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

  frc::Joystick * operatorStick = new frc::Joystick(0);
  int c_rightMotorID = 0;
  int c_leftMotorID = 0;

  // rev::CANSparkMax *c_rightMotor = new rev::CANSparkMax(c_rightMotorID, rev::CANSparkMax::CANSparkMaxLowLevel::MotorType::kBrushless);
  // rev::CANSparkMax *c_leftMotor = new rev::CANSparkMax(c_leftMotorID, rev::CANSparkMax::CANSparkMaxLowLevel::MotorType::kBrushless);

  // frc::Solenoid *leftSolenoid = new frc::Solenoid::Solenoid(0);
  // frc::Solenoid *rightSolenoid = new frc::Solenoid::Solenoid(1);

  
};
