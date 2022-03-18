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
#include <frc/ADIS16448_IMU.h>

#include <photonlib/PhotonCamera.h>
#include <math.h>
#include <rev/CANSparkMax.h>
#include <Test.h>
#include "frc/Errors.h"

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

  //Change ID slot there to test
  rev::CANSparkMax * neo = new rev::CANSparkMax(18, rev::CANSparkMax::MotorType::kBrushless);

  //Johnson Motor, uncomment/change ID's when testing
  //rev::CANSparkMax * johnsonMotor = new rev::CANSparkMax(15, rev::CANSparkMax::MotorType::kBrushed);
  bool tested_motors = false;
  int count = 0;

  // Change this to match the name of your camera
  photonlib::PhotonCamera camera{"photonvision"};
  Test* TestFunctions = new Test();


};

