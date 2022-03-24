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

//#include <frc/ADIS16448_IMU.h>

#include <photonlib/PhotonCamera.h>
#include <math.h>
#include <rev/CANSparkMax.h>



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

  float goalPosition = 90; //if we have a 90 degree gyro thing to face towards the goal, else use photon....needs testing either way
  pathPoint robPos;
  float robTheta;
  std::vector<pathPoint> path;
  int pathi = 0;
  bool isTrue = false;
  bool resetPath = false;
  float coordOffset = 0; // how much the robot should go past the path point to pick up the bal
  float shootingDistance = 4;
  bool shootingPoints[7] = {0};
  bool tested = false;

  float d = 0;
  float theta = 0;
  bool moveFlag = false;
  bool turnFlag = false;
  bool shootFlag = false;
  float delays;

};


