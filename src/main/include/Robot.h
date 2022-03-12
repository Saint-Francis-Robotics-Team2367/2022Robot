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

  const double CAMERA_HEIGHT = 2.0;
  const double TARGET_HEIGHT = 102.0;

  // Angle between horizontal and the camera.
  const int CAMERA_PITCH = 40;

  // How far from the target we want to be
  const double GOAL_RANGE_METERS = 10.0;
  const double APEX_HEIGHT = TARGET_HEIGHT + 3;
  const double CAMERA_MOUNT_ANGLE = 40;

  double pitch_degree;
  double horizontal_dist = 0;
  
  double velocity = 0; 
  const double GRAV_CONST = 32.17;
  double theta_rads; 
  double theta_degs;
  double central_degs;
  const double pi = 3.14159;


  // PID constants should be tuned per robot
  const double P_GAIN = 0.1;
  const double D_GAIN = 0.0;

  float range = 0;

  // Change this to match the name of your camera
  photonlib::PhotonCamera camera{"photonvision"};

};
