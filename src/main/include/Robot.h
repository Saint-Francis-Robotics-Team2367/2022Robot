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

  const units::meter_t CAMERA_HEIGHT = 24_in;
  const units::meter_t TARGET_HEIGHT = 5_ft;

  // Angle between horizontal and the camera.
  const units::radian_t CAMERA_PITCH = 40_deg;

  // How far from the target we want to be
  const units::meter_t GOAL_RANGE_METERS = 10_ft;
  const units::meter_t APEX_HEIGHT = TARGET_HEIGHT + 3_ft;

  units::degree_t pitch_degree;
  units::meter_t horizontal_dist = 0;

  double velocity = 0; 
  const double GRAV_CONST = 32.17;
  units::radian_t theta_rads; 
  units::degrees_t theta_degs;
  units::degree_t central_degs;
  const double pi = 3.14159;


  // PID constants should be tuned per robot
  const double P_GAIN = 0.1;
  const double D_GAIN = 0.0;
  frc2::PIDController controller{P_GAIN, 0.0, D_GAIN};

  // Change this to match the name of your camera
  photonlib::PhotonCamera camera{"photonvision"};

  frc::XboxController xboxController{0};

  // Drive motors
  frc::PWMVictorSPX leftMotor{0};
  frc::PWMVictorSPX rightMotor{1};
  frc::DifferentialDrive drive{leftMotor, rightMotor};
};
