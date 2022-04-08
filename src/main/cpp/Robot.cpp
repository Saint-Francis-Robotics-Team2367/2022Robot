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

DriveBaseModule compRobotDrive;
frc::PowerDistribution pdp{40, frc::PowerDistribution::ModuleType::kCTRE};
frc::PneumaticsControlModule myPCM;

frc::DoubleSolenoid pnu1{frc::PneumaticsModuleType::CTREPCM, 0, 1};
frc::DoubleSolenoid pnu2{frc::PneumaticsModuleType::CTREPCM, 2, 3};
frc::DoubleSolenoid pnu3{frc::PneumaticsModuleType::CTREPCM, 4, 5};
frc::DoubleSolenoid pnu4{frc::PneumaticsModuleType::CTREPCM, 6, 7};

frc::Joystick *driverStick = new frc::Joystick(driverStickPort);
frc::Joystick *operatorStick = new frc::Joystick(operatorStickPort);

enum liftState
{
  pre_lift = 0,
  first_rung_grab,
  first_rung_lift,
  second_rung_grab,
  first_rung_release,
  clear_second_rung,
  reach_for_last_rung,
  last_lift,
  grab_final_rung,
  release_second_rung,
  final_state
};
int currLiftState = liftState::pre_lift;

void Robot::RobotInit()
{
  compRobotDrive.periodicInit();
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
  // Move this somewhere else later

  // Shooter Motor inits
  pdp.ClearStickyFaults();
  myPCM.ClearAllStickyFaults();
  pnu1.Set(frc::DoubleSolenoid::kReverse);
  pnu2.Set(frc::DoubleSolenoid::kReverse);
  pnu3.Set(frc::DoubleSolenoid::kReverse);
  pnu4.Set(frc::DoubleSolenoid::kReverse);

  currLiftState = 0;
}

void Robot::TeleopPeriodic()
{
  static bool switchState = false;
  if (switchState != myPCM.GetPressureSwitch())
  {
    switchState = myPCM.GetPressureSwitch();
    printf("Pressure Switch: %d\n", myPCM.GetPressureSwitch());
  }

  compRobotDrive.periodicRoutine();

  // lifter_code

  if (driverStick->GetRawButtonPressed(1))
    pnu1.Toggle();

  if (driverStick->GetRawButtonPressed(2))
    pnu2.Toggle();

  if (driverStick->GetRawButtonPressed(3))
    pnu3.Toggle();

  if (driverStick->GetRawButtonPressed(4))
    pnu4.Toggle();

  if (driverStick->GetRawButtonPressed(6))
  {
    printf("lift state:%d\n", currLiftState++);
    if (currLiftState > final_state)
      currLiftState = final_state;
  }

  if (driverStick->GetRawButtonPressed(5))
  {
    printf("lift state:%d\n", currLiftState--);
    if (currLiftState < 0)
      currLiftState = 0;
  }

  switch (currLiftState)
  {
  case liftState::pre_lift:
    // do nothing here to allow for manual movement
    break;

  case liftState::first_rung_grab:
    pnu1.Set(pnu1.kForward); // right arm extended
    pnu3.Set(pnu3.kForward); // right arm upright

    pnu2.Set(pnu2.kForward); // left arm extened
    pnu4.Set(pnu4.kReverse); // left arm forward
    break;

  case liftState::first_rung_lift:
    pnu1.Set(pnu1.kReverse); // right arm retracted
    pnu3.Set(pnu3.kForward); // right arm upright

    pnu2.Set(pnu2.kForward); // left arm extened
    pnu4.Set(pnu4.kReverse); // left arm forward
    break;

  case liftState::second_rung_grab:
    pnu1.Set(pnu1.kReverse); // right arm retracted
    pnu3.Set(pnu3.kForward); // right arm upright

    pnu2.Set(pnu2.kForward); // left arm extened
    pnu4.Set(pnu4.kForward); // left arm upright
    break;

  case liftState::first_rung_release:
    pnu1.Set(pnu1.kForward); // right arm extended
    pnu3.Set(pnu3.kForward); // right arm upright

    pnu2.Set(pnu2.kForward); // left arm extened
    pnu4.Set(pnu4.kForward); // left arm upright
    break;
  case liftState::clear_second_rung:
    pnu1.Set(pnu1.kReverse); // right arm retracted
    pnu3.Set(pnu3.kReverse); // right arm forward

    pnu2.Set(pnu2.kForward); // left arm extened
    pnu4.Set(pnu4.kForward); // left arm upright
    break;
  case liftState::reach_for_last_rung:
    pnu1.Set(pnu1.kForward);
    pnu3.Set(pnu3.kReverse);

    pnu2.Set(pnu2.kForward);
    pnu4.Set(pnu4.kForward);
    break;
  case liftState::last_lift:
    pnu1.Set(pnu1.kForward);
    pnu3.Set(pnu3.kReverse);

    pnu2.Set(pnu2.kReverse);
    pnu4.Set(pnu4.kForward);
    break;
  case liftState::grab_final_rung:
    pnu1.Set(pnu1.kForward);
    pnu3.Set(pnu3.kForward);

    pnu2.Set(pnu2.kReverse);
    pnu4.Set(pnu4.kForward);
    break;
  case liftState::release_second_rung:
    pnu1.Set(pnu1.kForward);
    pnu3.Set(pnu3.kForward);

    pnu2.Set(pnu2.kForward);
    pnu4.Set(pnu4.kForward);
    break;

  default:
    break;
  }
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
