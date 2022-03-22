// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constructor.h"
#include "ModuleBase.h"

// // All Module Includes
#include "DriveBaseModule.h"
#include "ErrorModule.h"
#include "AutonomousModule.h"
void Robot::RobotInit() {
  
  neoPID.SetP(0.5);
  neoPID.SetReference(7, rev::CANSparkMax::ControlType::kPosition);
  frc::SmartDashboard::PutNumber("encoder", neoEncoder.GetPosition());

void Robot::RobotInit() {
    if (!Constructor::constructThreadedRobot(std::vector<ModuleBase*> {new ErrorModule, new DriveBaseModule, new AutonomousModule}, this)) { // Pass a reference of this object to all modules
    // frc::DriverStation::ReportError("[Constructor] Web Construction has failed; ensure it is acyclic and constructable");
    return;
  }
}

void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("range", range);
  frc::SmartDashboard::PutNumber("velocity", velocity);
  frc::SmartDashboard::PutNumber("theta_degs", theta_degs);
  frc::SmartDashboard::PutNumber("setpoint", setpoint);
  frc::SmartDashboard::PutNumber("horiz dist", horizontal_dist);
  frc::SmartDashboard::PutNumber("vdist from apex", dist_from_apex);
  //neo->Set(-0.33);
  //johnsonMotor->Set(1)
}
 
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  //Move this somewhere else later

  //Shooter Motor inits
    shooterMotorPID.SetP(shooterkP);
    shooterMotorPID.SetI(shooterkI);
    shooterMotorPID.SetD(shooterkD);
    shooterMotorPID.SetFF(shooterkFF);

  //These aren't good hood motor inits but whatever, change
  hoodMotorPID.SetP(0.2);
  hoodMotorPID.SetI(0);
  hoodMotorPID.SetD(0.7);

  //also not good
  turretMotorPID.SetP(0.2);
  turretMotorPID.SetI(0);
  turretMotorPID.SetD(0.7);
}

void Robot::TeleopPeriodic() {
  frc::SmartDashboard::GetNumber("vdist from apex", 3);
  photonlib::PhotonPipelineResult result = camera.GetLatestResult();
  if (result.HasTargets()) {
        // First calculate range
        // pitch_degree = units::degree_t{result.GetBestTarget().GetPitch()};
        pitch_degree = 0;

        float range = result.GetBestTarget().GetCameraRelativePose().X().value();

        theta_rads = atan((2*APEX_HEIGHT*(range+sqrt(pow(range, 2)-(TARGET_HEIGHT*(pow(range, 2))/APEX_HEIGHT))))/(pow(range, 2)));
        theta_degs = theta_rads * (180 / pi);
        central_degs = 90 - theta_degs;
        // Use this range as the measurement we give to the PID controller.
        horizontal_dist = cos(CAMERA_MOUNT_ANGLE)*range;
        velocity = sqrt(2*APEX_HEIGHT*GRAV_CONST)/sin(theta_rads);
        // forwardSpeed = -controller.Calculate(range.value(), GOAL_RANGE_METERS.value());
        setpoint = (max_turns_neo550/360) * theta_degs; 
        



        //does driverstick have to be in the module itself or listened for in DriveBase and sent when it's clicked
        if(driverStick->GetRawButtonPressed(1) && !pressed) {
          //Set Shooter Motor to rotate at Caclulated RPS, so it can shoot at it's required velocity, kvelocity control, needs to speed up

          //check if needed to be converted to ticks or rotations
          shooterMotorPID.SetReference(velocity, rev::CANSparkMax::ControlType::kVelocity);
          

          //setpoint above is for the hood angle, so use PID's to get that to the right place
          hoodMotorPID.SetReference(setpoint, rev::CANSparkMax::ControlType::kPosition);

          pressed = true;

          //does it need time to speed up, how much?
          //intake ball in here?, need to send info I think right here
          //have to get ball up to shooter though....
        }

        //go back to 0?
        shooterMotorPID.SetReference(0, rev::CANSparkMax::ControlType::kVelocity);
        
        



        //Insert Tracking Code Herew, remember in bounds of 330 degree range of motion

        //can adjust

        float offsetDegree = result.GetBestTarget().GetYaw(); //does yaw even work or do we use the offeset in x degrees !!!!!


        //CHANGE THIS and MAKE CONSTANT, figure out
        double TICKS_PER_DEGREE = 1;

        if(offsetDegree + currentTurretPosition > 330)  {  //this obviously won't work but serves as a reminder that there are bounds for the turret
          offsetDegree = 330;
        }

        float x_offset_ticks = offsetDegree * TICKS_PER_DEGREE; //if PID in ticks here, figure out

        // float x_offset_rotations = offsetDegree * else solve for this (PRETTY SURE ROTATIONS IS RIGHT THOUGH)

        
        //could have thing with just a P constant, and calculate a D constant ourselves too, like how we did it in feature/vision, try that if you want
        turretMotorPID.SetReference(x_offset_ticks, rev::CANSparkMax::ControlType::kVelocity);






        /* old code with no actual super good constant, also only use one motor now so it's not even bad

         if (fabs(v) > 3)
        m_robotDrive->ArcadeDrive(0, v * 0.02);
        else {
        m_robotDrive->ArcadeDrive(-y, 0);
        }



        */

       //new code
      //figure out from CAD how many ticks is in a degree

        //remember that it is a 330 degree range of motion FOR TURRET (IMPLEMENT WITH THIS !!!!!!!)
        // setShooterSetpoint(setpoint);

      } else {
        // If we have no targets, stay still.
        // forwardSpeed = 0;


        //Use Gyro to get approximate posiiton, but how when it's in its own module, can we create two instances of it?, make sure Gyro turn is also within the bounds 0, 330 or whatever

        //My thoughts
        //Have a GoalInitPos (remember auto paths reset init pos!!!!!!)
        //just turn to the gyro value, if still don't get it, idk do some turn in place ig...

        std::cout << "no targets" << std::endl;
      }
      
  c_leftInput = operatorStick->GetButtonPressed();
  c_rightInput = operatorStick->GetRawAxis(1);
  double yMovement = leftStick - rightStick;
  double xMovement = leftStick + rightStick;
  int ySign = 1;
  int xSign = 1;
  double prcX, prcY;

  prcY = fabs(c_leftInput);
  prcX = fabs(c_rightInput);

  if (prcY <= deadzone) {
    
    c_->Set(0);
    leftLeadMotor->Set(0);
  } else {

    rightLeadMotor->Set(ySign * (1/(1 - deadzone) * (yMovement) - (deadzone/(1 - deadzone)))); // left stick y-axis
    leftLeadMotor->Set(xSign * (1/(1 - deadzone) * (xMovement) - (deadzone/(1 - deadzone))));

  }
  // neo->Set(1.0);
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
 }

void Robot::TestPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

