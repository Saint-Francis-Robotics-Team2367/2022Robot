#include "ShooterModule.h"
#include <frc/SmartDashboard/SmartDashboard.h>

void ShooterModule::periodicInit() {
    //Shooter Motor inits
    shooterMotorPID.SetP(shooterkP);
    shooterMotorPID.SetI(shooterkI);
    shooterMotorPID.SetD(shooterkD);
    shooterMotorPID.SetFF(shooterkFF);

    // //These aren't good hood motor inits but whatever, change
    // hoodMotorPID.SetP(0.2);
    // hoodMotorPID.SetI(0);
    // hoodMotorPID.SetD(0.7);

    // //also not good
    // turretMotorPID.SetP(0.2);
    // turretMotorPID.SetI(0);
    // turretMotorPID.SetD(0.7);
}

void ShooterModule::periodicRoutine() {
    Message* m;
    if (stateRef->IsAutonomousEnabled()) {
        m = pipes[1]->popQueue();
    }
    else if (stateRef->IsTeleopEnabled()) {
        m = pipes[0]->popQueue();
    }

    // photonlib::PhotonPipelineResult result = camera.GetLatestResult();
    // if (!result.HasTargets()) {
    //     result = prevVisionResult;
    // }
    if (m) {
        if (m->str == "shoot") {
            // float range;
            // if (stateRef->IsAutonomousEnabled()) 
            //     range = m->vals[0];
            // else {
            //     float range = result.GetBestTarget().GetCameraRelativePose().X().value();
            // }
            // theta_rads = atan((2*APEX_HEIGHT*(range+sqrt(pow(range, 2)-(TARGET_HEIGHT*(pow(range, 2))/APEX_HEIGHT))))/(pow(range, 2)));
            // theta_degs = theta_rads * (180 / pi);
            // central_degs = 90 - theta_degs; // Use this range as the measurement we give to the PID controller.
            // horizontal_dist = cos(CAMERA_MOUNT_ANGLE)*range;
            // velocity = sqrt(2*APEX_HEIGHT*GRAV_CONST)/sin(theta_rads);
            // setpoint = (max_turns_neo550/360) * theta_degs; //convert setpoint to rotations
            // hoodMotorPID.SetReference(setpoint, rev::CANSparkMax::ControlType::kPosition);

            // while (fabs(hoodMotorEncoder.GetPosition() - setpoint) > 0.2) {
            //     continue;
            // }

            shooterMotorPID.SetReference(velocity, rev::CANSparkMax::ControlType::kVelocity);

            while (fabs(shooterMotorEncoder.GetVelocity() - setpoint) > 0.2) {
                continue;
            }

            pipes[2]->pushQueue(new Message("shooting", 1));
            float shootStart = frc::Timer::GetFPGATimestamp().value();
            while ((frc::Timer::GetFPGATimestamp().value() - shootStart) > 0.5) {
                continue;
            }
            pipes[2]->pushQueue(new Message("shooting", 0));
            shooterMotorPID.SetReference(0, rev::CANSparkMax::ControlType::kVelocity);

            pipes[1]->pushQueue(new Message("done", 0));
        }
        if (m->str == "track") {
            if (m->vals[0]) {
                track = true;
            }
            else {
                track = false;
            }
        }
        if (m->str == "test") {
            if (m->vals[0]) {
                shoot1->Set(1);
                shoot2->Set(-1);
                shooterMotor->Set(1);
            }
            else {
                shoot1->Set(0);
                shoot2->Set(0);
                shooterMotor->Set(0);
            }
        }
    }

    // if (track) {
    //     float offsetDegree = result.GetBestTarget().GetYaw(); 
    //     if (offsetDegree > 0) {
    //         if ((offsetDegree + turretTheta) > turretLimitPos) {
    //             return;
    //         }
    //     }
    //     else {
    //         if ((offsetDegree + turretTheta) < turretLimitNeg) {
    //             return;
    //         }
    //     }
        
    //     turretMotor->Set(offsetDegree * 0.1);
    //     turretTheta += turretMotorEncoder.GetPosition() / turretMotorEncoder.GetCountsPerRevolution() * 360 / 500;
    //     turretMotorEncoder.SetPosition(0);
    // }

}

std::vector<uint8_t> ShooterModule::getConstructorArgs() { return std::vector<uint8_t> {DriveBaseModuleID, AutonomousModuleID, IntakeModuleID}; }
