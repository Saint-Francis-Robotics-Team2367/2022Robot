#include "ShooterModule.h"
#include <frc/SmartDashboard/SmartDashboard.h>

void ShooterModule::periodicInit() {
    //Shooter Motor inits
    // shooterMotorPID.SetP(shooterkP);
    // shooterMotorPID.SetI(shooterkI);
    // shooterMotorPID.SetD(shooterkD);
    // shooterMotorPID.SetFF(shooterkFF);

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
    if (stateRef->IsDisabled()) return;
    Message* m = nullptr;
    if (stateRef->IsAutonomousEnabled()) {
        m = pipes[1]->popQueue();
    }
    else if (stateRef->IsTeleopEnabled()) {
        m = pipes[0]->popQueue();
    }

    if (m) {
        if (m->str == "shoot") {

            pipes[2]->pushQueue(new Message("shooting", 1));
            float shootStart = frc::Timer::GetFPGATimestamp().value();
            while ((frc::Timer::GetFPGATimestamp().value() - shootStart) > 0.5) {
                continue;
            }
            pipes[2]->pushQueue(new Message("shooting", 0));
            //shooterMotorPID.SetReference(0, rev::CANSparkMax::ControlType::kVelocity);

            pipes[1]->pushQueue(new Message("done", 0));
        }

        if (m->str == "test") {
            if (m->vals[0]) {
                shoot1->Set(0.2);
                shoot2->Set(-0.2);
                shooterMotor->Set(0.2);
                pipes[2]->pushQueue(new Message("shooting", 1));

            }
            else {
                shoot1->Set(0);
                shoot2->Set(0);
                shooterMotor->Set(0);
                pipes[2]->pushQueue(new Message("shooting", 0));
            }
        }
    }


}

std::vector<uint8_t> ShooterModule::getConstructorArgs() { return std::vector<uint8_t> {DriveBaseModuleID, AutonomousModuleID, IntakeModuleID}; }
