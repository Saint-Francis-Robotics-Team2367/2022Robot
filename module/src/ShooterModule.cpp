#include "ShooterModule.h"
#include "frc/smartdashboard/SmartDashboard.h"

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


    shoot2->Follow(*shoot1, true);
    shoot2->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    shoot1->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    shootPid.SetP(kP);
    shootPid.SetI(kI);
    shootPid.SetD(kD);
    shootPid.SetIZone(Iz);
    shootPid.SetFF(FF);
    shootPid.SetOutputRange(minShooterOutput, maxShooterOutput);
    frc::SmartDashboard::PutNumber("shootSpeedSetPoint", shootSpeedSetPoint); 
}

void ShooterModule::periodicRoutine() {


    frc::SmartDashboard::PutNumber("shooterSpeed", shootEncoder.GetVelocity());
    if(driverStick->GetRawButton(2))
    {
        shootPid.SetOutputRange(minShooterOutput, maxShooterOutput);
        shootSpeedSetPoint = frc::SmartDashboard::GetNumber("shootSpeedSetPoint", shootSpeedSetPoint);
    
        shootPid.SetReference(shootSpeedSetPoint , rev::CANSparkMax::ControlType::kVelocity, 0);
        if(shootEncoder.GetVelocity() < shootSpeedSetPoint * 0.95)
        {
            shooterMotor->Set(-1.0);
        }
        else    
        {
            shooterMotor->StopMotor();
        }
            
    }
    else
    {
        shoot1->StopMotor();
        //shootPid.SetReference(0.0, rev::CANSparkMax::ControlType::kVelocity);
    }

    if (driverStick->GetRawButton(4))
        shooterMotor->Set(-1.0);
    else
    {
      if(!driverStick->GetRawButton(2))
        shooterMotor->StopMotor();
    }
         
    
    /*
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

*/
}