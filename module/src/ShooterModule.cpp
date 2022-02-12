#include "ShooterModule.h"

bool ShooterModule::zeroEncoders()
{
    pos0 = pos1 = pos2 = 0;
    encoder0.SetPosition(0);
    encoder0.SetPositionConversionFactor(0.168);
    encoder1.SetPosition(0);
    encoder1.SetPositionConversionFactor(0.168);
    encoder2.SetPosition(0);
    encoder2.SetPositionConversionFactor(0.168);
    return true;
}

bool ShooterModule::setPID()
{
    PID0.SetP(m_P);
    PID0.SetI(m_I);
    PID0.SetD(m_D);
    PID0.SetIZone(iZone);

    PID1.SetP(m_P);
    PID1.SetI(m_I);
    PID1.SetD(m_D);
    PID1.SetIZone(iZone);

    PID2.SetP(m_P);
    PID2.SetI(m_I);
    PID2.SetD(m_D);
    PID2.SetIZone(iZone);
    return true;
}

void ShooterModule::periodicInit()
{
    this->msInterval = ShooterModuleRunInterval;

    this->ErrorModulePipe = pipes[0];
    this->IntakeModulePipe = pipes[1];

    this->zeroEncoders();
    this->setPID();
}

void ShooterModule::periodicRoutine()
{
    if (!errors.empty())
    { // Handle internal ModuleBase Errors
        ErrorModulePipe->pushQueue(errors.front());
        errors.pop();
    }

    if (feedButtonState) {
        feedBall();
    }

    if (intakeButtonState) {
        intakeBall();
    }
}

bool ShooterModule::feedBall() {
    if (numBalls == 0) {
        return false;
    }

    pos2 += 2 * feedRate / PI;
    PID2.SetReference(pos2, rev::CANSparkMax::ControlType::kPosition);
    numBalls -= 1;

    if (numBalls == 1) {
        pos1 += 2 * indexRate / PI;
        PID1.SetReference(pos1, rev::CANSparkMax::ControlType::kPosition);
    }

    return true;
}

bool ShooterModule::intakeBall() {
    if (numBalls == 2) {
        return false;
    }

    pos0 += 2 * feedRate / PI;
    PID0.SetReference(pos1, rev::CANSparkMax::ControlType::kPosition);

    if (switchState) {
        if (numBalls == 0) {
            pos1 += 2 * indexRate / PI;
            PID1.SetReference(pos1, rev::CANSparkMax::ControlType::kPosition);
        }
        numBalls += 1;
    }

    return true;
}

std::vector<uint8_t> ShooterModule::getConstructorArgs() { return std::vector<uint8_t>{ErrorModuleID}; }