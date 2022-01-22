#include "DriveBaseModule.h"

bool DriveBaseModule::initDriveMotor(rev::CANSparkMax* motor, rev::CANSparkMax* follower, bool invert) {
  motor->SetInverted(invert);
  follower->Follow(*motor, false);
  return motor->GetLastError() == rev::REVLibError::kOk;
}

bool DriveBaseModule::setPowerBudget(rev::CANSparkMax* motor, float iPeak, float iRated, int limitCycles) {
  motor->SetSmartCurrentLimit(iRated);
  motor->SetSecondaryCurrentLimit(iPeak, limitCycles);
  return motor->GetLastError() == rev::REVLibError::kOk;
}

bool DriveBaseModule::setDriveCurrLimit(float iPeak, float iRated, int limitCycles) {
  bool setlFront = setPowerBudget(lMotor, iPeak, iRated, limitCycles);
  bool setrFront = setPowerBudget(rMotor, iPeak, iRated, limitCycles);
  bool setlBack = setPowerBudget(lMotorFollower, iPeak, iRated, limitCycles);
  bool setrBack = setPowerBudget(rMotorFollower, iPeak, iRated, limitCycles);

  return setlFront && setrFront && setlBack && setrBack; // Failure on false
}

void DriveBaseModule::arcadeDrive(float vel, float dir) {
  // Convert joystick input into motor outputs in voltage mode

	if (std::abs(vel) <= yDeadband) { vel = 0; }
	if (std::abs(dir) <= xDeadband) { dir = 0; }

  float lMotorOutput;
  float rMotorOutput;

	// SFDrive Arcade Drive
	double maxInput = std::copysign(std::max(std::abs(vel), std::abs(dir)), vel);

	if (vel >= 0.0) {
		if (dir >= 0.0) {
			lMotorOutput = maxInput;
			rMotorOutput = vel - dir;
		} else {
			lMotorOutput = vel + dir;
			rMotorOutput = maxInput;
		}
	} else {
		if (dir >= 0.0) {
			lMotorOutput = vel + dir;
			rMotorOutput = maxInput;
		} else {
			lMotorOutput = maxInput;
			rMotorOutput = vel - dir;
		}
	}

	lMotor->Set(lMotorOutput);
	rMotor->Set(rMotorOutput);
}

void DriveBaseModule::periodicInit() {
  this->msInterval = DriveBaseModuleRunInterval;
  
  this->ErrorModulePipe = pipes[0];

  // this->BrownoutModulePipe = pipes[1];
  // this->AutonomousModulePipe = pipes[2];

  driverStick = new frc::Joystick(driverStickPort);
  operatorStick = new frc::Joystick(operatorStickPort);

  lMotor = new rev::CANSparkMax(lMotorLeaderID, rev::CANSparkMax::MotorType::kBrushless);
  lMotorFollower = new rev::CANSparkMax(lMotorFollowerID, rev::CANSparkMax::MotorType::kBrushless);

  rMotor = new rev::CANSparkMax(rMotorLeaderID, rev::CANSparkMax::MotorType::kBrushless);
  rMotorFollower = new rev::CANSparkMax(rMotorFollowerID, rev::CANSparkMax::MotorType::kBrushless);

  if (!(initDriveMotor(lMotor, lMotorFollower, lInvert) && initDriveMotor(rMotor, rMotorFollower, rInvert))) {
    ErrorModulePipe->pushQueue(new Message("Could not initialize motors!", FATAL));
    return;
  }

  if (!setDriveCurrLimit(motorInitMaxCurrent, motorInitRatedCurrent, motorInitLimitCycles)) {
    ErrorModulePipe->pushQueue(new Message("Failed to set motor current limit", HIGH)); // Not irrecoverable, but pretty bad
  }

  // Need to add PID Setters

  ErrorModulePipe->pushQueue(new Message("Ready", INFO));
}

void DriveBaseModule::periodicRoutine() {
  // Use mode of robot to determine control source
  // Autonomous -> AutonomousPipe
  // Monitor input from BrownoutPipe
  // Command manipulators from operatorStick state

  if (!errors.empty()) { // Handle internal ModuleBase Errors
    ErrorModulePipe->pushQueue(errors.front());
    errors.pop();
  }

  if (stateRef->IsOperatorControl()) {
    arcadeDrive(driverStick->GetRawAxis(1), driverStick->GetRawAxis(4));
    return;
  }
	// Add rest of manipulator code...
}

std::vector<uint8_t> DriveBaseModule::getConstructorArgs() { return std::vector<uint8_t> {ErrorModuleID}; }