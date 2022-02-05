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

bool DriveBaseModule::PIDTurn(float angle, float radius, float maxAcc, float maxVelocity) {
  rEncoder.SetPosition(0);
  lEncoder.SetPosition(0);
  rEncoder.SetPositionConversionFactor(0.168); //check if this works!
  lEncoder.SetPositionConversionFactor(0.168); 

  float currentPosition, currentVelocity, endpoint, setpoint, timeElapsed, distanceToDeccelerate = 0; //currentPosition is the set point
  float prevTime = frc::Timer::GetFPGATimestamp().value();
  endpoint = (angle / 360.0) * (radius + centerToWheel) * (2 * PI);
  // frc::SmartDashboard::PutNumber("endpoint", endpoint);


//never use while loops unless threading
  while(currentPosition < endpoint){
    timeElapsed = frc::Timer::GetFPGATimestamp().value() - prevTime;
    distanceToDeccelerate = (3 * currentVelocity * currentVelocity) / (2 * maxAcc);
    if (distanceToDeccelerate > endpoint - currentPosition) {
      currentVelocity -= (maxAcc * timeElapsed);
    }
    else //increase velocity
    {
      currentVelocity += (maxAcc * timeElapsed);
      if (currentVelocity > maxVelocity)
      {
        currentVelocity = maxVelocity;
      }
    }

    currentPosition += currentVelocity * timeElapsed;
    if(currentPosition > endpoint) {
      currentPosition = endpoint;
    }
    //same as other
   
    double outerSetpoint = (currentPosition * 12) / (PI * 6); // for now this is ticks (maybe rotations / gearRatio if not then)
    double innerSetpoint = ((radius - centerToWheel)/(radius + centerToWheel)) * outerSetpoint;
    
    // frc::SmartDashboard::PutNumber("outerSet", outerSetpoint);
    // frc::SmartDashboard::PutNumber("innerSet", innerSetpoint);

    if(currentPosition < endpoint){
      lPID.SetReference(outerSetpoint, rev::ControlType::kPosition);
      rPID.SetReference(innerSetpoint, rev::ControlType::kPosition);
    }
    prevTime = frc::Timer::GetFPGATimestamp().value();
  }
  return true;
}

bool DriveBaseModule::PIDDrive(float totalFeet, float maxAcc, float maxVelocity) {
  //forward movement only *implement backwards movement with if statement if necessary
  float currentPosition, currentVelocity, timeElapsed, distanceToDeccelerate, setpoint = 0; //currentPosition is the set point
  float prevTime = frc::Timer::GetFPGATimestamp().value();


  rEncoder.SetPosition(0);
  lEncoder.SetPosition(0);
  rEncoder.SetPositionConversionFactor(0.168); //check if this works!
  lEncoder.SetPositionConversionFactor(0.168); 

  while(currentPosition < totalFeet){
    timeElapsed = frc::Timer::GetFPGATimestamp().value() - prevTime;
    distanceToDeccelerate = (3 * currentVelocity * currentVelocity) / (2 * maxAcc);
    if (distanceToDeccelerate > totalFeet - currentPosition) {
      currentVelocity -= (maxAcc * timeElapsed);
    }
    else //increase velocity
    {
      currentVelocity += (maxAcc * timeElapsed);
      if (currentVelocity > maxVelocity)
      {
        currentVelocity = maxVelocity;
      }
    }

    currentPosition += currentVelocity * timeElapsed;
    if(currentPosition > totalFeet) {
      currentPosition = totalFeet;
    }

    //converting currentPosition to ticks? for the motor: inches / (circum) * ticks * gearboxRatio, might look at this later
    setpoint = (currentPosition * 12) / (PI * 6); // for now this is ticks (maybe rotations / gearRatio if not then)
    lPID.SetReference(setpoint, rev::ControlType::kPosition);
    rPID.SetReference(setpoint, rev::ControlType::kPosition);
    prevTime = frc::Timer::GetFPGATimestamp().value();
  }
  return true;
}

void DriveBaseModule::periodicInit() {
  this->msInterval = DriveBaseModuleRunInterval;
  
  this->ErrorModulePipe = pipes[0];

  // this->BrownoutModulePipe = pipes[1];
  // this->AutonomousModulePipe = pipes[2];

  driverStick = new frc::Joystick(driverStickPort);
  operatorStick = new frc::Joystick(operatorStickPort);

  

  // if (!(initDriveMotor(lMotor, lMotorFollower, lInvert) && initDriveMotor(rMotor, rMotorFollower, rInvert))) {
  //   ErrorModulePipe->pushQueue(new Message("Could not initialize motors!", FATAL));
  //   return;
  // }

  if (!setDriveCurrLimit(motorInitMaxCurrent, motorInitRatedCurrent, motorInitLimitCycles)) {
    ErrorModulePipe->pushQueue(new Message("Failed to set motor current limit", HIGH)); // Not irrecoverable, but pretty bad
  }

  // Need to add PID Setters!!

  // ErrorModulePipe->pushQueue(new Message("Ready", INFO));

  double m_P = 0.39, m_I = 0.02, m_D = 2.13, iZone = 0.03;

  // lPID = (rev::SparkMaxPIDController*)malloc(sizeof(rev::SparkMaxPIDController));
  // *lPID = lMotor->GetPIDController();

  // rPID = (rev::SparkMaxPIDController*)malloc(sizeof(rev::SparkMaxPIDController));
  // *rPID = rMotor->GetPIDController();

  lPID.SetP(m_P);
  lPID.SetI(m_I);
  lPID.SetD(m_D);
  lPID.SetIZone(iZone);

  rPID.SetP(m_P);
  rPID.SetI(m_I);
  rPID.SetD(m_D);
  rPID.SetIZone(iZone);

  //redid Encoders here, as well as h file
  // lEncoder = (rev::SparkMaxRelativeEncoder*)malloc(sizeof(rev::SparkMaxRelativeEncoder));
  // *lEncoder = lMotor->GetEncoder(); 

  // rEncoder =  (rev::SparkMaxRelativeEncoder*)malloc(sizeof(rev::SparkMaxRelativeEncoder));
  // *rEncoder = rMotor->GetEncoder(); 

  rEncoder.SetPosition(0);
  lEncoder.SetPosition(0);
  rEncoder.SetPositionConversionFactor(0.168); //check if this works!
  lEncoder.SetPositionConversionFactor(0.168); 
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

  if (stateRef->IsTeleop()) {
    arcadeDrive(driverStick->GetRawAxis(1), driverStick->GetRawAxis(4));
    return;
  }

  if (stateRef->IsAutonomous()) {
    if (!this->pressed && driverStick->GetRawButtonPressed(1)) {
      PIDTurn(90, 5, 1, 1);
      this->pressed = true;
    }
  }
	// Add rest of manipulator code...
}

std::vector<uint8_t> DriveBaseModule::getConstructorArgs() { return std::vector<uint8_t> {ErrorModuleID}; }