#include "ClimberModule.h"

void ClimberModule::climber(int joystickInput, rev::CANSparkMax *c_leftMotor, rev::CANSparkMax *c_rightMotor, frc::Solenoid *leftSolenoid, frc::Solenoid *rightSolenoid) {
if (joystickInput == 0) { // left bumper
    c_leftMotor->Set(-1.0);
    frc::SmartDashboard::PutBoolean("test", true);
    std::cout << "left button pressed" << std::endl;
    frc::SmartDashboard::PutBoolean("right btn", true);

  }  
  
  if (joystickInput == 1) { // right bumper
    c_rightMotor->Set(-1.0);
    std::cout << "right button pressed" << std::endl;
    frc::SmartDashboard::PutBoolean("right btn", true);

  } 
  if (joystickInput == 2){ // left trigger
    c_leftMotor->Set(1.0);
    frc::SmartDashboard::PutBoolean("left trigger", true);
    std::cout << "left trigger pressed" << std::endl;
  } 
  if (joystickInput == 3) { // right trigger
    c_rightMotor->Set(1.0);
    frc::SmartDashboard::PutBoolean("right trigger", true);
    std::cout << "right trigger pressed" << std::endl;
  } 
  if (joystickInput == 4) { // Y, solenoid
    leftSolenoid->Set(true);
  }
  if (joystickInput == 5) { // B, solenoid
    rightSolenoid->Set(true);
  }

  
}

void ClimberModule::periodicInit() {
  this->msInterval = ClimberModuleRunInterval;
  leftSolenoid->Set(false);
  rightSolenoid->Set(false);
 
}

void ClimberModule::periodicRoutine() {
  // Use mode of robo
  Message* m = nullptr;
  if (stateRef->IsAutonomousEnabled())
    m = pipes[1]->popQueue();
  else if (stateRef->IsTeleopEnabled()) 
    m = pipes[0]->popQueue();

  if (!m) return;

  if (m->str == "climb")
  {
    activateClimber = true;
    std::cout << "activating climber" << std::endl;
  }

  if (activateClimber)
  {
    GenericPipe* p = pipes[0];
    
    Message* m = p->popQueue();
    if (m) {
      frc::SmartDashboard::PutBoolean("Message", true);
      if (m->str == "climb") {
        climber(m->vals[0], c_leftMotor, c_rightMotor, leftSolenoid, rightSolenoid);
      }
    }

  }
}

std::vector<uint8_t> ClimberModule::getConstructorArgs() { return std::vector<uint8_t> {DriveBaseModuleID}; }