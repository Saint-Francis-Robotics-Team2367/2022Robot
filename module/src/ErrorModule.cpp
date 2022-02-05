#include "ErrorModule.h"

void ErrorModule::periodicInit() {
  this->msInterval = ErrorModuleRunInterval; // 50ms between calls
  // frc::DriverStation::ReportError("[ErrorModule] Ready");
}

void ErrorModule::periodicRoutine() {
  if (!errors.empty()) { // The Error Module is throwing Errors
    // frc::DriverStation::ReportError("[ErrorModule] " + priorities[(int)errors.front()->val] + errors.front()->str + "\n");
    errors.pop();
  }

  Message* msg;	
  std::string trace = "";

  for (uint8_t i = 0; i < pipes.size(); i++) { // First Module is used for FileIO module, which does not report errors
    msg = pipes[i]->popQueue();

    if (!msg) { continue; } // No Errors present


    trace += "[" + prefixes[i] + "] " + priorities[(int)msg->val] + msg->str + "\n";

    // Do file IO to write errors to log
  }

  // if (!trace.empty()) frc::DriverStation::ReportError(trace);
}

std::vector<uint8_t> ErrorModule::getConstructorArgs() { return std::vector<uint8_t> {DriveBaseModuleID}; }
