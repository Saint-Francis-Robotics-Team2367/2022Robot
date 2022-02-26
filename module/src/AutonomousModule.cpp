#include "AutonomousModule.h"

std::vector<uint8_t> AutonomousModule::getConstructorArgs() { return std::vector<uint8_t> {DriveBaseModuleID}; }