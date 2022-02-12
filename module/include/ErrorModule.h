#ifndef ERRORMODULE_H
#define ERRORMODULE_H

#include <string>
#include <iostream>

#include <frc/DriverStation.h>

#include "ModuleBase.h"
#include "GenericPipe.h"
#include "Macros.h"

class ErrorModule : public ModuleBase {
  std::string priorities[5] = {"INFO: ", "LOW: ", "MEDIUM: ", "HIGH: ", "FATAL: "};
  std::string prefixes[7] = {"DriveBase", "Joystick", "Brownout", "Autonomous", "Pneumatics", "Int/Eth", "Shuffleboard"};
  GenericPipe* FileIOPipe;

  public:  
  std::vector<uint8_t> getConstructorArgs();
  void periodicInit();
  void periodicRoutine();
};
#endif
