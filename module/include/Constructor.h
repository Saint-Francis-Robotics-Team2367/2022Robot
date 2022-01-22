// Krishna Mani FRC 2020 : Constructs the robot graph programmatically 

#ifndef CONSTRUCTOR_H
#define CONSTRUCTOR_H

#include <thread>
#include <vector>
#include <iostream>

#include "GenericPipe.h"
#include "ModuleBase.h"
#include "Robot.h"

class Constructor {
  public:
  static bool constructThreadedRobot(std::vector<ModuleBase*>, Robot*);
};

#endif
