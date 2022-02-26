#ifndef AUTONOMOUSMODULE_H
#define AUTONOMOUSMODULE_H

#include "Macros.h"
#include "ModuleBase.h"
#include "GenericPipe.h"
#include <vector>

#define PATHS_FILE "paths.txt"

struct pathPoint {
    float x;
    float y;
    float r;
};

enum stage {
    ball,
    goal
};

class AutonomousModule : public ModuleBase
{
public:
    std::vector<uint8_t> getConstructorArgs();
    void periodicInit();
    void periodicRoutine();
    std::vector<std::vector<pathPoint>> paths;
private:
    void initializePaths();
    stage autostage;
};

#endif
