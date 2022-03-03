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
};

enum stage {
    ballcollection,
    shooting
};

class AutonomousModule : public ModuleBase
{
public:
    std::vector<uint8_t> getConstructorArgs();
    void periodicInit();
    void periodicRoutine();
    std::vector<pathPoint> path;
private:
    void initializePaths();
    stage autostage = ballcollection;
    pathPoint robPos;
    float robTheta;

    int pathi = 0;
};

#endif
