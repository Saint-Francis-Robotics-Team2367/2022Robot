
#ifndef AUTONOMOUSMODULE_H
#define AUTONOMOUSMODULE_H

#include "Macros.h"
#include "ModuleBase.h"
#include "GenericPipe.h"
#include <vector>
#include <math.h>


class ShooterModule : public ModuleBase
{
public:
    std::vector<uint8_t> getConstructorArgs();
    void periodicInit();
    void periodicRoutine();
    void align_shooter();
private:
    void initializePaths();
    stage autostage = ballcollection;
    pathPoint robPos;
    float robTheta;

    int pathi = 0;
    bool isTrue = false;
    bool resetPath = false;
};

#endif