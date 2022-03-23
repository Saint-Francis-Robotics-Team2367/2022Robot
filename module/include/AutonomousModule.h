// #ifndef AUTONOMOUSMODULE_H
// #define AUTONOMOUSMODULE_H

// #include "Macros.h"
// #include "ModuleBase.h"
// #include "GenericPipe.h"
// #include <vector>
// #include <math.h>

// #include <iostream>
// #include <fstream>
// #include <string>

// #define PATHS_FILE "paths.txt"

// struct pathPoint {
//     float x;
//     float y;
// };

// enum stage {
//     ballcollection,
//     shooting
// };

// class AutonomousModule : public ModuleBase
// {
// public:
//     std::vector<uint8_t> getConstructorArgs();
//     void periodicInit();
//     void periodicRoutine();
//     std::vector<pathPoint> path;
//     bool shootingPoints[7] = {0};
// private:
//     void initializePaths();
//     stage autostage = ballcollection;
//     pathPoint robPos;
//     float robTheta;

//     int pathi = 0;
//     bool isTrue = false;
//     bool resetPath = false;

//     float coordOffset = 0; // how much the robot should go past the path point to pick up the ball

//     float shootingDistance = 4;
// };

// #endif
