#include "AutonomousModule.h"

void AutonomousModule::periodicInit() {
    robPos.x = 0;
    robPos.y = 0;
}

void AutonomousModule::periodicRoutine() {
    float theta = atan2((path[0].x - robPos.x), (path[0].y - robPos.y));
    theta = theta - robTheta;

    std::vector<float> mVec = {theta, 0, 10, 1};
    Message m = Message("PT", mVec);
    pipes[0]->pushQueue(&m);
    
}

std::vector<uint8_t> AutonomousModule::getConstructorArgs() { return std::vector<uint8_t> {DriveBaseModuleID}; }