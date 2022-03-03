#include "AutonomousModule.h"

void AutonomousModule::periodicInit() {
    robPos.x = 0;
    robPos.y = 0;
    pathPoint p;
    p.x = 3;
    p.y = 3;
    path.push_back(p);
    this->msInterval = AutonomousModuleRunInterval;
}

void AutonomousModule::periodicRoutine() {
    if (stateRef->IsDisabled()) {return;}

    if ((stateRef->IsAutonomous()) && (pathi < path.size())) {
        float theta = atan2((path[0].x - robPos.x), (path[0].y - robPos.y));
        theta = theta - robTheta;

        std::vector<float> mVec = {theta, 0, 7, 21};
        Message m = Message("PT", mVec);
        //pipes[0]->pushQueue(&m);

        // float d = sqrt(pow((path[0].x - robPos.x), 2) + pow((path[0].y - robPos.y), 2));
        
        // mVec = {d, 7, 21};
        // m = Message("PD", mVec);
        // pipes[0]->pushQueue(&m);

        // robPos.x += path[0].x;
        // robPos.y += path[0].y;

        pathi++;
    }
}  
std::vector<uint8_t> AutonomousModule::getConstructorArgs() { return std::vector<uint8_t> {DriveBaseModuleID}; }