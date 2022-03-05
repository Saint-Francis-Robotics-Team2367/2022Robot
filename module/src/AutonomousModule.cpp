#include "AutonomousModule.h"
#include <frc/SmartDashboard/SmartDashboard.h>

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

    if ((stateRef->IsAutonomousEnabled()) && (pathi < path.size())) {
        frc::SmartDashboard::PutBoolean("IN auto module", true);
        float theta = atan2((path[0].x - robPos.x), (path[0].y - robPos.y));
        frc::SmartDashboard::PutNumber("IN auto module", true);
        theta = theta - robTheta;
        frc::SmartDashboard::PutNumber("theta", theta);

        std::vector<float> mVec = {30, 0, 7, 21};
        frc::SmartDashboard::PutNumber("num iterations", pathi) ;

        pipes[1]->pushQueue(new Message("PT", 30));
            //isTrue = true;
        
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