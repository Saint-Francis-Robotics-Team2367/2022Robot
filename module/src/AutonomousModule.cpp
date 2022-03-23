// #include "AutonomousModule.h"
// #include <frc/SmartDashboard/SmartDashboard.h>
// #include "paths.h"
// #include "ShooterModule.h"

// void AutonomousModule::periodicInit() {
//     initializePaths();
//     this->msInterval = AutonomousModuleRunInterval;
// }

// void AutonomousModule::initializePaths() {
//         int pathi = 0;
//         int sindex = 0;

//         while(sindex < paths.length()){ 
//             if (paths[sindex] == '*') {
//                 pathi++;
//             }
//             else if ((pathi == currpath) && (paths[sindex] == 's')) {
//                 pathPoint finalp = path[path.size() - 1];
//                 float dToGoal = sqrt(pow(finalp.x, 2) + pow(finalp.y, 2));

//                 std::cout << "dtogoal " << dToGoal << std::endl;
                
//                 pathPoint shootingp;
//                 shootingp.x = (-1 * finalp.x / dToGoal) * (dToGoal - shootingDistance) + finalp.x;
//                 shootingp.y = (-1 * finalp.y / dToGoal) * (dToGoal - shootingDistance) + finalp.y;
                
//                 path.push_back(shootingp);
//                 shootingPoints[path.size() - 1] = true;
//                 sindex += 2;
//             }
//             else if (pathi == currpath) {
//                 pathPoint p;
//                 p.x = atof(paths.substr(sindex, 6).c_str());
//                 p.y = atof(paths.substr(sindex + 7, 6).c_str());
//                 path.push_back(p);
//                 sindex += 14;
//             }
//         }
//         pathi = 1;
//         robPos = path[0];

//         for (int i = 0; i < path.size(); i++) {
//             std::cout << path[i].x << ' ' << path[i].y << std::endl;
//         }
// }

// void AutonomousModule::periodicRoutine() {
//     if (stateRef->IsDisabled()) {
//         if (!resetPath) {
//             pathi = 1;
//             robPos = path[0];
//             robTheta = 0;
//             resetPath = true;
//         }
//         return;
//     }
 
//     if ((stateRef->IsAutonomousEnabled()) && (pathi < path.size())) {
//         resetPath = false;
//         Message* m = pipes[0]->popQueue();
//         if ((!m) && (pathi > 0)) return;

//         if (shootingPoints[pathi - 1]) {
//             pipes[2]->pushQueue(new Message("shoot", 0));

//             while (!pipes[2]->popQueue()) {continue;}
//         }

//         pathPoint delta;
//         delta.x = (path[pathi].x - robPos.x);
//         delta.y = (path[pathi].y - robPos.y);

//         float d = sqrt(pow(delta.x, 2) + pow(delta.y, 2));
//         pathPoint unitDir;
//         unitDir.x = delta.x / d;
//         unitDir.y = delta.y / d;

//         delta.x = delta.x + unitDir.x * coordOffset;
//         delta.y = delta.y + unitDir.y * coordOffset;



//         float theta = atan2((path[pathi].x - robPos.x), (path[pathi].y - robPos.y)) * (180/(3.14159265));
//         theta = theta - robTheta;
//         robTheta += theta;

//         std::vector<float> mVec = {theta, 0, 7, 21};

//         pipes[0]->pushQueue(new Message("PT", mVec));
        
//         d = sqrt(pow(delta.x, 2) + pow(delta.y, 2));
//         mVec = {d, 7, 21};
//         pipes[0]->pushQueue(new Message("PD", mVec));

//         robPos.x += delta.x;
//         robPos.y += delta.y;

//         pathi++;

//     }
        
// }  
// std::vector<uint8_t> AutonomousModule::getConstructorArgs() { return std::vector<uint8_t> {DriveBaseModuleID, IntakeModuleID, ShooterModuleID}; }
