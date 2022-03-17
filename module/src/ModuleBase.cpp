// // Krishna Mani FRC 2020

// #include "ModuleBase.h"

// void ModuleBase::init(std::vector<GenericPipe*> pipes, Robot* stateRef) {
//   this->pipes = pipes; 
//   this->stateRef = stateRef; 

//   periodicInit(); 

//   while (true) { 
//     auto nextRun = std::chrono::steady_clock::now() + std::chrono::milliseconds(msInterval);
//     periodicRoutine();
    
//     if (std::chrono::steady_clock::now() > nextRun) { // periodicRoutine() has overrun its set interval
//       errors.push(new Message("Execution Interval Overrun!", 3));
//       continue;
//     }

//     std::this_thread::sleep_until(nextRun);
//   }
// }
