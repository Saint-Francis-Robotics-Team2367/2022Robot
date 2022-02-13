// ModuleID Definitions for use in constructorArgs

#define ErrorModuleID 0
#define DriveBaseModuleID 1
#define BrownoutModuleID 2
#define AutonomousModuleID 3
#define PneumaticsModuleID 4
#define InetEthModuleID 5
#define ShuffleboardModuleID 6
#define ErrorFileIOModuleID 7

// Error loglevel
#define INFO 0
#define LOW 1
#define MEDIUM 2
#define HIGH 3
#define FATAL 4

// Frequency of running periodicRoutine()
#define ErrorModuleRunInterval 80 
#define DriveBaseModuleRunInterval 20
#define IntakeModuleRunInterval 20 
#define ControllerModuleRunInterval 35