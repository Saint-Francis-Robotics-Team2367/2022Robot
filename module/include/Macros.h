// ModuleID Definitions for use in constructorArgs

#define ErrorModuleID 0
#define DriveBaseModuleID 1
#define AutonomousModuleID 2
#define ClimberModuleID 6

// Error loglevel
#define INFO 0
#define LOW 1
#define MEDIUM 2
#define HIGH 3
#define FATAL 4

// Frequency of running periodicRoutine()
#define ErrorModuleRunInterval 80 
#define DriveBaseModuleRunInterval 20 
#define ControllerModuleRunInterval 35
#define AutonomousModuleRunInterval 30
// arbitrary value, pls change
#define ClimberModuleRunInterval 70