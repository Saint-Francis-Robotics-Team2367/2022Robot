// Krishna Mani FRC 2020 : Base class for all robot threaded modules

#ifndef MODULEBASE_H
#define MODULEBASE_H

#include <vector>
#include <queue>
#include <thread>
#include <chrono>

#include "GenericPipe.h"
#include "Robot.h"

class ModuleBase {
	public:
	std::vector<GenericPipe*> pipes;
	std::queue<Message*> errors;

	int msInterval; 
	Robot* stateRef;

	void init(std::vector<GenericPipe*>, Robot*);
	
	virtual std::vector<uint8_t> getConstructorArgs() = 0;
	virtual void periodicRoutine() = 0;
	virtual void periodicInit() = 0;

};

#endif
