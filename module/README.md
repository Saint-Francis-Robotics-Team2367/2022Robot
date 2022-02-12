# ModuleBase
This is the base class from which all threaded modules will inherit. It implements functionality generic to all modules with a vector of pipes and scheduling based on function runtime and ideal frequency.
The pure virtual functions `ModuleBase::periodicInit()` and `ModuleBase::periodicRoutine()` are where child classes will implement functionality specific to the module. The constructor is left unimplemented due to differences in module structure. 
Guidelines for module writing:
- Do not write a constructor, there is nothing guaranteed at the time of object construction
- Do all of your init work in `periodicInit()`, which includes setting up pipes and, critically, setting up `msInterval`
- `periodicRoutine()` will not be called if you don't set `msInterval`
- Encapsulate into `periodicRoutine()`, as it the only thing that is called.
- Provide specific and descriptive errors, although the module name is not required in the error message (the error module extracts a pseudo-stacktrace from pipe metadata)

The ordering of the pipe vector is important.
- `YourChildClass::pipes[0]` is used internally by `ModuleBase::init()` to report overrun errors; use this to report your custom errors as well
- Document your code; the graph constructor is responsible for figuring out how to permute pipes so that everything is connected correctly
