Getting started with the EEROS robotics framework
=================================================

Preparation and Hello World
---------------------------

  1. Setup an development environment as desribed [here](setupDevelopmentEnvironment.md).
  2. Start KDevelop
  3. Create a new C++ project:
     1. Project -> New
     2. Choose Standard/Terminal as project type and HelloEeros as project name.
     3. Create the project by clicking Finish
     4. Create the build directory with the given default settings
  4. Say hallo with EEROS:
```cpp
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>

int main() {
	StreamLogWriter w(std::cout);
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	
	log.info() << "Hello, EEROS";
 
    return 0;
}
```
