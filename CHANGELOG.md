# Change Log

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).


## v1.3.1
(2021-11-24) ([GitHub compare v1.3.0...v1.3.1](https://github.com/eeros-project/eeros-framework/compare/v1.3.0...v1.3.1))

### Added Features
* Add new blocks for sensors and actors
* Rearrange filters
* Add parabolic gain
* Reduce waiting time when reading the mouse


## v1.3.0
(2021-07-13) ([GitHub compare v1.2.0...v1.3.0](https://github.com/eeros-project/eeros-framework/compare/v1.2.0...v1.3.0))

### Added Features
* Improve readability of CMake files
* Add CMake modules lib loader
* Add library only build option
* Raise cmake to version 3.10 including all wrapper libraries
* Add install targets for examples
* New Kalman filter block
* Template spezialization in SocketData block improved
* SocketData block has no more three stage buffer delay
* Keyboard input block overhauled
* Error in path planner fixed
* Add check to disable timedomains wrongly added twice
* Fix error in D block when subsequent timestamps are equal
* ROS blocks only run when rosmaster is present
* Add new block for generic algorithms
* Add new input class for subsystems
* Add feature to integrator block
* Add off range check for critical inputs and signal checker
* Adjust executor to triggering from EtherCAT


## v1.2.0
(2020-11-25) ([GitHub compare v1.1.0...v1.2.0](https://github.com/eeros-project/eeros-framework/compare/v1.1.0...v1.2.0))

### Added Features
* Add support for CANopen
* Remove compiler warnings
* Compile unit tests conditionally
* Add delay block
* Protect against unintentional copying of blocks
* Improve runtime measurement
* Revise path planners
* Add initial state for integrator block
* Improve integration of libucl library
* Add scaling to mouse input block
* Revise sequencer and sequences
* Rename and rearrange examples
* Revise logger


## v1.1.0
(2019-10-22) ([GitHub compare v1.0.0...v1.1.0](https://github.com/eeros-project/eeros-framework/compare/v1.0.0...v1.1.0))

### Added Features
* Make control blocks thread safe
* Simplify creation of steps and sequences
* Allow for switches to be combined
* Add millisecond resolution to logger output
* Peripheral output block now has fail safe state
* Thread priorities can be chosen upon construction
* Add connection information to Socket connection classes
* SignalChecker can limit check the norm of a vector
* Improve unit tests


## v1.0.0
(2019-01-29) ([Github compare v0.6.0...v1.0.0](https://github.com/eeros-project/eeros-framework/compare/v0.6.0...v1.0.0))

### Added Features
* **control/Gain:** Add smooth gain change, min/max gain, new unit tests and Doxygen documentation. Remove old unit tests. Remove code duplications. ([pull request #9](https://github.com/eeros-project/eeros-framework/pull/9))
* **control/MAFilter:** Implement new class with unit tests and Doxygen documentation. ([pull request #10](https://github.com/eeros-project/eeros-framework/pull/10))
* **control/MedianFilter:** Implement new class with unit tests and Doxygen documentation. ([pull request #12](https://github.com/eeros-project/eeros-framework/pull/12))
* **control/ros/RosPublisherSafetyLevel:** Allow to publish the safety level to ROS.
* **examples/ros:** Improve ROS examples.

### Breaking Changes
* **cmake:** Change library versioning. Version information is fetched from git.


## v0.6.0
(2019-01-16)

### Notes
In this version, CMake does not fetch the library version information from git.

