# Change Log

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).


## Unreleased
(v1.0.0 targeted for 2019-01-29) ([Github compare v0.6.0...master](https://github.com/eeros-project/eeros-framework/compare/v0.6.0...master))

### Added Features
* **control/Gain:** Add smooth gain change, min/max gain, new unit tests and Doxygen documentation. Remove old unit tests. Remove code duplications. ([pull request #109](https://github.com/eeros-project/eeros-framework/pull/9))
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
