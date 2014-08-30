Setup a development environment
===============================

Requirements
------------

  - PC or VM with a Linux based operating system

Debian, Ubuntu or Linux Mint
----------------------------

  1. Install necessary tools and libraries: `$ sudo apt-get install git make cmake g++`
  2. Install an IDE. We recommend KDevelop, but you can use Eclipse or another C++ IDE as well. To install KDevelop type: `$ sudo apt-get install kdevelop`
  3. Optional: install additional libraries:
     1. [Comedi library](http://www.comedi.org), version must be at least 0.10! On Ubuntu 14.04 you can install the version from the package archive: `$ sudo apt-get install libcomedi-dev`. On Debian 7 you have to build the Comedi library from source (more details can be found in the INSTALL file shipped with the source code of comedilib).
     2. ncurses: `$ sudo apt-get install libncurses5-dev`
  4. Install the EEROS framework or [build it from source](build.md).
  
Gentoo Linux
------------

  1. Install necessary tools an libraries: `# emerge git make cmake g++`
  2. Install an IDE. We recommend KDevelop, but you can use Eclipse or another C++ IDE as well. To install KDevelop type: `# emerge kdevelop`
  3. Optional: install additional libraries:
     1. [Comedi library](http://www.comedi.org), version must be at least 0.10! There is now ebuild for libcomedi, you have to build it from source yourself (more details can be found in the INSTALL file shipped with the source code of comedilib).
     2. ncurses: `# emerge libncurses5-dev`
  4. Install the EEROS framework or [build it from source](build.md).

