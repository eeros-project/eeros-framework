#include <iostream>
#include <eeros/logger/SysLogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/ui/CursesUI.hpp>
#include <unistd.h>

using namespace eeros;
using namespace eeros::sequencer;
using namespace eeros::logger;
using namespace eeros::ui;

namespace eeros {
	namespace examples {
		namespace sequencer {
			namespace simple {
				
				/**** Dummy robot ****/
				class Robot {
				public:
					Robot() : x(0), y(0), z(0) { }
					
					void moveXY(double x, double y) {
						this->x = x; this->y = y;
						usleep(200000);
					}
					
					void moveZ(double z) {
						this->z = z;
						usleep(200000);
					}
					
					double getPosX() {
						return x;
					}
					
					double getPosY() {
						return y;
					}
					
					double getPosZ() {
						return z;
					}
					
					bool tcpSensor() {
						return static_cast<int>(x + y) % 2;
					}
					
					double x, y, z;
				};
				
				/**** Move sequence ****/
				class MoveSequence : public Sequence<void, double, double> {
				public:
					MoveSequence(std::string name, Sequencer* sequencer, Robot* robot) : Sequence<void, double, double>(name, sequencer), robot(robot) { }
					
					void run(double x, double y) {
						log.trace() << "Sequence '" << getName() << "': Moving robot to position x = " << x << " / y = " << y;
						robot->moveZ(5);
						robot->moveXY(x, y);
						robot->moveZ(0);
					}
					
				private:
					Robot* robot;
				};
				
				/**** Detect sequence ****/
				class DetectSequence : public Sequence<bool, double> {
				public:
					DetectSequence(std::string name, Sequencer* sequencer, Robot* robot) : Sequence<bool, double>(name, sequencer), robot(robot) { }
						
					virtual bool checkPreCondition() {
						if(robot->getPosZ() > -0.1) return true;
						return false;
					}
						
					bool run(double z) {
						log.trace() << "Sequence '" << getName() << "': Moving down (to " << z << ")";
						robot->moveZ(z);
						log.trace() << "Sequence '" << getName() << "': Reading sensor value";
						bool res = robot->tcpSensor();
						log.trace() << "Sequence '" << getName() << "': Moving up (to 0)";
						robot->moveZ(0);
						return res;
					}
					
				private:
					Robot* robot;
				};
				
				class MainSequence : public Sequence<> {
				public:
					MainSequence(std::string name, Sequencer* sequencer, Robot* robot, CursesUI* ui) : Sequence<>(name, sequencer), robot(robot), move("move", sequencer, robot), detect("detect", sequencer, robot), ui(ui) {
						log.trace() << "Creating new sequence '" << name << "'...";
					}
						
					void run() {
						double x, y;
						for(int i = 0; i < 8; i++) {
							if(ui != nullptr) { // we have a UI -> ask the user where to look
								//ui->promptForInt(std::to_string(i));
								x = 0; y = 0;
							}
							else {
								x = 2 + 2 * i;
								y = 7 - i;
							}
							log.trace() << "Sequence '" << getName() << "': " << "Moving to position (" << x << '/' << y << ')';
							move(x, y);
							yield();
						
							log.info() << "Sequence '" << getName() << "': " << "Detecting object at position (" << x << '/' << y << ')';
							auto resA = detect(-2);
							if(resA.result) {
								if(resA.value) log.info() << "Sequence '" << getName() << "': -> Object detected!";
								else log.info() << "Sequence '" << getName() << "': -> No object detected!";
							}
							else {
								log.warn() << "Sequence '" << getName() << "': detection failed!";
							}
							yield();
						}
					}
					
					void init() {
						log.trace() << "Sequence '" << getName() << "': " << "Init started";
						sleep(1);
						log.trace() << "Sequence '" << getName() << "': " << "Init done";
					}
					
					void exit() {
						log.trace() << "Sequence '" << getName() << "': " << "Exit started";
						sleep(1);
						log.trace() << "Sequence '" << getName() << "': " << "Exit done";
					}
					
				private:
					Robot* robot;
					MoveSequence move;
					DetectSequence detect;
					CursesUI* ui;
				};
			}
		}
	}
}

int main(int argc, char* argv[]) {
	using namespace eeros::examples::sequencer::simple;
	
	// Compute command line arguments
	char c;
	bool verbose = false;
	bool withUi = false;
	bool startInStepMode = false;
	while((c = getopt(argc, argv, "vus")) != -1) {
		switch(c) {
			case 'v': // verbose
				verbose = true;
				break;
			case 'u':
				withUi = true;
				break;
			case 's':
				startInStepMode = true;
				break;
			case '?':
				std::cerr << "Unknown option `-" << static_cast<char>(optopt) <<"'." << std::endl;
				return -1;
			default:
				abort();
		}
	}
	
	// Create and initialize logger
	SysLogWriter w("Simple Sequencer Example");
	Logger<LogWriter>::setDefaultWriter(&w);
	Logger<LogWriter> log;
	if(verbose) w.show(~0);
	
	log.info() << "Simple Sequencer Example started...";
	
	// Create dummy robot
	Robot robot;
	
	// Create sequencer and UI
	Sequencer sequencer;
	CursesUI ui(sequencer);
	CursesUI* ui_ptr = nullptr; if(withUi) ui_ptr = &ui;
	MainSequence mainSeq("Main Sequence", &sequencer, &robot, ui_ptr);
	if(withUi) ui.dispay();
	
	if(startInStepMode) sequencer.stepMode();
	sequencer.start(&mainSeq);
	
	// Wait until sequencer terminates
	sequencer.join();
	ui.exit();
	ui.join();
	
	log.info() << "Simple Sequencer Example finished...";
}
