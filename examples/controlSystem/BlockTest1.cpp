#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Step.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/core/Thread.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/sequencer/Step.hpp>

#include <chrono>
#include <unistd.h>
#include <thread>
#include <sys/mman.h>

using namespace eeros::logger;
using namespace eeros::control;
using namespace eeros::math;
using namespace eeros::sequencer;

class MainSequence : public Sequence {
public:
	MainSequence(std::string name, Sequencer& seq) : Sequence(name, seq) { 
//		setNonBlocking();
	}
		
	int action() {
		for (int i = 0; i < 5; i++) usleep(1000000);
	}
private:
};

int main() {
	StreamLogWriter w(std::cout);
	w.show(LogLevel::TRACE);
	Logger log;
	Logger::setDefaultWriter(&w);
	log.set(&w);
 
	log.info() << "Block Test started";
	
	unsigned char dummy[8*1024] = {};
	(mlockall(MCL_CURRENT | MCL_FUTURE) != -1);

	using clk = std::chrono::steady_clock;
	using time_point = clk::time_point;
	time_point start = clk::now();
	time_point stop = clk::now();
	double def = std::chrono::duration<double>(stop - start).count();
	start = clk::now();
// 	eeros::Thread t;
 	std::thread* t = new std::thread();
// 	usleep(4000);

	stop = clk::now();
	double run = std::chrono::duration<double>(stop - start).count() - def;
	log.info() << run;

	log.info() << "start seq";
	auto& sequencer = Sequencer::instance();
	MainSequence mainSeq("Main Sequence", sequencer);
 	sequencer.addSequence(mainSeq);
	start = clk::now();
 	mainSeq.start();
	stop = clk::now();
	run = std::chrono::duration<double>(stop - start).count() - def;
	log.info() << run;
	
	mainSeq.wait();	// wait until sequencer terminates

	start = clk::now();
 	mainSeq.start();
	stop = clk::now();
	run = std::chrono::duration<double>(stop - start).count() - def;
	log.info() << run;
	
	mainSeq.wait();	// wait until sequencer terminates
	stop = clk::now();
	run = std::chrono::duration<double>(stop - start).count() - def;
	log.info() << run;
	return 0;
}