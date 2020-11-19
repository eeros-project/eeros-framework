#include <eeros/control/TimeDomain.hpp> 
#include <eeros/control/Constant.hpp>
#include <eeros/control/SocketData.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/task/Lambda.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/task/Periodic.hpp>
#include <signal.h>

using namespace eeros::control;
using namespace eeros::safety;
using namespace eeros::sockets;
using namespace eeros::logger;
using namespace eeros::math;
using namespace eeros::task;
using namespace eeros;

class ControlSystem {
 public:
  ControlSystem(double dt) : 
    c1({1.5, 2.2, 3.3, 4.6}),
    c2(56.5),
    c3(-28),
    c4(-34.987),
    socketA("", 9876, 0.1),  // server
    log(Logger::getLogger('C')),
    timedomain("Main time domain", dt, true) {
  
    socketA.getOut().getSignal().setName("socketRead");
    socketA.getIn().connect(c1.getOut());
//  socketA.getIn().connect(c2.getOut());
//  socketA.getIn().connect(c3.getOut());
//  socketA.getIn().connect(c4.getOut());
    timedomain.addBlock(c1);
    timedomain.addBlock(c2);
    timedomain.addBlock(c3);
    timedomain.addBlock(c4);
    timedomain.addBlock(socketA);
    
    Executor::instance().add(timedomain);
  }
    
  Constant<Vector4> c1;
  Constant<double> c2;
  Constant<int> c3;
  Constant<Matrix< 1,1,double >> c4;
  SocketData<Vector4, Matrix<6,1,double>> socketA;		// send Vector4, receive Matrix<6,1,double>, connect to c1
// SocketData<Vector4, Matrix<3,1,int>> socketA;			// send Vector4, receive Matrix<2,1,int>, connect to c1
// SocketData<Vector4, double> socketA;				// send Vector4, receive double, connect to c1
// SocketData<Vector4, int> socketA;				// send Vector4, receive int, connect to c1
// SocketData<Vector4, std::nullptr_t> socketA;			// send Vector4, receive nothing, connect to c1
// SocketData<double, Matrix<6,1,double>> socketA;			// send double, receive Matrix<6,1,double>, connect to c2
// SocketData<Matrix<1,1,double>, Matrix<6,1,double>> socketA;	// send Matrix<1,1,double>, receive Matrix<6,1,double>, connect to c4
// SocketData<int, Matrix<6,1,double>> socketA;			// send int, receive Matrix<6,1,double>, connect to c3
// SocketData<std::nullptr_t, Matrix<6,1,double>> socketA;		// send nothing, receive Matrix<6,1,double>, no connection
// SocketData<std::nullptr_t, int> socketA;			// send nothing, receive int, no connection
    
 private:
  Logger log;
  TimeDomain timedomain;
};

class SocketServerSafetyProperties : public SafetyProperties {
 public: 
  SocketServerSafetyProperties() : off("System off") {
    addLevel(off);
    setEntryLevel(off);
  }
  SafetyLevel off;
}; 

void signalHandler(int signum) {
  Executor::instance().stop();
}

int main(int argc, char **argv) {
  signal(SIGINT, signalHandler);
  double dt = 0.01;
  
  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();
  log.info() << "EEROS started, socket server";
  
  ControlSystem cs(dt);
  SocketServerSafetyProperties sp;
  SafetySystem ss(sp, dt);

  Lambda l1;
  Periodic periodic("per1", 1.0, l1);
  periodic.monitors.push_back([&](PeriodicCounter &pc, Logger &log){
    log.info() << cs.socketA.getOut().getSignal();
    cs.c1.setValue(cs.c1.getOut().getSignal().getValue() + (0.1));
  });
    
  auto& executor = Executor::instance();
  executor.setMainTask(ss);
  executor.add(periodic);
  executor.run();
  
  log.info() << "Server program end";	
  return 0;
}

