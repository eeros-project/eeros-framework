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
    c1({0.2, 0.3, 0.4, 0.5, 0.6, -0.7}),
    c2(56.5),
    c3(-28),
    c4({-5,8,-321}),
    socketA("127.0.0.1", 9876, 0.1),  // client
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
    
  Constant<Matrix<6,1,double>> c1;
  Constant<double> c2;
  Constant<int> c3;
  Constant<Matrix<3,1,int>> c4;
  SocketData<Matrix<6,1,double>, Vector4> socketA;	// send Matrix<6,1,double>, receive Vector4, connect to c1
// SocketData<Matrix<3,1,int>, Vector4> socketA;		// send Matrix<3,1,double>, receive Vector4, connect to c4
// SocketData<double, Vector4> socketA;			// send double, receive Vector4, connect to c2
// SocketData<int, Vector4> socketA;			// send int, receive Vector4, connect to c3
// SocketData<std::nullptr_t, Vector4> socketA;		// send nothing, receive Vector4, no connection
// SocketData<Matrix<6,1,double>, double> socketA;		// send Matrix<6,1,double>, receive double, connect to c1
// SocketData<Matrix<6,1,double>, int> socketA;		// send Matrix<6,1,double>, receive int, connect to c1
// SocketData<Matrix<6,1,double>, std::nullptr_t> socketA;	// send Matrix<6,1,double>, receive nothing, connect to c1
// SocketData<int, std::nullptr_t> socketA;		// send int, receive nothing, connect to c3
    
 private:
  Logger log;
  TimeDomain timedomain;
};

class SocketClientSafetyProperties : public SafetyProperties {
public: 
  SocketClientSafetyProperties() : off("System off") {
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

  log.info() << "EEROS started, socket client";
  
  ControlSystem cs(dt);
  SocketClientSafetyProperties sp;
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
  
  log.info() << "Client program end";	
  return 0;
}

