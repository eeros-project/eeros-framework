#include <eeros/core/Executor.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/control/ros2/RosPublisherDoubleArray.hpp>
#include <eeros/control/ros2/RosPublisherDouble.hpp>
#include <eeros/control/ros2/RosPublisherSafetyLevel.hpp>
#include <eeros/control/ros2/RosPublisherOdometry.hpp>
#include <eeros/control/ros2/RosSubscriberDoubleArray.hpp>
#include <eeros/control/ros2/RosSubscriberDouble.hpp>
#include <eeros/control/ros2/RosSubscriberTwist.hpp>
#include <eeros/control/ros2/RosTools.hpp>
#include <signal.h>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::safety;
using namespace eeros::math;
using namespace eeros::logger;
using namespace eeros::hal;

class ControlSystem {
public:
  ControlSystem(const rclcpp::Node::SharedPtr node, double dt)
      : c1({2.4, NAN, 4.443, 23.6, -11.2, 1.3, 0.003}),
        c2(0.5),
        c30({0.1, 0.2, 0.3}),
        c31({0, 0, M_PI}),
        c32({-0.1, -0.2, -0.3}),
        c33({1, 2, 3}),
        vectorOut(node, "/test/vector"),
        doubleOut(node, "/test/val"),
        slOut(node, "/test/safetyLevel"),
        odom(node, "/test/odom", "rbf", "of"),
        vectorIn(node, "/rosNodeTalker/vector"),
        doubleIn(node, "/rosNodeTalker/val"),
        twistIn(node, "/rosNodeTalker/twist"),
        timedomain("Main time domain", dt, true) {
    vectorOut.getIn().connect(c1.getOut());
    doubleOut.getIn().connect(c2.getOut());
    odom.getIn(0).connect(c30.getOut());
    odom.getIn(1).connect(c31.getOut());
    odom.getIn(2).connect(c32.getOut());
    odom.getIn(3).connect(c33.getOut());
    odom.getInCovarPose().connect(c34.getOut());
    odom.getInCovarTwist().connect(c35.getOut());
    timedomain.addBlock(c1);
    timedomain.addBlock(c2);
    timedomain.addBlock(c30);
    timedomain.addBlock(c31);
    timedomain.addBlock(c32);
    timedomain.addBlock(c33);
    timedomain.addBlock(c34);
    timedomain.addBlock(c35);
    timedomain.addBlock(vectorOut);
    timedomain.addBlock(doubleOut);
    timedomain.addBlock(slOut);
    timedomain.addBlock(odom);
    timedomain.addBlock(vectorIn);
    timedomain.addBlock(doubleIn);
    timedomain.addBlock(twistIn);
    Executor::instance().add(timedomain);
  }
  
  typedef Matrix<7, 1, double> Vector7;
  Constant<Vector7> c1;
  Constant<> c2;
  Constant<Vector3> c30, c31, c32, c33;
  Constant<std::array<double,36>> c34, c35;
  RosPublisherDoubleArray<Vector7> vectorOut;
  RosPublisherDouble doubleOut;
  RosPublisherSafetyLevel slOut;
  RosPublisherOdometry odom;
  RosSubscriberDoubleArray<Vector2> vectorIn;
  RosSubscriberDouble doubleIn;
  RosSubscriberTwist twistIn;
  TimeDomain timedomain;
};

class ROSTestSafetyProperties : public SafetyProperties {
 public:
  ROSTestSafetyProperties(ControlSystem& cs) : slOne("one"), slTwo("two"), se("change"), log(Logger::getLogger()) {
    addLevel(slOne);
    addLevel(slTwo);
    slOne.addEvent(se, slTwo, kPrivateEvent);
    slTwo.addEvent(se, slOne, kPublicEvent);
    setEntryLevel(slOne);
    
    slOne.setLevelAction([&](SafetyContext* privateContext) {
      cs.c1.setValue(cs.c1.getValue() + diff);
      cs.c2.setValue(cs.c2.getValue() - diff);
      if ((slOne.getNofActivations() % 10) == 0) {
        log.info() << cs.doubleIn.getOut().getSignal();
        log.info() << cs.vectorIn.getOut().getSignal();
        log.info() << cs.twistIn.getOut(0).getSignal();
        log.info() << cs.twistIn.getOut(1).getSignal();
      }
      if ((slOne.getNofActivations() % 50) == 0) {
        privateContext->triggerEvent(se);
      }
    });

    slTwo.setLevelAction([&](SafetyContext* privateContext) {
      cs.c1.setValue(cs.c1.getValue() - diff);
      cs.c2.setValue(cs.c2.getValue() + diff);
      if ((slTwo.getNofActivations() % 10) == 0) {
        log.info() << cs.doubleIn.getOut().getSignal();
        log.info() << cs.vectorIn.getOut().getSignal();
        log.info() << cs.twistIn.getOut(0).getSignal();
        log.info() << cs.twistIn.getOut(1).getSignal();
      }
      if ((slTwo.getNofActivations() % 50) == 0) {
        privateContext->triggerEvent(se);
      }
    });
  }

  SafetyLevel slOne;
  SafetyLevel slTwo;
  SafetyEvent se;
  Logger log;
  double diff = 0.1;
};

void signalHandler(int signum) {
  Executor::stop();
}

int main(int argc, char **argv) {	
  double dt = 0.1;

  Logger::setDefaultStreamLogger(std::cout);
  Logger log = Logger::getLogger();
  log.show(LogLevel::TRACE);
  log.info() << "ROS Test1 started";

  RosTools::initRos(argc, argv);
  RosTools::registerCustomSigIntHandler(signalHandler);
  auto node = RosTools::initNode("eerosNode");
  if (node != nullptr) log.info() << "ROS node initialized: " << node->get_name();
     
  ControlSystem cs(node, dt);
  ROSTestSafetyProperties sp(cs);
  SafetySystem ss(sp, dt);
  cs.slOut.setSafetySystem(ss);
  
  signal(SIGINT, signalHandler);	
  auto &executor = Executor::instance();
//   executor.syncWithRosTime();
  executor.setMainTask(ss);
  executor.run();
  
  log.info() << "ROS Test1 end";
  return 0;
}
