#include <eeros/core/Executor.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/control/ros2/RosPublisherDigitalSignal.hpp>
#include <eeros/control/ros2/RosPublisherAnalogSignal.hpp>
#include <eeros/control/ros2/RosPublisherSafetyLevel.hpp>
#include <eeros/control/ros2/RosPublisherOdometry.hpp>
#include <eeros/control/ros2/RosPublisherTwistStamped.hpp>
#include <eeros/control/ros2/RosSubscriberAnalogSignal.hpp>
#include <eeros/control/ros2/RosSubscriberDigitalSignal.hpp>
#include <eeros/control/ros2/RosSubscriberTwist.hpp>
#include <eeros/control/ros2/RosSubscriberTwistStamped.hpp>
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
      : c10(0.5),
        c11({2.4, (double)NAN, 4.443, 23.6, -11.2, 1.3, 0.003}),
        c12(false),
        c13({true, true, false}),
        c30({0.1, 0.2, 0.3}),
        c31({0.0, 0.0, M_PI}),
        c32({-0.1, -0.2, -0.3}),
        c33({1.0, 2.0, 3.0}),
        c36({0.5, -0.2, 0.0, 0.0, 0.0, 0.32}),
        analogOut(node, "/test/analogSignal"),
        analogVectorOut(node, "/test/analogSignalVector"),
        digitalOut(node, "/test/digitalSignal"),
        digitalVectorOut(node, "/test/digitalSignalVector"),
        slOut(node, "/test/safetyLevel"),
        odomOut(node, "/test/odom", "rbf", "of"),
        twistStampedOut(node, "/test/twistStamped"),
        analogIn(node, "/rosNodeTalker/analogSignal"),
        analogVectorIn(node, "/rosNodeTalker/analogSignalVector"),
        digitalIn(node, "/rosNodeTalker/digitalSignal"),
        digitalVectorIn(node, "/rosNodeTalker/digitalSignalVector"),
        twistIn(node, "/rosNodeTalker/twist"),
        twistStampedIn(node, "/test/twistStamped"),
        timedomain("Main time domain", dt, true) {
    analogOut.getIn().connect(c10.getOut());
    analogVectorOut.getIn().connect(c11.getOut());
    digitalOut.getIn().connect(c12.getOut());
    digitalVectorOut.getIn().connect(c13.getOut());
    odomOut.getIn(0).connect(c30.getOut());
    odomOut.getIn(1).connect(c31.getOut());
    odomOut.getIn(2).connect(c32.getOut());
    odomOut.getIn(3).connect(c33.getOut());
    odomOut.getInCovarPose().connect(c34.getOut());
    odomOut.getInCovarTwist().connect(c35.getOut());
    twistStampedOut.getIn().connect(c36.getOut());
    analogIn.getOut().getSignal().setName("analog signal in");
    analogVectorIn.getOut().getSignal().setName("analog signal vector in");
    digitalIn.getOut().getSignal().setName("digital signal in");
    digitalVectorIn.getOut().getSignal().setName("digital signal vector in");
    twistIn.getOut(0).getSignal().setName("twist linear signal in");
    twistIn.getOut(1).getSignal().setName("twist angular signal in");
    twistStampedIn.getOut().getSignal().setName("twist stamped in");
    timedomain.addBlock(c10);
    timedomain.addBlock(c11);
    timedomain.addBlock(c12);
    timedomain.addBlock(c13);
    timedomain.addBlock(c30);
    timedomain.addBlock(c31);
    timedomain.addBlock(c32);
    timedomain.addBlock(c33);
    timedomain.addBlock(c34);
    timedomain.addBlock(c35);
    timedomain.addBlock(c36);
    timedomain.addBlock(analogOut);
    timedomain.addBlock(analogVectorOut);
    timedomain.addBlock(digitalOut);
    timedomain.addBlock(digitalVectorOut);
    timedomain.addBlock(slOut);
    timedomain.addBlock(odomOut);
    timedomain.addBlock(twistStampedOut);
    timedomain.addBlock(analogIn);
    timedomain.addBlock(analogVectorIn);
    timedomain.addBlock(digitalIn);
    timedomain.addBlock(digitalVectorIn);
    timedomain.addBlock(twistIn);
    timedomain.addBlock(twistStampedIn);
    Executor::instance().add(timedomain);
  }
  
  typedef Matrix<7, 1, double> Vector7d;
  typedef Matrix<3, 1, bool> Vector3b;
  typedef Matrix<6, 1, double> VectorTwist;
  Constant<> c10;
  Constant<Vector7d> c11;
  Constant<bool> c12;
  Constant<Vector3b> c13;
  Constant<Vector3> c30, c31, c32, c33;
  Constant<std::array<double,36>> c34, c35;
  Constant<VectorTwist> c36;
  RosPublisherAnalogSignal<> analogOut;
  RosPublisherAnalogSignal<Vector7d> analogVectorOut;
  RosPublisherDigitalSignal<> digitalOut;
  RosPublisherDigitalSignal<Vector3b> digitalVectorOut;
  RosPublisherSafetyLevel slOut;
  RosPublisherOdometry odomOut;
  RosPublisherTwistStamped twistStampedOut;
  RosSubscriberAnalogSignal<> analogIn;
  RosSubscriberAnalogSignal<Vector2> analogVectorIn;
  RosSubscriberDigitalSignal<> digitalIn;
  RosSubscriberDigitalSignal<Vector3b> digitalVectorIn;
  RosSubscriberTwist twistIn;
  RosSubscriberTwistStamped twistStampedIn;
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
      cs.c10.setValue(cs.c10.getValue() + diff);
      cs.c11.setValue(cs.c11.getValue() - diff);
      if ((slOne.getNofActivations() % 10) == 0) {
        log.info() << cs.analogIn.getOut().getSignal();
        log.info() << cs.analogVectorIn.getOut().getSignal();
        log.info() << cs.digitalIn.getOut().getSignal();
        log.info() << cs.digitalVectorIn.getOut().getSignal();
        log.info() << cs.twistIn.getOut(0).getSignal();
        log.info() << cs.twistIn.getOut(1).getSignal();
        log.info() << cs.twistStampedIn.getOut().getSignal();
        log.info();
      }
      if ((slOne.getNofActivations() % 50) == 0) {
        privateContext->triggerEvent(se);
      }
    });

    slTwo.setLevelAction([&](SafetyContext* privateContext) {
      cs.c10.setValue(cs.c10.getValue() - diff);
      cs.c11.setValue(cs.c11.getValue() + diff);
      if ((slTwo.getNofActivations() % 10) == 0) {
        log.info() << cs.analogIn.getOut().getSignal();
        log.info() << cs.analogVectorIn.getOut().getSignal();
        log.info() << cs.digitalIn.getOut().getSignal();
        log.info() << cs.digitalVectorIn.getOut().getSignal();
        log.info() << cs.twistIn.getOut(0).getSignal();
        log.info() << cs.twistIn.getOut(1).getSignal();
        log.info() << cs.twistStampedIn.getOut().getSignal();
        log.info();
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
  // log.show(LogLevel::TRACE);
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
