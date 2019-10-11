#include <eeros/control/SignalChecker.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Fault.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <gtest/gtest.h>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;

 class SafetyPropertiesTest : public safety::SafetyProperties {
public:
	SafetyPropertiesTest() : seUp("se up"), seDown("se Down"), sl1("sl 1"), sl2("sl 2") {	
		addLevel(sl1);
		addLevel(sl2);
		sl1.addEvent(seUp, sl2, safety::kPublicEvent);
		sl2.addEvent(seDown, sl1, safety::kPublicEvent);
		setEntryLevel(sl1);	
	}
   safety::SafetyEvent seUp, seDown;
   safety::SafetyLevel sl1, sl2;
};

// Test unconnected input
TEST(controlSignalCheckerTest, unconnected) {
	SignalChecker<> s1(0,0);
	s1.setName("s1");
	try {
		s1.run();
		FAIL();
	} catch(eeros::Fault const & err) {
		EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 's1'"));
	}
}

// Test limits
TEST(controlSignalCheckerTest, limitsDouble) {
	SignalChecker<> s1(0.5,1.0);
	s1.setName("s1");
	Constant<> c1(2.0);
	s1.getIn().connect(c1.getOut());
	c1.run();
	s1.run();
	SafetyPropertiesTest ssProperties;
  safety::SafetySystem ss(ssProperties, 0.1);
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl1);
	s1.registerSafetyEvent(ss, ssProperties.seUp);
	s1.run();
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl2);
	ss.triggerEvent(ssProperties.seDown);
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl1);
	s1.run();
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl1);
	s1.reset();
	s1.run();
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl2);
	ss.triggerEvent(ssProperties.seDown);
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl1);
	s1.setActiveLevel(ssProperties.sl2);
	s1.run();
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl1);
	s1.setActiveLevel(ssProperties.sl1);
	s1.reset();
	s1.run();
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl2);
	ss.triggerEvent(ssProperties.seDown);
	ss.run();
	c1.setValue(0.7);
	c1.run();
	s1.reset();
	s1.run();
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl1);
}

// Test limits
TEST(controlSignalCheckerTest, limitsMatrix) {
	SignalChecker<Matrix<2,1,double>> s1({-1.0,-1.0},{1.0,1.0});
	s1.setName("s1");
	Constant<Matrix<2,1,double>> c1({2.0,2.0});
	s1.getIn().connect(c1.getOut());
	c1.run();
	s1.run();
	SafetyPropertiesTest ssProperties;
  safety::SafetySystem ss(ssProperties, 0.1);
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl1);
	s1.registerSafetyEvent(ss, ssProperties.seUp);
	s1.run();
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl2);
	ss.triggerEvent(ssProperties.seDown);
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl1);
	c1.setValue({0.7,2.0});
	c1.run();
	s1.reset();
	s1.run();
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl2);
	ss.triggerEvent(ssProperties.seDown);
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl1);
	c1.setValue({-0.1,-0.99});
	c1.run();
	s1.reset();
	s1.run();
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl1);
	c1.setValue({-1.1,3.2});
	c1.run();
	s1.reset();
	s1.run();
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl2);
}

TEST(controlSignalCheckerTest, limitsMatrixNorm) {
  SignalChecker<Matrix<3,1,double>, double, true> s1(0.5, 1.0); // norm limit: 0.5 to 1.0
  Constant<Matrix<3,1,double>> c1({0.57737, 0.57737, 0.57737}); // norm: 1.00003
  s1.getIn().connect(c1.getOut());
  c1.run();
  s1.run(); // doesn't fire since no SE registered
  SafetyPropertiesTest ssProperties;
  safety::SafetySystem ss(ssProperties, 0.1);
  ss.run();
  EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl1);
  s1.registerSafetyEvent(ss, ssProperties.seUp);
  s1.run(); // fires
  ss.run();
  EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl2);
  ss.triggerEvent(ssProperties.seDown);
  ss.run();
  EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl1);
  c1.setValue({0.577351, 0.577351, 0.577351}); // norm: 1.0
  c1.run();
  s1.reset();
  s1.run(); // fires
  ss.run();
  EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl2);
  ss.triggerEvent(ssProperties.seDown);
  ss.run();
  EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl1);
  c1.setValue({0.577, 0.577, 0.577}); // norm: 0.999393
  c1.run();
  s1.reset();
  s1.run(); // doesn't fire
  ss.run();
  EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl1);
  c1.setValue({0.25, 0.25, 0.25}); // norm: 0.433013
  c1.run();
  s1.run(); // fires
  ss.run();
  EXPECT_TRUE(ss.getCurrentLevel() == ssProperties.sl2);
}