#include <eeros/safety/SafetySystem.hpp>
#include <eeros/core/Fault.hpp>
#include <gtest/gtest.h>

using namespace eeros;
using namespace eeros::safety;

class SafetyPropertiesTest : public SafetyProperties {
public:
	SafetyPropertiesTest()  : 
		se1("se1"), se2("se2"), se3("se3"), se4("se4"),
		sl1("1"), sl2("2"), sl3("3"), sl4("4"), sl5("5")
		{	

		addLevel(sl1);
		addLevel(sl2);
		addLevel(sl3);
		addLevel(sl4);
		addLevel(sl5);
		
		sl1.addEvent(se1, sl2, kPublicEvent);
		sl1.addEvent(se2, sl4, kPublicEvent);
		sl1.addEvent(se4, sl5, kPublicEvent);
		sl2.addEvent(se1, sl3, kPublicEvent);
		addEventToLevelAndAbove(sl3, se3, sl1, kPublicEvent);
		addEventToAllLevelsBetween(sl4, sl5, se1, sl2, kPublicEvent);

// 		slOff.setLevelAction([&](SafetyContext* privateContext) {Executor::stop();});
// 		slShuttingDown.setLevelAction([&](SafetyContext* privateContext) {privateContext->triggerEvent(seSwitchingOff);});
// 		slInitialized.setLevelAction([&](SafetyContext* privateContext) {
// 			if(slInitialized.getNofActivations() > 5){
// 				outTest->set(true);	// switch on in1 via sim-eeros
// 			}
// 		});
// 		slRunning.setLevelAction([&](SafetyContext* privateContext){
// 			if(slRunning.getNofActivations() > 5){
// 				outTest->set(false);	// switch off in1 via sim-eeros
// 			}
// 		});
		// Define entry level
		setEntryLevel(sl1);
	}
	
	SafetyEvent se1, se2, se3, se4;	
	SafetyLevel sl1, sl2, sl3, sl4, sl5;
};


// Test level ordering
TEST(safetyLevelTest, order) {
	SafetyPropertiesTest sp;
	EXPECT_TRUE(sp.sl1 < sp.sl2);
	EXPECT_TRUE(sp.sl5 >= sp.sl2);
}

// Test event triggering
TEST(safetyLevelTest, trigger) {
	SafetyPropertiesTest sp;
	SafetySystem ss(sp, 1);
	EXPECT_TRUE(ss.getCurrentLevel() == sp.sl1);
	ss.triggerEvent(sp.se1);	// go sl2
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == sp.sl2);
	ss.triggerEvent(sp.se3);	// should stay
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == sp.sl2);
	ss.triggerEvent(sp.se2);	// should stay
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == sp.sl2);
	ss.triggerEvent(sp.se1);	// go sl3
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == sp.sl3);
	ss.triggerEvent(sp.se1);	// should stay
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == sp.sl3);
	ss.triggerEvent(sp.se3);	// go sl1
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == sp.sl1);

	ss.triggerEvent(sp.se2);
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == sp.sl4);
	ss.triggerEvent(sp.se3);	// go sl1
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == sp.sl1);

	ss.triggerEvent(sp.se4);
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == sp.sl5);
	ss.triggerEvent(sp.se3);	// go sl1
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == sp.sl1);

	ss.triggerEvent(sp.se2);
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == sp.sl4);
	ss.triggerEvent(sp.se1);	// go sl2
	ss.run();
	EXPECT_TRUE(ss.getCurrentLevel() == sp.sl2);
}
