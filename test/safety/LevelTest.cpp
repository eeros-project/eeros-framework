#include <eeros/safety/SafetySystem.hpp>
#include <eeros/core/Fault.hpp>
#include <gtest/gtest.h>

using namespace eeros;
using namespace eeros::safety;
using namespace eeros::logger;

class SafetyPropertiesTest1 : public SafetyProperties {
 public:
  SafetyPropertiesTest1()  
      : se1("se1"), se2("se2"), se3("se3"), se4("se4"),
        sl1("1"), sl2("2"), sl3("3"), sl4("4"), sl5("5") {
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

    setEntryLevel(sl1);
  }
  
  SafetyEvent se1, se2, se3, se4;	
  SafetyLevel sl1, sl2, sl3, sl4, sl5;
};


// Test level ordering
TEST(safetyLevelTest, order) {
  SafetyPropertiesTest1 sp;
  EXPECT_TRUE(sp.sl1 < sp.sl2);
  EXPECT_TRUE(sp.sl5 >= sp.sl2);
}

// Test event triggering
TEST(safetyLevelTest, trigger) {
  std::cout.setstate(std::ios_base::badbit);
  logger::Logger::setDefaultStreamLogger(std::cout);
  SafetyPropertiesTest1 sp;
  SafetySystem ss(sp, 1);
  ss.run();
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

class ScaraSafetyProperties : public eeros::safety::SafetyProperties {
public:
  ScaraSafetyProperties() :
    off                        ("off"                                              ),
    notinit_emergency          ("emergency, not init"                              ),
    notinit_waitingForApproval ("waitingForApproval, not init, press WHITE button" ),
    notinit_systemOn           ("systemOn, not init"                               ),
    manualParking3             ("manualParking 3" ),
    manualParking2             ("manualParking 2" ),
    manualParking1             ("manualParking 1" ),
    manualParking0             ("manualParking 0" ),
    robotParked                ("robotParked"     ),
    homing3                    ("homing 3"                                 ),
    homing2                    ("homing 2"                                 ),
    homing1                    ("homing 1"                                 ),
    homing0                    ("homing 0"                                 ),
    robotHomed                 ("robot homed, press GREEN button to start" ),
    emergency                  ("Emergency"            ),
    resetEmergency             ("Emergency reset"      ),
    waitingForApproval         ("Waiting for approval" ),
    set_autoParking            ("Set Autoparking axes" ),
    autoParking_shutdown3      ("Autoparking q3"       ),
    autoParking_shutdown2      ("Autoparking q2"       ),
    autoParking_shutdown1      ("Autoparking q1"       ),
    autoParking_shutdown0      ("Autoparking q0"       ),
    systemOn                   ("System on"            ),
    powerOn                    ("Power on"             ),
    goingToReady               ("System going to ready"),
    ready                      ("System ready"         ),
    set_moving                 ("Set Moving"           ),
    moving                     ("Moving"               ),
    set_moving_joystick        ("Set Moving joystick"  ),
    moving_joystick            ("Moving joystick"      ),
    notinit_goToWaitingForApproval ("notinit_goToWaitingForApproval"),

    doOff                          ("doOff"),
    approvalIsOn                   ("approvalIsOn"),
    doManualParking                ("doManualParking          "),
    manualParkingDone3             ("manualParkingDone3       "),
    manualParkingDone2             ("manualParkingDone2       "),
    manualParkingDone1             ("manualParkingDone1       "),
    manualParkingDone0             ("manualParkingDone0       "),
    doHoming                       ("doHoming                 "),
    homingDone3                    ("homingDone3              "),
    homingDone2                    ("homingDone2              "),
    homingDone1                    ("homingDone1              "),
    homingDone0                    ("homingDone0              "),
    doSystemOn                     ("doSystemOn               "),
    doPowerUp                      ("doPowerUp                "),
    doPowerDown                    ("doPowerDown              "),
    goToReady                      ("goToReady                "),
    isReady                        ("isReady                  "),
    doEmergency                    ("doEmergency              "),
    doResetEmergency               ("doResetEmergency         "),
    resetEmergencyDone             ("resetEmergencyDone       "),
    doEmergency_posOutRange        ("doEmergency_posOutRange  "),
    doEmergency_velOutRange        ("doEmergency_velOutRange  "),
    doSetMoving                    ("doSetMotion              "),
    doStartMoving                  ("doStartMotion            "),
    doStopMoving                   ("doStopMotion             "),
    doSetMoving_joystick           ("doSetMoving_joystick     "),
    doStartMoving_joystick         ("doStartMoving_joystick   "),
    doStopMoving_joystick          ("doStopMoving_joystick    "),
    doAutoParking_shutdown         ("doAutoParking_shutdown   "),
    doStartAutoParking             ("doStartAutoParking       "),
    autoParking_shutdownDone3      ("autoParking_shutdownDone3"),
    autoParking_shutdownDone2      ("autoParking_shutdownDone2"),
    autoParking_shutdownDone1      ("autoParking_shutdownDone1"),
    autoParking_shutdownDone0      ("autoParking_shutdownDone0"),
    ev1                            ("event 1                  "),
    ev2                            ("event 2                  ")
  {
    addLevel(off                        );
    addLevel(notinit_emergency          );
    addLevel(notinit_waitingForApproval );
    addLevel(notinit_systemOn           );
    addLevel(manualParking3             );
    addLevel(manualParking2             );
    addLevel(manualParking1             );
    addLevel(manualParking0             );
    addLevel(robotParked                );
    addLevel(homing3                    );
    addLevel(homing2                    );
    addLevel(homing1                    );
    addLevel(homing0                    );
    addLevel(robotHomed                 );
    addLevel(emergency                  );
    addLevel(resetEmergency             );
    addLevel(waitingForApproval         );
    addLevel(systemOn                   );
    addLevel(powerOn                    );
    addLevel(goingToReady               );
    addLevel(ready                      );
    addLevel(set_moving                 );
    addLevel(moving                     );
    addLevel(set_moving_joystick        );
    addLevel(moving_joystick            );
    addLevel(set_autoParking            );
    addLevel(autoParking_shutdown3      );
    addLevel(autoParking_shutdown2      );
    addLevel(autoParking_shutdown1      );
    addLevel(autoParking_shutdown0      );

    off                        .addEvent(notinit_goToWaitingForApproval, notinit_waitingForApproval, kPublicEvent  );
    off                        .addEvent(ev1,                            homing3,                    kPublicEvent  );
    notinit_waitingForApproval .addEvent(approvalIsOn,                   notinit_systemOn,           kPublicEvent  );
    notinit_systemOn           .addEvent(doManualParking,                manualParking3,             kPublicEvent  );
    notinit_emergency          .addEvent(doOff,                          off,                        kPrivateEvent );
    notinit_emergency          .addEvent(ev2,                            moving,                     kPublicEvent );
    manualParking3             .addEvent(manualParkingDone3,             manualParking2,             kPrivateEvent );
    manualParking2             .addEvent(manualParkingDone2,             manualParking1,             kPrivateEvent );
    manualParking1             .addEvent(manualParkingDone1,             manualParking0,             kPrivateEvent );
    manualParking0             .addEvent(manualParkingDone0,             robotParked,                kPrivateEvent );
    robotParked                .addEvent(doHoming,                       homing3,                    kPublicEvent  );
    homing3                    .addEvent(homingDone3,                    homing2,                    kPrivateEvent );
    homing2                    .addEvent(homingDone2,                    homing1,                    kPrivateEvent );
    homing1                    .addEvent(homingDone1,                    homing0,                    kPrivateEvent );
    homing0                    .addEvent(homingDone0,                    robotHomed,                 kPrivateEvent );
    robotHomed                 .addEvent(doSystemOn,                     systemOn,                   kPublicEvent  );
    emergency                  .addEvent(doResetEmergency,               resetEmergency,             kPublicEvent  );
    resetEmergency             .addEvent(resetEmergencyDone,             waitingForApproval,         kPrivateEvent );
    waitingForApproval         .addEvent(approvalIsOn,                   systemOn,                   kPublicEvent  );
    systemOn                   .addEvent(doOff,                          off,                        kPublicEvent  );
    systemOn                   .addEvent(doPowerUp,                      powerOn,                    kPublicEvent  );
    powerOn                    .addEvent(doPowerDown,                    systemOn,                   kPublicEvent  );
    powerOn                    .addEvent(goToReady,                      goingToReady,               kPublicEvent  );
    goingToReady               .addEvent(isReady,                        ready,                      kPublicEvent  );
    ready                      .addEvent(doSetMoving_joystick,           set_moving_joystick,        kPublicEvent  );
    ready                      .addEvent(doSetMoving,                    set_moving,                 kPublicEvent  );
    ready                      .addEvent(doAutoParking_shutdown,         set_autoParking,            kPublicEvent  );
    set_moving                 .addEvent(doStartMoving,                  moving,                     kPublicEvent  );
    moving                     .addEvent(doStopMoving,                   ready,                      kPublicEvent  );
    set_moving_joystick        .addEvent(doStartMoving_joystick,         moving_joystick,            kPublicEvent  );
    moving_joystick            .addEvent(doStopMoving_joystick,          ready,                      kPublicEvent  );
    set_autoParking            .addEvent(doStartAutoParking,             autoParking_shutdown3,      kPrivateEvent );
    autoParking_shutdown3      .addEvent(autoParking_shutdownDone3,      autoParking_shutdown2,      kPrivateEvent );
    autoParking_shutdown2      .addEvent(autoParking_shutdownDone2,      autoParking_shutdown1,      kPrivateEvent );
    autoParking_shutdown1      .addEvent(autoParking_shutdownDone1,      autoParking_shutdown0,      kPrivateEvent );
    autoParking_shutdown0      .addEvent(autoParking_shutdownDone0,      off,                        kPrivateEvent );

    // Add events to multiple levels
    addEventToAllLevelsBetween(off,            robotHomed,            doEmergency, notinit_emergency, kPublicEvent );
    addEventToAllLevelsBetween(resetEmergency, autoParking_shutdown0, doEmergency, emergency,         kPublicEvent );

    off.setLevelAction([this](SafetyContext* privateContext) { });

    notinit_emergency.setLevelAction([](SafetyContext* privateContext) { });

    homing3.setLevelAction([this](SafetyContext* privateContext) {
      privateContext->triggerEvent(doEmergency);
    });

    moving.setLevelAction([this](SafetyContext* privateContext) {
      if (moving.getNofActivations() > 2)
        privateContext->triggerEvent(doEmergency);
    });

    setEntryLevel(off);
  }

  SafetyLevel off;
  SafetyLevel notinit_emergency;
  SafetyLevel notinit_waitingForApproval;
  SafetyLevel notinit_systemOn;
  SafetyLevel manualParking3, manualParking2, manualParking1, manualParking0;
  SafetyLevel robotParked;
  SafetyLevel homing3, homing2, homing1, homing0;
  SafetyLevel robotHomed;
  SafetyLevel emergency;
  SafetyLevel resetEmergency;
  SafetyLevel waitingForApproval;
  SafetyLevel set_autoParking;
  SafetyLevel autoParking_shutdown3, autoParking_shutdown2, autoParking_shutdown1, autoParking_shutdown0;
  SafetyLevel systemOn;
  SafetyLevel powerOn;
  SafetyLevel goingToReady;
  SafetyLevel ready;
  SafetyLevel set_moving;
  SafetyLevel moving;
  SafetyLevel set_moving_joystick;
  SafetyLevel moving_joystick;

  SafetyEvent notinit_goToWaitingForApproval;
  SafetyEvent doOff;
  SafetyEvent approvalIsOn;
  SafetyEvent doManualParking;
  SafetyEvent manualParkingDone3, manualParkingDone2, manualParkingDone1, manualParkingDone0;
  SafetyEvent doHoming;
  SafetyEvent homingDone3, homingDone2, homingDone1, homingDone0;
  SafetyEvent doSystemOn;
  SafetyEvent doPowerUp;
  SafetyEvent doPowerDown;
  SafetyEvent goToReady;
  SafetyEvent isReady;
  SafetyEvent doEmergency;
  SafetyEvent doResetEmergency;
  SafetyEvent resetEmergencyDone;
  SafetyEvent doEmergency_posOutRange;
  SafetyEvent doEmergency_velOutRange;
  SafetyEvent doSetMoving;
  SafetyEvent doStartMoving;
  SafetyEvent doStopMoving;
  SafetyEvent doSetMoving_joystick;
  SafetyEvent doStartMoving_joystick;
  SafetyEvent doStopMoving_joystick;
  SafetyEvent doAutoParking_shutdown;
  SafetyEvent doStartAutoParking;
  SafetyEvent autoParking_shutdownDone3, autoParking_shutdownDone2, autoParking_shutdownDone1, autoParking_shutdownDone0;
  SafetyEvent ev1, ev2;
};

// Test event triggering
TEST(safetyLevelTest, trigger2) {
  ScaraSafetyProperties sp;
  SafetySystem ss(sp, 1);
  ss.run();
  EXPECT_TRUE(ss.getCurrentLevel() == sp.off);
  ss.triggerEvent(sp.ev1);
  ss.run();
  ss.run();
  EXPECT_TRUE(ss.getCurrentLevel() == sp.notinit_emergency);
  ss.triggerEvent(sp.ev2);
  ss.run();
  EXPECT_TRUE(ss.getCurrentLevel() == sp.moving);
  ss.run();
  ss.run();
  ss.run();
  EXPECT_TRUE(ss.getCurrentLevel() == sp.emergency);
}


