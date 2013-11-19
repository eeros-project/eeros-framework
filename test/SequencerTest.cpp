#include "SequencerTest.hpp"
#include "MySequencer.hpp"
#include "MySequence.hpp"
#include "CallingSubSequence.hpp"
#include "CallingNonBlockingSubSequence.hpp"
#include "SequenceMoveException1_3.hpp"
#include "SequenceMoveException2a.hpp"
#include "SequenceMoveException2b.hpp"

CPPUNIT_TEST_SUITE_REGISTRATION(SequencerTest);

void SequencerTest::setUp(){
}

void SequencerTest::tearDown(){
}

void SequencerTest::testSimpleSequence(){
	//The first allocation of a sequencer is automatically the Main-Sequencer!!
	MySequencer mainSequencer("MainSequencer");
	MySequence sequence("MySequence", mainSequencer);

	//Thread erzeugen:
	mainSequencer.start();
	
	//Thread stoppen wird in Step Stopping gemacht
	//callerThread.stop();
	
	while(!mainSequencer.isTerminated()){
		//std::cout << "waiting for executor to terminate..." << std::endl;
	}
	
	CPPUNIT_ASSERT(sequence.getCalledMethode().compare("Init Initialising Initialised Homed Move Stopping") == 0);
}

void SequencerTest::testSimpleSubSequence(){
	MySequencer mainSequencer("MainSequencer");
	CallingSubSequence sequence("CallingSubSequence", mainSequencer);
	//Thread erzeugen:
	mainSequencer.start();
	
	//Thread stoppen wird in Step Stopping gemacht
	//callerThread.stop();
	
	while(!mainSequencer.isTerminated()){
		//std::cout << "waiting for executor to terminate..." << std::endl;
	}
	
	CPPUNIT_ASSERT(sequence.getCalledMethode().compare("Init Initialising Initialised Homed Move MoveToA MoveToB MoveToC Stopping") == 0);
}

void SequencerTest::testNonBlockingSubSequence(){
	MySequencer mainSequencer("MainSequencer");
	CallingNonBlockingSubSequence sequence("CallingNonBlockingSubSequence", mainSequencer);
	//Thread erzeugen:
	mainSequencer.start();
	
	//Thread stoppen wird in Step Stopping gemacht
	//callerThread.stop();
	
	while(!mainSequencer.isTerminated()){
		//std::cout << "waiting for executor to terminate..." << std::endl;
	}
	
	CPPUNIT_ASSERT(sequence.getCalledMethode().compare("Init Initialising Initialised Homed Move MoveToA MoveToB MoveToC Stop Wait Stopping") == 0);
}

void SequencerTest::testErrorHandlerCase1_3(){
	MySequencer mainSequencer("MainSequencer");
	SequenceMoveException1_3 sequence("SequenceMoveException1_3", mainSequencer);
	
	//Thread erzeugen:
	mainSequencer.start();
	
	//Thread stoppen wird in Step Stopping gemacht
	//callerThread.stop();
	
	while(!mainSequencer.isTerminated()){
		//std::cout << "waiting for executor to terminate..." << std::endl;
	}
	
	//after the exceptionn the sequence is restarted agein after the exception is handled!
	//1. all the steps until the exception occurs where called first "Init Initialising Initialised Homed Move MoveException "
	std::string s1 = "Init Initialising Initialised Homed Move MoveException ";
	//2. the ErrorHandler is called "Reset Referencing ResetB ReferencingB Restart "
	std::string s2 = "Reset Referencing ResetB ReferencingB Restart ";
	//3. the sequence is restarted "Init Initialising Initialised Homed Move MoveException Stopping"
	//   in this run the sequencer will terminate normally
	std::string s3 = "Init Initialising Initialised Homed Move MoveException Stopping";
	CPPUNIT_ASSERT(sequence.getCalledMethode().compare(s1.append(s2.append(s3))) == 0);
}

void SequencerTest::testErrorHandlerCase2a(){
	MySequencer mainSequencer("MainSequencer");
	SequenceMoveException2a sequence("SequenceMoveException2a", mainSequencer);
	
	//Thread erzeugen:
	mainSequencer.start();
	
	//Thread stoppen wird in Step Stopping gemacht
	//callerThread.stop();
	
	while(!mainSequencer.isTerminated()){
		//std::cout << "waiting for executor to terminate..." << std::endl;
	}
	//after the exceptionn the sequence continue in the same method
	//1. all the steps until the exception occurs where called first "Init Initialising Initialised Homed Move MoveException "
	std::string s1 = "Init Initialising Initialised Homed Move MoveException ";
	//2. the ErrorHandler is called (without ErrorHandlerB) "Reset Referencing Restart "
	std::string s2 = "Reset Restart ";
	//3. the sequence continues "MoveException Stopping"
	//   in this run the sequencer will terminate normally
	std::string s3 = "MoveException Stopping";
	CPPUNIT_ASSERT(sequence.getCalledMethode().compare(s1.append(s2.append(s3))) == 0);
}

void SequencerTest::testErrorHandlerCase2b(){
	MySequencer mainSequencer("MainSequencer");
	SequenceMoveException2b sequence("SequenceMoveException2b", mainSequencer);
	
	//Thread erzeugen:
	mainSequencer.start();
	
	//Thread stoppen wird in Step Stopping gemacht
	//callerThread.stop();
	
	while(!mainSequencer.isTerminated()){
		//std::cout << "waiting for executor to terminate..." << std::endl;
	}
	//after the exception the sequence continues in the method homed of SequenceMoveException2b
	//1. all the steps until the exception occurs where called first "Init Initialising Initialised Homed Move MoveException "
	std::string s1 = "Init Initialising Initialised Homed Move MoveException ";
	//2. the ErrorHandler is called (without ErrorHandlerB) "Reset Referencing Restart "
	std::string s2 = "Reset Restart ";
	//3. the sequence continues "MoveException Stopping"
	//   in this run the sequencer will terminate normally
	std::string s3 = "Homed Move MoveException Stopping";
	CPPUNIT_ASSERT(sequence.getCalledMethode().compare(s1.append(s2.append(s3))) == 0);
}