#include "SequencerTest.hpp"
#include "MySequence.hpp"

CPPUNIT_TEST_SUITE_REGISTRATION(SequencerTest);

void SequencerTest::setUp(){
}

void SequencerTest::tearDown(){
}

void SequencerTest::testCallSequence(){
	//The first allocation of a sequencer is automatically the Main-Sequencer!!
	MySequencer mainSequencer("MainSequencer");
	MySequence sequence("MySequence", mainSequencer);

	//Thread erzeugen:
	mainSequencer.start();
	
	//Thread stoppen wird in Step Stopping gemacht
	//callerThread.stop();
	
	while(!mainSequencer.isTerminated()){
		std::cout << "waiting for executor to terminate..." << std::endl;
	}
	
	CPPUNIT_ASSERT(sequence.getCalledMethode().compare("InitInitialisingInitialisedHomedMoveStopping")==0);
}

MySequencer::MySequencer(std::string name) : eeros::sequencer::Sequencer(name){
}