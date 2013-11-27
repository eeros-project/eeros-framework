#include "SequencerTest.hpp"
#include "MySequencer.hpp"
#include "MySequence.hpp"
#include "CallingSubSequence.hpp"
#include "CallingNonBlockingSubSequence.hpp"
#include "SequenceMoveException1_3.hpp"
#include "SequenceMoveException2a.hpp"
#include "SequenceMoveException2b.hpp"
#include "CallingNonBlockingSequence_ErrorHandler.hpp"
#include "SequenceMoveException6a.hpp"
#include "SequenceMoveException6b.hpp"

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
	
	eeros::sequencer::Sequence* subSequence = eeros::sequencer::Sequence::getSequence("BlockingSubSequence");
	if(subSequence){
		delete subSequence;
	}
}

void SequencerTest::testNonBlockingSubSequence(){
	MySequencer mainSequencer("MainSequencer");
	//For 5th case (-> note case 5 (in my Folder)).
	//set restartSequencer = true to restart the sequencer
	//set restartSequencer = false to not restart the sequencer and not waiting
	CallingNonBlockingSubSequence sequence("CallingNonBlockingSubSequence", mainSequencer, false, false);
	//Thread erzeugen:
	mainSequencer.start();
	
	//Thread stoppen wird in Step Stopping gemacht
	//callerThread.stop();
	
	while(!mainSequencer.isTerminated()){
		//std::cout << "waiting for executor to terminate..." << std::endl;
	}
	
	CPPUNIT_ASSERT(sequence.getCalledMethode().compare("Init Initialising Initialised Homed Move MoveToA MoveToB MoveToC Stop Wait Stopping") == 0);
	
	eeros::sequencer::Sequencer* seq = 0;
	try{
		seq = eeros::sequencer::Sequencer::getMainSequencer()->findSequencer("SubSequencer");
	}catch(const char * e){
	}
	if(seq){
		delete seq;
	}
	eeros::sequencer::Sequence* subSequence = eeros::sequencer::Sequence::getSequence("NonBlockingSubSequence");
	if(subSequence){
		delete subSequence;
	}
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
	//3. the sequence continues "Homed Move MoveException Stopping"
	//   in this run the sequencer will terminate normally
	std::string s3 = "Homed Move MoveException Stopping";
	CPPUNIT_ASSERT(sequence.getCalledMethode().compare(s1.append(s2.append(s3))) == 0);
}

void SequencerTest::testErrorHandlerCase5(){
	MySequencer mainSequencer("MainSequencer");
	CallingNonBlockingSequence_ErrorHandler sequence("CallingNonBlockingSequence_ErrorHandler", mainSequencer, false);
	
	//Thread erzeugen:
	mainSequencer.start();
	
	//Thread stoppen wird in Step Stopping gemacht
	//callerThread.stop();
	
	while(!mainSequencer.isTerminated()){
		//std::cout << "waiting for executor to terminate..." << std::endl;
	}
	//after the exception the sequence is restaarted
	//1. all the steps until the exception occurs where called first "Init Initialising Initialised Homed Move MoveException "
	std::string s1 = "Init Initialising Initialised Homed MoveException ";
	//2. the ErrorHandler is called (without ErrorHandlerB) "Reset Referencing Restart "
	std::string s2 = "Reset Restart ";
	//3. the sequence restarts"
	//   in this run the sequencer will terminate normally
	std::string s3 = "Init Initialising Initialised Homed MoveException ";
	//4. NonBlockingSubSequence ends here before the main sequence stopping is from the main sequncer
	std::string s4 = "MoveToA MoveToB MoveToC Stop Wait Stopping";
	
	//std::string str1 = sequence.getCalledMethode();
	std::string str2;
	str2.append(s1.append(s2.append(s3.append(s4))));
	
	CPPUNIT_ASSERT(sequence.getCalledMethode().compare(str2) == 0);
	
	eeros::sequencer::Sequencer* seq = 0;
	try{
		seq = eeros::sequencer::Sequencer::getMainSequencer()->findSequencer("SubSequencer");
	}catch(const char * e){
	}
	if(seq){
		delete seq;
	}
	eeros::sequencer::Sequence* subSequence = eeros::sequencer::Sequence::getSequence("NonBlockingSubSequence");
	if(subSequence){
		delete subSequence;
	}
}

void SequencerTest::testErrorHandlerCase5Restart(){
	MySequencer mainSequencer("MainSequencer");
	CallingNonBlockingSequence_ErrorHandler sequence("CallingNonBlockingSequence_ErrorHandler", mainSequencer, true);
	CallingNonBlockingSequence_ErrorHandler::i = 0;
	
	//Thread erzeugen:
	mainSequencer.start();
	
	//Thread stoppen wird in Step Stopping gemacht
	//callerThread.stop();
	
	while(!mainSequencer.isTerminated()){
		//std::cout << "waiting for executor to terminate..." << std::endl;
	}
	
	//after the exception the sequence is restaarted
	//1. all the steps until the exception occurs where called first "Init Initialising Initialised Homed Move MoveException "
	std::string s1 = "Init Initialising Initialised Homed MoveException ";
	//2. the ErrorHandler is called (without ErrorHandlerB) "Reset Referencing Restart "
	std::string s2 = "Reset Restart ";
	//3. the sequence restarts"
	//   in this run the sequencer will terminate normally
	std::string s3 = "Init Initialising Initialised Homed MoveException ";
	//4. NonBlockingSubSequence ends here before the main sequence stopping is from the main sequncer
	std::string s4 = "MoveToA MoveToB MoveToC Stop ";
	std::string s5 = "Wait Stopping";
	
	//std::string str1 = sequence.getCalledMethode();
	std::string str2;
	str2.append(s1.append(s2.append(s3.append(s4))));
	str2.append(s4.append(s5));
	
	CPPUNIT_ASSERT(sequence.getCalledMethode().compare(str2) == 0);
	
	
	eeros::sequencer::Sequencer* seq = 0;
	try{
		seq = eeros::sequencer::Sequencer::getMainSequencer()->findSequencer("SubSequencer");
	}catch(const char * e){
	}
	if(seq){
		delete seq;
	}
	eeros::sequencer::Sequence* subSequence = eeros::sequencer::Sequence::getSequence("NonBlockingSubSequence");
	if(subSequence){
		delete subSequence;
	}
}

void SequencerTest::testErrorHandlerCase6a(){
	MySequencer mainSequencer("MainSequencer");
	SequenceMoveException6a sequence("SequenceMoveException6a", mainSequencer);
	SequenceMoveException6a::i = 0;
	
	//Thread erzeugen:
	mainSequencer.start();
	
	//Thread stoppen wird in Step Stopping gemacht
	//callerThread.stop();
	
	while(!mainSequencer.isTerminated()){
		//std::cout << "waiting for executor to terminate..." << std::endl;
	}
	
	//after the exception the sequence is restaarted
	//1. all the steps until the exception occurs where called first "Init Initialising Initialised Homed Move MoveException "
	std::string s1 = "Init Initialising Initialised Homed MoveException ";
	//2. the ErrorHandler is called (without ErrorHandlerB) "Reset Referencing Restart "
	std::string s2 = "Reset Restart ";
	//3. the sequence continues"
	//   in this run the sequencer will terminate normally
	std::string s3 = "MoveException ";
	//4. NonBlockingSubSequence ends here before the main sequence stopping is from the main sequncer
	std::string s4 = "MoveToA MoveToB MoveToC Stop ";
	std::string s5 = "Wait Stopping";
	
	//std::string str1 = sequence.getCalledMethode();
	std::string str2;
	str2.append(s1.append(s2.append(s3.append(s4))));
	str2.append(s5);
	
	CPPUNIT_ASSERT(sequence.getCalledMethode().compare(str2) == 0);
	
	
	eeros::sequencer::Sequencer* seq = 0;
	try{
		seq = eeros::sequencer::Sequencer::getMainSequencer()->findSequencer("SubSequencer");
	}catch(const char * e){
	}
	if(seq){
		delete seq;
	}
	eeros::sequencer::Sequence* subSequence = eeros::sequencer::Sequence::getSequence("NonBlockingSubSequence");
	if(subSequence){
		delete subSequence;
	}
}

void SequencerTest::testErrorHandlerCase6b(){
	MySequencer mainSequencer("MainSequencer");
	SequenceMoveException6b sequence("SequenceMoveException6b", mainSequencer);
	SequenceMoveException6b::i = 0;
	
	//Thread erzeugen:
	mainSequencer.start();
	
	//Thread stoppen wird in Step Stopping gemacht
	//callerThread.stop();
	
	while(!mainSequencer.isTerminated()){
		//std::cout << "waiting for executor to terminate..." << std::endl;
	}
	
	//after the exception the sequence is restaarted
	//1. all the steps until the exception occurs where called first "Init Initialising Initialised Homed Move MoveException "
	std::string s1 = "Init Initialising Initialised Homed MoveException ";
	//2. the ErrorHandler is called (without ErrorHandlerB) "Reset Referencing Restart "
	std::string s2 = "Reset Restart ";
	//3. the sequence restarts on the homed method"
	//   in this run the sequencer will terminate normally
	std::string s3 = "Homed MoveException ";
	//4. NonBlockingSubSequence ends here before the main sequence stopping is from the main sequncer
	std::string s4 = "MoveToA MoveToB MoveToC Stop ";
	std::string s5 = "Wait Stopping";
	
	//std::string str1 = sequence.getCalledMethode();
	std::string str2;
	str2.append(s1.append(s2.append(s3.append(s4))));
	str2.append(s5);
	
	CPPUNIT_ASSERT(sequence.getCalledMethode().compare(str2) == 0);
	
	
	eeros::sequencer::Sequencer* seq = 0;
	try{
		seq = eeros::sequencer::Sequencer::getMainSequencer()->findSequencer("SubSequencer");
	}catch(const char * e){
	}
	if(seq){
		delete seq;
	}
	eeros::sequencer::Sequence* subSequence = eeros::sequencer::Sequence::getSequence("NonBlockingSubSequence");
	if(subSequence){
		delete subSequence;
	}
}

void SequencerTest::testErrorHandlerCase7(){
	MySequencer mainSequencer("MainSequencer");
	//For 5th case (-> note case 5 (in my Folder)).
	//set restartSequencer = true to restart the sequencer
	//set restartSequencer = false to not restart the sequencer and not waiting
	CallingNonBlockingSubSequence sequence("CallingNonBlockingSubSequence", mainSequencer, false, true);
	//Thread erzeugen:
	mainSequencer.start();
	
	//Thread stoppen wird in Step Stopping gemacht
	//callerThread.stop();
	
	while(!mainSequencer.isTerminated()){
		//std::cout << "waiting for executor to terminate..." << std::endl;
	}
	//1. Sequencer starts
	std::string s1 ="Init Initialising Initialised ";
	//2. SubSequnecer with Exception starts
	std::string s2 ="Init MoveException Reset Restart ";
	//3. SubSequencer restarts
	std::string s3 ="Init MoveException ";
	std::string s4 ="Homed Stopping";
	//Sequencer ends
	std::string s5 ="Homed Move Wait Stopping";
	
	std::string str = s1.append(s2.append(s3.append(s4.append(s5))));
	
	CPPUNIT_ASSERT(sequence.getCalledMethode().compare(str) == 0);
	
	eeros::sequencer::Sequencer* seq = 0;
	try{
		seq = eeros::sequencer::Sequencer::getMainSequencer()->findSequencer("SubSequencer");
	}catch(const char * e){
	}
	if(seq){
		delete seq;
	}
	eeros::sequencer::Sequence* subSequence = eeros::sequencer::Sequence::getSequence("NonBlockingSubSequence");
	if(subSequence){
		delete subSequence;
	}
}