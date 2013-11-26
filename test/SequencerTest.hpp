#ifndef ORG_EEROS_TEST_SEQUENCERTEST_HPP_
#define ORG_EEROS_TEST_SEQUENCERTEST_HPP_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

class SequencerTest : public CppUnit::TestFixture{
	
	CPPUNIT_TEST_SUITE(SequencerTest);
	//CPPUNIT_TEST(testSimpleSequence);
	CPPUNIT_TEST(testSimpleSubSequence);
	CPPUNIT_TEST(testNonBlockingSubSequence);
	CPPUNIT_TEST(testErrorHandlerCase1_3);
	CPPUNIT_TEST(testErrorHandlerCase2a);
	CPPUNIT_TEST(testErrorHandlerCase2b);
	CPPUNIT_TEST(testErrorHandlerCase5);
	CPPUNIT_TEST(testErrorHandlerCase5Restart);
	//CPPUNIT_TEST(testErrorHandlerCase6a);
	//CPPUNIT_TEST(testErrorHandlerCase6b);
	CPPUNIT_TEST_SUITE_END();
	
public:
	void setUp();	
	void tearDown();	
	void testSimpleSequence();
	void testSimpleSubSequence();
	void testNonBlockingSubSequence();
	//Cases corresponds to the WIKI: http://wiki.ntb.ch/collaboration/eeros/developer_documentation/sequencer/error_handler
	void testErrorHandlerCase1_3();
	//from this point no second ErrorHandler (A and B) is tested, because it is similar to testErrorHandlerCase1_3
	//Also for case 4a
	void testErrorHandlerCase2a();
	//Also for case 4b
	void testErrorHandlerCase2b();
	//case 5
	void testErrorHandlerCase5();
	void testErrorHandlerCase5Restart();
	//case 6
	void testErrorHandlerCase6a();
	void testErrorHandlerCase6b();
};

#endif