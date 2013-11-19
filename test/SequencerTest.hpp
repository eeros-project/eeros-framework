#ifndef ORG_EEROS_TEST_SEQUENCERTEST_HPP_
#define ORG_EEROS_TEST_SEQUENCERTEST_HPP_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

class SequencerTest : public CppUnit::TestFixture{
	
	CPPUNIT_TEST_SUITE(SequencerTest);
	CPPUNIT_TEST(testSimpleSequence);
	CPPUNIT_TEST(testSimpleSubSequence);
	CPPUNIT_TEST(testNonBlockingSubSequence);
	CPPUNIT_TEST(testErrorHandlerCase1_3);
	CPPUNIT_TEST(testErrorHandlerCase2a);
	CPPUNIT_TEST(testErrorHandlerCase2b);
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
	void testErrorHandlerCase2a();
	void testErrorHandlerCase2b();
};

#endif