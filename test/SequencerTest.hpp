#ifndef ORG_EEROS_TEST_SEQUENCERTEST_HPP_
#define ORG_EEROS_TEST_SEQUENCERTEST_HPP_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

class SequencerTest : public CppUnit::TestFixture{
	
	CPPUNIT_TEST_SUITE(SequencerTest);
	CPPUNIT_TEST(testSimpleSequence);
	CPPUNIT_TEST(testSimpleSubSequence);
	CPPUNIT_TEST(testNonBlockingSubSequence);
	CPPUNIT_TEST_SUITE_END();
	
public:
	void setUp();	
	void tearDown();	
	void testSimpleSequence();
	void testSimpleSubSequence();
	void testNonBlockingSubSequence();
};

#endif