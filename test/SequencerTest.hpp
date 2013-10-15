#ifndef ORG_EEROS_TEST_SEQUENCERTEST_HPP_
#define ORG_EEROS_TEST_SEQUENCERTEST_HPP_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include <eeros/sequencer/Sequencer.hpp>

class SequencerTest : public CppUnit::TestFixture{
	
	CPPUNIT_TEST_SUITE(SequencerTest);
	CPPUNIT_TEST(testSimpleSequence);
	CPPUNIT_TEST(testSimpleSubSequence);
	CPPUNIT_TEST_SUITE_END();
	
public:
	void setUp();	
	void tearDown();	
	void testSimpleSequence();
	void testSimpleSubSequence();
};



class MySequencer : public eeros::sequencer::Sequencer{
public:
	MySequencer(std::string name);
};

#endif