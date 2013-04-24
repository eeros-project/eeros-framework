#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include "AnSignal.hpp"

class SignalTest : public CppUnit::TestFixture
{
private:
    AnSignal *unnamedScalarSignal, *namedScalarSignal, *namedVectorSignal;

public:
    void setUp();
    void tearDown();
    void testGetSet();
	void testEquality();
	void testCompatibility();
};