#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include "Signal.hpp"

class SignalTest : public CppUnit::TestFixture
{
private:
    Signal *unnamedScalarSignal, *namedScalarSignal, *namedVectorSignal;

public:
    void setUp();
    void tearDown();
    void testConstructor();
};