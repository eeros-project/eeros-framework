#include "SignalTest.hpp"

void SignalTest::setUp()
{
    unnamedScalarSignal = new Signal();
    namedScalarSignal = new Signal("namedScalarSignal", "m", "A");
    namedVectorSignal = new Signal();
}
    
void SignalTest::tearDown()
{
    delete unnamedScalarSignal;
    delete namedScalarSignal;
    delete namedVectorSignal;
}
    
void SignalTest::testConstructor()
{
    CPPUNIT_ASSERT(unnamedScalarSignal->getValue() == 0);
}
