#include "SignalTest.hpp"

#define TESTVECTORLENGTH 5

void SignalTest::setUp()
{
    unnamedScalarSignal = new AnSignal();
    namedScalarSignal = new AnSignal("namedScalarSignal", "m", "A");
    namedVectorSignal = new AnSignal();
}
    
void SignalTest::tearDown()
{
    delete unnamedScalarSignal;
    delete namedScalarSignal;
    delete namedVectorSignal;
}
    
void SignalTest::testGetSet()
{
	double tempValues[TESTVECTORLENGTH] = {0.5, 1.0, 2.0, 4.0, 8.0};
    CPPUNIT_ASSERT(unnamedScalarSignal->getValue() == 0);
    CPPUNIT_ASSERT(unnamedScalarSignal->getName() == "unnamed");
    CPPUNIT_ASSERT(unnamedScalarSignal->getUnit() == "");
    CPPUNIT_ASSERT(unnamedScalarSignal->getCoordinateSystem() == "");
	
	unnamedScalarSignal->setValue(0.1);
	namedScalarSignal->setValue(0.2);
	namedVectorSignal->setValue(tempValues);
	
    CPPUNIT_ASSERT(unnamedScalarSignal->getValue() == 0.1);
    CPPUNIT_ASSERT(namedScalarSignal->getValue() == 0.2);
	for(int i = 0; i < TESTVECTORLENGTH; i++)
	{
		CPPUNIT_ASSERT(namedVectorSignal->getValue(i) == tempValues[i]);
	}
}

void SignalTest::testEquality()
{
	
}