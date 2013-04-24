#include <cppunit/TestSuite.h>
#include <cppunit/TestCaller.h>
#include <cppunit/ui/text/TestRunner.h>

#include "SignalTest.hpp"

int main( int argc, char **argv)
{
    CppUnit::TestSuite suite;

    suite.addTest(new CppUnit::TestCaller<SignalTest>("testConstructor", &SignalTest::testGetSet));
    //suite.addTest(new CppUnit::TestCaller<SignalTest>("testEquality", &SignalTest::testEquality));
    
    CppUnit::TextUi::TestRunner runner;
    runner.addTest(&suite);
    runner.run();
    return 0;
}