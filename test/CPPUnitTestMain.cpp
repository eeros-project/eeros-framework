#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/ui/text/TestRunner.h>

//Dieser Code ist generisch, falls man mit Macros arbeitet
//D.h. das Main ist immer dasselbe
int main(){
	CppUnit::TextUi::TestRunner runner;
	CppUnit::TestFactoryRegistry &registry = CppUnit::TestFactoryRegistry::getRegistry();
	runner.addTest(registry.makeTest());
	bool wasSuccesful = runner.run();
	return wasSuccesful;
}