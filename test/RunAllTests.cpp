#include <gtest/gtest.h>

#include <getopt.h>
#include <TestVariables.hpp>
#include <EerosEnvironment.hpp>
#include <eeros/core/EEROSException.hpp>

/**
 * 
 * execute all unit tests with comedilib: ./unitTests  --library comedi
 * execute all unit tests with flinklib: ./unitTests  --library flink
 * execute hal unit tests with comedilib: ./unitTests --library comedi --gtest_filter=hal_*
 * execute math unit tests: ./unitTests --gtest_filter=math_*
 * 
 * */

bool libcomedi = false;
bool libflink = false;

using namespace eeros::test;

int main(int argc, char **argv){

	// available long_options
	static struct option long_options[] =
	{
	    {"library", optional_argument, NULL, 'l'},
	    {"gtest_filter", optional_argument, NULL, 'g'},
	    {NULL, 0, NULL, 0}
	};
  
	// create copy of argv
	int rArgc = argc;
	auto rArray = new char*[rArgc+1];
	for(int i=0; i <= rArgc; i++) {
		rArray[i] = argv[i];
	}

	// Error message if long dashes (en dash) are used
	int i;
	for (i=0; i < rArgc; i++) {
		 if ((rArray[i][0] == 226) && (rArray[i][1] == 128) && (rArray[i][2] == 147)) {
			fprintf(stderr, "Error: Invalid arguments. En dashes are used.\n");
			return -1;
		 }
	}
	
	/* Compute command line arguments */
	int c;
	std::string libStr;
	while ((c = getopt_long(rArgc, rArray, "l:g:", long_options, NULL)) != -1) {
		switch(c) {
			case 'l': // lib found
				libStr.assign(rArray[optind]);
				if(libStr == "comedi"){
					libcomedi = true;
				}
				else if(libStr == "flink"){
					libflink = true;
				}
				else{
					throw eeros::EEROSException("unknown library given as argument: " + libStr);
				}
				break;
			case 'g':
				//googleTest argument -> ignore
				break;
			default:
				// ignore all other args
				break;
		}
	}
	
	// cleanup copy of argv
	delete [] rArray;
	
	// init googleTest and run
	testing::Environment* const eeros_env = testing::AddGlobalTestEnvironment(new EerosEnvironment);
	
	::testing::InitGoogleTest(&argc, argv);
		
	return RUN_ALL_TESTS();
  
	
}