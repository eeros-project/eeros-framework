#include <gtest/gtest.h>

#include <getopt.h>
// #include <unistd.h>
#include <TestVariables.hpp>
#include <EerosEnvironment.hpp>
#include <EerosEnvironmentInvalidConfig.hpp>
#include <eeros/core/Fault.hpp>

/**
 * 
 * execute all unit tests with comedilib: 	./unitTests  --library comedi
 * or with short option: 			./unitTests -l comedi
 * execute all unit tests with flinklib: 	./unitTests  --library flink
 * execute hal unit tests with comedilib: 	./unitTests --library comedi --gtest_filter=hal*
 * or with short option:			./unitTests -l comedi --gtest_filter=hal*
 * execute math unit tests: 			./unitTests --gtest_filter=math*
 * 
 * */

bool libcomedi = false;		// comedi-eeros
bool libflink = false;		// flink-eeros
bool libsim = false;		// sim-eeros

using namespace eeros::test;

int main(int argc, char **argv){

	// available long_options
	static struct option long_options[] =
	{
	    {"library", 	required_argument, NULL, 'l'},
	    {"gtest_filter", 	required_argument, NULL, 'g'},
	    {"config", 		required_argument, NULL, 'c'},
	    {NULL, 		0, 		   NULL,  0 }
	};
  
	// create copy of argv
	int copyArgc = argc;
	auto copyArgv = new char*[copyArgc+1];
	for(int i=0; i <= copyArgc; i++) {
		copyArgv[i] = argv[i];
	}
	
	// Error message if long dashes (en dash) are used
	int i;
	for (i=0; i < copyArgc; i++) {
		 if ((copyArgv[i][0] == 226) && (copyArgv[i][1] == 128) && (copyArgv[i][2] == 147)) {
			fprintf(stderr, "Error: Invalid arguments. En dashes are used.\n");
			return -1;
		 }
	}
	
	// Compute command line arguments
	int c;
	std::string libStr;
	while ((c = getopt_long(copyArgc, copyArgv, "l:g:c:", long_options, NULL)) != -1) {
		switch(c) {
			case 'l': // lib found
				if(optarg){
					libStr = optarg;
					if(libStr == "comedi"){
						libcomedi = true;
					}
					else if(libStr == "flink"){
						libflink = true;
					}
					else if(libStr == "sim"){
						libsim = true;
					}
					else{
						throw eeros::Fault("unknown library given as argument: " + libStr);
					}
				}
				else{
					throw eeros::Fault("optarg empty!");
				}
				break;
			case 'g':
				//ignore -> googleTest argument
				break;
			case 'c':
				//ignore -> hal config argument
				break;
			case '?':
				std::cout << "unknown argument" << std::endl;
				break;
			default:
				// ignore all other args
				break;
		}
	}
	
	delete [] copyArgv;
	
	if (libflink || libcomedi || libsim) {
		// environment with invalid config for tests
		testing::AddGlobalTestEnvironment(new EerosEnvironmentInvalidConfig);
		
		// environment with valid config for tests
		testing::AddGlobalTestEnvironment(new EerosEnvironment);
	}
	// init googleTest and run
	testing::InitGoogleTest(&argc, argv);
		
	return RUN_ALL_TESTS();
  
	
}