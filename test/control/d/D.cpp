#include <eeros/core/Runnable.hpp>
#include <eeros/control/D.hpp>

#include <iostream>
#include <fstream>

#include "../Utils.hpp"

template <typename T = double>
class DBlockTest {
	public:
		DBlockTest() {
			uut.getIn().connect(data);
		}
		
		int run(const char* filepath) {
			std::ifstream file(filepath);
			if (!file.is_open()) return -2;
			
			int line = 0;
			int error = 0;
			uint64_t timestamp = 0;
			
			while (!file.eof()) {
				line++;
				double in;
				double out;
				
				file >> in; // read input data
				file >> out; // read reference result
				
				if (file.eof()) break;
				
				data.getSignal().setValue(in);
				data.getSignal().setTimestamp(timestamp);
				timestamp += 1000000;
				
				uut.run();

				if(!Utils::compareApprox(out, uut.getOut().getSignal().getValue(), 0.001)) {
					error++;
					std::cout << "line " << line << " expecting " << out << " calculated " << uut.getOut().getSignal().getValue() << std::endl;
				}
			}
			
			file.close();
			return error;
		}
		
	protected:
		eeros::control::Output<T> data;
		eeros::control::D<T> uut;
};

int main(int argc, char* argv[]) {
	DBlockTest<> tester;
	if (argc == 2) {
		tester.run(argv[1]);
	}
	else {
		std::cout << "illegal number of arguments" << std::endl;
	}
	return -3;
}
