#include <eeros/control/Gain.hpp>
#include <iostream>
#include <fstream>
#include "../../Utils.hpp"

template <typename Tin, typename Tout, typename Tgain>
class GainBlockTest {
	public:
		GainBlockTest() {
			uut.getIn().connect(data);
		}
		
		int run(const char* filepath) {
			std::ifstream file(filepath);
			if(!file.is_open()) return -100;
			
			int line = 0;
			int error = 0;
			
			while(!file.eof()) {
				line++;
				
				// Read test value
				Tin testValue;
				file >> testValue;
				data.getSignal().setValue(testValue);
				if(file.eof()) break;
				
				// Read gain
				Tgain testGain;
				file >> testGain;
				uut.setGain(testGain);
				if(file.eof()) break;
				
				// Read reference result
				Tout refRes;
				file >> refRes;
				if(file.eof()) break;
				
				uut.run();

				if(!Utils::compareApprox(refRes, uut.getOut().getSignal().getValue(), 0.001)) {
					error++;
					std::cout << "line " << line << " expecting " << refRes << " calculated " << uut.getOut().getSignal().getValue() << std::endl;
				}
			}
			file.close();
			return error;
		}
		
	protected:
		eeros::control::Output<Tout> data;
		eeros::control::Gain<Tout, Tgain> uut;
};

int main(int argc, char* argv[]) {
	
	GainBlockTest<int, int, int>			tester_i_i_i;
	GainBlockTest<double, double, double>	tester_d_d_d;
	
	if(argc == 3) {
		char t = *(argv[1]);
		switch(t) {
			case 'i':
				return tester_i_i_i.run(argv[2]);
				
			case 'd':
				return tester_d_d_d.run(argv[2]);
			default:
				std::cout << "Illegal argument (" << t << "), use 'd' for double or 'i' for integer!" << std::endl;
				return -3;
		}
	}
	else {
		std::cout << "Illegal number of arguments!" << std::endl;
		return -2;
	}
	return -1;
}
