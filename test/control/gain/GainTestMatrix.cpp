#include <eeros/math/Matrix.hpp>
#include <eeros/control/Gain.hpp>
#include <iostream>
#include <fstream>
#include "../../Utils.hpp"
#include <type_traits>


template <typename Tin, typename Tout, typename Tgain, bool elementwise = false>
class GainBlockTest {
	public:
		GainBlockTest() {
			uut.getIn().connect(data);
		}
		
		int run(const char* filepath) {
			std::ifstream file(filepath);
			if(!file.is_open()) {
				std::cout << "Failed to open file!" << std::endl;
				return -100;
			}
			
			int line = 0;
			int error = 0;
			
			while(!file.eof()) {
				line++;
				
				// Read test value
				Tin testValue;
				for(int i = 0; i < testValue.getNofRows()  * testValue.getNofColums(); i++) {
					double in;
					file >> in;
					testValue[i] = in;
 				}
				data.getSignal().setValue(testValue);
				if(file.eof()) break;
				
				// Read gain
				Tgain testGain;
				for(int i = 0; i < testGain.getNofRows()  * testGain.getNofColums(); i++) {
					double gain;
					file >> gain;
					testGain[i] = gain;
 				}
 				uut.setGain(testGain);
				if(file.eof()) break;
				
				// Read reference results
				Tout refRes;
				for(int i = 0; i < refRes.getNofRows()  * refRes.getNofColums(); i++) {
					double res;
					file >> res;
					refRes[i] = res;
				}
				if(file.eof()) break;
				
				uut.run();

				for(int i = 0; i < refRes.getNofRows()  * refRes.getNofColums(); i++) {
					if(!Utils::compareApprox(refRes[i], uut.getOut().getSignal().getValue()[i], 0.001)) {
						error++;
						std::cout << "line " << line << " expecting " << refRes[i] << " calculated " << uut.getOut().getSignal().getValue()[i] << std::endl;
					}
				}
			}
			file.close();
			return error;
		}
		
	protected:
		eeros::control::Output<Tin> data;
		eeros::control::Gain<Tout, Tgain, elementwise> uut;
};

int main(int argc, char* argv[]) {
	
	using namespace eeros::math;

	GainBlockTest<Matrix<3, 1>, Matrix<3, 1>, Matrix<3, 1>, true>	tester_m3x1_m3x1_m3x1;
	GainBlockTest<Matrix<3, 1>, Matrix<3, 1>, Matrix<3, 3>>			tester_m3x1_m3x1_m3x3;
	GainBlockTest<Matrix<2, 2>, Matrix<2, 2>, Matrix<2, 2>>			tester_m2x2_m2x2_m2x2;
//	GainBlockTest<Matrix<3, 2>, Matrix<4, 2>, Matrix<4, 3>>			tester_m3x2_m4x2_m4x3;
	
	if(argc == 6) {
		unsigned int m_in = atoi(argv[1]);
		unsigned int n_in = atoi(argv[2]);
		unsigned int m_gain = atoi(argv[3]);
		unsigned int n_gain = atoi(argv[4]);
		
		if(m_in == 3 && n_in == 1 && m_gain == 3 && n_gain == 1) {
			return tester_m3x1_m3x1_m3x1.run(argv[5]);
		}
		else if(m_in == 3 && n_in == 1 && m_gain == 3 && n_gain == 3) {
			return tester_m3x1_m3x1_m3x3.run(argv[5]);
		}
		else if(m_in == 2 && n_in == 2 && m_gain == 2 && n_gain == 2) {
			return tester_m2x2_m2x2_m2x2.run(argv[5]);
		}
// 		else if(m_in == 3 && n_in == 2 && m_gain == 4 && n_gain == 3) {
// 			return tester_m3x2_m4x2_m4x3.run(argv[5]);
// 		}
		else {
			std::cout << "Test with given dimension is not available." << std::endl;
			return -3;
		}
	}
	else {
		std::cout << "Illegal number of arguments!" << std::endl;
		return -2;
	}
	return -1;
}
