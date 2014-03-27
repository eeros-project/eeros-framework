#ifndef ORG_EEROS_TEST_CONTROL_SUMBLOCKTEST_HPP_
#define ORG_EEROS_TEST_CONTROL_SUMBLOCKTEST_HPP_
#include <eeros/core/Runnable.hpp>
#include <eeros/control/Sum.hpp>

#include <iostream>
#include <fstream>

#include "../../Utils.hpp"

template <int N, typename T = double>
class SumBlockTest {
	public:
		SumBlockTest() {
			for (int i = 0; i < N; i++) {
				uut.getIn(i).connect(data[i]);
			}
		}
		
		int run(const char* filepath) {
			std::ifstream file(filepath);
			if (!file.is_open()) return -2;
			
			int line = 0;
			int error = 0;
			
			while (!file.eof()) {
				line++;
 				for (int i = 0; i < N; i++) {
					double in;
					file >> in; // read input data
 					data[i].getSignal().setValue(in);
 				}
				if (file.eof()) break;
				double out;
				file >> out; // read reference result
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
		eeros::control::Output<T> data[N];
		eeros::control::Sum<N> uut;
};

#endif // ORG_EEROS_TEST_CONTROL_SUMBLOCKTEST_HPP_
