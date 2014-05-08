#ifndef ORG_EEROS_TEST_CONTROL_SUMBLOCKTEST_HPP_
#define ORG_EEROS_TEST_CONTROL_SUMBLOCKTEST_HPP_
#include <eeros/core/Runnable.hpp>
#include <eeros/control/Sum.hpp>

#include <iostream>
#include <fstream>

#include "../../Utils.hpp"

template <unsigned int I, typename T>
class SumBlockTestScalar {
	public:
		SumBlockTestScalar() {
			for (int i = 0; i < I; i++) {
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
 				for (int i = 0; i < I; i++) {
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
		eeros::control::Output<T> data[I];
		eeros::control::Sum<I, T> uut;
};

template <unsigned int I, unsigned int M, unsigned int N, typename T>
class SumBlockTestMatrix {
	public:
		SumBlockTestMatrix() {
			for (int i = 0; i < I; i++) {
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
				
				T testValue;
 				for (int i = 0; i < I; i++) {
					for(unsigned int j = 0; j < M * N; j++) {
						double in;
						file >> in; // read input data
						testValue(j) = in;
					}
 					data[i].getSignal().setValue(testValue);
 				}
				if (file.eof()) break;
				T refRes;
				for(unsigned int j = 0; j < M * N; j++) {
					double out;
					file >> out; // read reference data
					refRes(j) = out;
				}
				
				uut.run();
				
				for(unsigned int j = 0; j < M * N; j++) {
					if(!Utils::compareApprox(refRes(j), uut.getOut().getSignal().getValue()(j), 0.001)) {
						error++;
						std::cout << "line " << line << " expecting " << refRes(j) << " calculated " << uut.getOut().getSignal().getValue()(j) << std::endl;
					}
				}
			}
			
			file.close();
			return error;
		}
		
	protected:
		eeros::control::Output<T> data[I];
		eeros::control::Sum<I, T> uut;
};

#endif // ORG_EEROS_TEST_CONTROL_SUMBLOCKTEST_HPP_
