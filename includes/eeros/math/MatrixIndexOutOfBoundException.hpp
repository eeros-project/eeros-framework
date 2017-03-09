#ifndef ORG_EEROS_CORE_MATRIXINDEXOUTOFBOUNDEXCEPTION_HPP
#define ORG_EEROS_CORE_MATRIXINDEXOUTOFBOUNDEXCEPTION_HPP

#include <eeros/core/Fault.hpp>
#include <string>

namespace eeros {
	namespace math {
		class MatrixIndexOutOfBoundException : public eeros::Fault {

		public:
			MatrixIndexOutOfBoundException(unsigned int i, unsigned int I);
			MatrixIndexOutOfBoundException(unsigned int m, unsigned int M, unsigned int n, unsigned int N);
			MatrixIndexOutOfBoundException(unsigned int m, unsigned int M, unsigned int n, unsigned int N, std::string postfix);
			virtual ~MatrixIndexOutOfBoundException() throw();
		};
	};
};

#endif // ORG_EEROS_CORE_MATRIXINDEXOUTOFBOUNDEXCEPTION_HPP
