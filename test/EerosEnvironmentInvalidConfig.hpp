#ifndef ORG_EEROS_TEST_ENVIRONMENT_INVALID_CONFIG_HPP_
#define ORG_EEROS_TEST_ENVIRONMENT_INVALID_CONFIG_HPP_

#include <string>
#include <gtest/gtest.h>

namespace eeros {
	namespace test {
		
		class EerosEnvironmentInvalidConfig : public  testing::Environment {
		public:
			virtual void SetUp();
			virtual void TearDown();
			
		private:
			
		};

	};
};

#endif /* ORG_EEROS_TEST_ENVIRONMENT_INVALID_CONFIG_HPP_ */