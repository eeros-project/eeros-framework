#include <eeros/control/SocketData.hpp>
#include <eeros/core/Fault.hpp>
#include <gtest/gtest.h>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;


// Test initial values for NaN
TEST(controlSocketDataTest, name) {
	SocketData<Vector2, double, Vector2, double> s(9876);
	EXPECT_EQ(s.getName(), std::string(""));
	s.setName("socket data block");
	EXPECT_EQ(s.getName(), std::string("socket data block"));
}
