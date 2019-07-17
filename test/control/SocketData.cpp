#include <eeros/control/SocketData.hpp>
#include <eeros/core/Fault.hpp>
#include <gtest/gtest.h>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;


// Test name
TEST(controlSocketDataTest, nan) {
  SocketData<Vector2, Vector2> s("192.168.1.1", 9876);
  ASSERT_STREQ (s.getName().c_str(), "");

  s.setName("socket data block");
  ASSERT_STREQ (s.getName().c_str(), "socket data block");
}
