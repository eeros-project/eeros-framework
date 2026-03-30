#include <eeros/control/SocketData.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/core/Fault.hpp>
#include <cmath>
#include <gtest/gtest.h>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;


// Test name
TEST(controlSocketDataTest, name) {
  std::cout.setstate(std::ios_base::badbit);
  eeros::logger::Logger::setDefaultStreamLogger(std::cout);
  SocketData<Vector2, Vector2> s("127.0.0.1", 9876, 0.1);
  EXPECT_STREQ (s.getName().c_str(), "");

  s.setName("socket data block");
  EXPECT_STREQ (s.getName().c_str(), "socket data block");
}

// Test allocation
TEST(controlSocketDataTest, allocClient) {
  std::cout.setstate(std::ios_base::badbit);
  eeros::logger::Logger::setDefaultStreamLogger(std::cout);
  SocketData<Vector2, int> s1("127.0.0.1", 9876, 0.1);
  SocketData<Vector2, Matrix<2,3,double>> s2("127.0.0.1", 9876, 0.1);
  SocketData<Vector2, Matrix<2,3,int>> s3("127.0.0.1", 9876, 0.1);
  SocketData<int, Vector2> s4("127.0.0.1", 9876, 0.1);
  SocketData<Matrix<2,3,double>, Vector2> s5("127.0.0.1", 9876, 0.1);
  SocketData<Matrix<2,3,int>, Vector2> s6("127.0.0.1", 9876, 0.1);
  SocketData<Matrix<2,3,bool>, bool> s7("127.0.0.1", 9876, 0.1);
  SocketData<Vector2, std::nullptr_t> s8("127.0.0.1", 9876, 0.1);
  SocketData<std::nullptr_t, Vector2> s9("127.0.0.1", 9876, 0.1);
  EXPECT_TRUE(true);
}

// Test allocation
TEST(controlSocketDataTest, allocServer) {
  std::cout.setstate(std::ios_base::badbit);
  eeros::logger::Logger::setDefaultStreamLogger(std::cout);
  SocketData<Vector2, int> s1("", 9800, 0.1);
  SocketData<Vector2, Matrix<2,3,double>> s2("", 9801, 0.1);
  SocketData<Vector2, Matrix<2,3,int>> s3("", 9802, 0.1);
  SocketData<int, Vector2> s4("", 9803, 0.1);
  SocketData<Matrix<2,3,double>, Vector2> s5("", 9804, 0.1);
  SocketData<Matrix<2,3,int>, Vector2> s6("", 9805, 0.1);
  SocketData<Matrix<2,3,bool>, bool> s7("", 9806, 0.1);
  SocketData<Vector2, std::nullptr_t> s8("", 9807, 0.1);
  SocketData<std::nullptr_t, Vector2> s9("", 9808, 0.1);
  EXPECT_TRUE(true);
}

// Test connection Matrix, Matrix
TEST(controlSocketDataTest, connect1) {
  std::cout.setstate(std::ios_base::badbit);
  eeros::logger::Logger::setDefaultStreamLogger(std::cout);
  SocketData<Matrix<6,1,double>, Vector4> client("127.0.0.1", 9876, 0.1);
  SocketData<Vector4, Matrix<6,1,double>>server("", 9876, 0.1);
  while (!client.isConnected() || !server.isConnected());
  EXPECT_TRUE(std::isnan(client.getOut().getSignal().getValue()[0]));
  EXPECT_TRUE(std::isnan(server.getOut().getSignal().getValue()[0]));
  Constant<Matrix<6,1,double>> c1({0.1, 0.2, 0.3, 0.4, 0.5, 0.6});
  Constant<Vector4> c2({-0.1, -0.2, -0.3, -0.4});
  client.getIn().connect(c1.getOut());
  server.getIn().connect(c2.getOut());
  c1.run();
  c2.run();
  client.run();
  server.run();
  client.resetNew();
  server.resetNew();
  while (!client.isNew() || !server.isNew());
  client.run();
  server.run();
  client.resetNew();
  server.resetNew();
  while (!client.isNew() || !server.isNew());
  EXPECT_EQ(server.getOut().getSignal().getValue()[4], 0.5);
  EXPECT_EQ(client.getOut().getSignal().getValue()[2], -0.3);
}

// Test connection Matrix<int>, Matrix
TEST(controlSocketDataTest, connect2) {
  std::cout.setstate(std::ios_base::badbit);
  eeros::logger::Logger::setDefaultStreamLogger(std::cout);
  SocketData<Matrix<6,1,int>, Vector4> client("127.0.0.1", 9876, 0.1);
  SocketData<Vector4, Matrix<6,1,int>>server("", 9876, 0.1);
  while (!client.isConnected() || !server.isConnected());
  EXPECT_TRUE(std::isnan(client.getOut().getSignal().getValue()[0]));
  EXPECT_EQ(server.getOut().getSignal().getValue()[0], std::numeric_limits<int>::min());
  Constant<Matrix<6,1,int>> c1({1, 2, 3, 4, 5, 6});
  Constant<Vector4> c2({-0.1, -0.2, -0.3, -0.4});
  client.getIn().connect(c1.getOut());
  server.getIn().connect(c2.getOut());
  c1.run();
  c2.run();
  client.run();
  server.run();
  client.resetNew();
  server.resetNew();
  while (!client.isNew() || !server.isNew());
  client.run();
  server.run();
  client.resetNew();
  server.resetNew();
  while (!client.isNew() || !server.isNew());
  EXPECT_EQ(server.getOut().getSignal().getValue()[4], 5);
  EXPECT_EQ(client.getOut().getSignal().getValue()[2], -0.3);
}

// Test connection double, Matrix
TEST(controlSocketDataTest, connect3) {
  std::cout.setstate(std::ios_base::badbit);
  eeros::logger::Logger::setDefaultStreamLogger(std::cout);
  SocketData<double, Vector4> client("127.0.0.1", 9876, 0.1);
  SocketData<Vector4, double>server("", 9876, 0.1);
  while (!client.isConnected() || !server.isConnected());
  EXPECT_TRUE(std::isnan(client.getOut().getSignal().getValue()[0]));
  EXPECT_TRUE(std::isnan(server.getOut().getSignal().getValue()));
  Constant<> c1{2.5};
  Constant<Vector4> c2({-0.1, -0.2, -0.3, -0.4});
  client.getIn().connect(c1.getOut());
  server.getIn().connect(c2.getOut());
  c1.run();
  c2.run();
  client.run();
  server.run();
  client.resetNew();
  server.resetNew();
  while (!client.isNew() || !server.isNew());
  client.run();
  server.run();
  client.resetNew();
  server.resetNew();
  while (!client.isNew() || !server.isNew());
  EXPECT_EQ(server.getOut().getSignal().getValue(), 2.5);
  EXPECT_EQ(client.getOut().getSignal().getValue()[2], -0.3);
}

// Test connection Matrix, int
TEST(controlSocketDataTest, connect4) {
  std::cout.setstate(std::ios_base::badbit);
  eeros::logger::Logger::setDefaultStreamLogger(std::cout);
  SocketData<Vector4, int> client("127.0.0.1", 9876, 0.1);
  SocketData<int, Vector4>server("", 9876, 0.1);
  while (!client.isConnected() || !server.isConnected());
  EXPECT_EQ(client.getOut().getSignal().getValue(), std::numeric_limits<int>::min());
  EXPECT_TRUE(std::isnan(server.getOut().getSignal().getValue()[0]));
  Constant<Vector4> c1({-0.1, -0.2, -0.3, -0.4});
  Constant<int> c2{-6};
  client.getIn().connect(c1.getOut());
  server.getIn().connect(c2.getOut());
  c1.run();
  c2.run();
  client.run();
  server.run();
  client.resetNew();
  server.resetNew();
  while (!client.isNew() || !server.isNew());
  client.run();
  server.run();
  client.resetNew();
  server.resetNew();
  while (!client.isNew() || !server.isNew());
  EXPECT_EQ(server.getOut().getSignal().getValue()[2], -0.3);
  EXPECT_EQ(client.getOut().getSignal().getValue(), -6);
}

// Test connection Matrix, nothing
TEST(controlSocketDataTest, connect5) {
  std::cout.setstate(std::ios_base::badbit);
  eeros::logger::Logger::setDefaultStreamLogger(std::cout);
  SocketData<Vector4, std::nullptr_t> client("127.0.0.1", 9876, 0.1);
  SocketData<std::nullptr_t, Vector4>server("", 9876, 0.1);
  while (!client.isConnected() || !server.isConnected());
  EXPECT_TRUE(std::isnan(server.getOut().getSignal().getValue()[0]));
  Constant<Vector4> c1({-0.1, -0.2, -0.3, -0.4});
  client.getIn().connect(c1.getOut());
  c1.run();
  client.run();
  server.run();
  server.resetNew();
  while (!server.isNew());
  client.run();
  server.run();
  server.resetNew();
  while (!server.isNew());
  EXPECT_EQ(server.getOut().getSignal().getValue()[2], -0.3);
}

// Test connection nothing, Matrix
TEST(controlSocketDataTest, connect6) {
  std::cout.setstate(std::ios_base::badbit);
  eeros::logger::Logger::setDefaultStreamLogger(std::cout);
  SocketData<std::nullptr_t, Vector4> client("127.0.0.1", 9876, 0.1);
  SocketData<Vector4, std::nullptr_t>server("", 9876, 0.1);
  while (!client.isConnected() || !server.isConnected());
  EXPECT_TRUE(std::isnan(client.getOut().getSignal().getValue()[0]));
  Constant<Vector4> c1({-0.1, -0.2, -0.3, -0.4});
  server.getIn().connect(c1.getOut());
  c1.run();
  client.run();
  server.run();
  client.resetNew();
  while (!client.isNew());
  client.run();
  server.run();
  client.resetNew();
  while (!client.isNew());
  EXPECT_EQ(client.getOut().getSignal().getValue()[2], -0.3);
}
