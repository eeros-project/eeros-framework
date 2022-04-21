// #include <eeros/control/filter/KalmanFilter.hpp>
#include <eeros/core/Fault.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/control/Constant.hpp>
#include <gtest/gtest.h>
#include <Utils.hpp>

using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;

// Test naming
TEST(controlKLFTest, naming) {
//   KalmanFilter<1,1,2,2> f({1,1,1,1},{1,1},{1,1},{1,1,1,1},{1,1,1,1},{1});
//   EXPECT_EQ(f.getName(), std::string(""));
//   f.setName("kalman filter 1");
//   EXPECT_EQ(f.getName(), std::string("kalman filter 1"));
}

// Test initial values for NaN
TEST(controlKLFTest, initialValue) {
//   KalmanFilter<2,double> m;
//   m.setName("mux");
//   EXPECT_TRUE(std::isnan(m.getOut().getSignal().getValue()[0]));
//   EXPECT_TRUE(std::isnan(m.getOut().getSignal().getValue()[1]));
//   try {
//     m.run();
//     FAIL();
//   } catch(eeros::Fault const & err) {
//     EXPECT_EQ(err.what(), std::string("Read from an unconnected input in block 'mux'"));
//   }
}

// Test inexisting input
TEST(controlKLFTest, input) {
//   KalmanFilter<2,double> m;
//   m.setName("mux");
//   m.getIn(1);
//   try {
//     m.getIn(2);
//     FAIL();
//   } catch(eeros::Fault const & err) {
//     EXPECT_EQ(err.what(), std::string("Trying to get inexistent element of input vector in block 'mux'"));
//   }
}


