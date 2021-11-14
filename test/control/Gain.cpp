#include <eeros/control/Gain.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/math/Matrix.hpp>
#include <Utils.hpp>
#include <gtest/gtest.h>


using namespace eeros;
using namespace eeros::control;
  using namespace math;


TEST(controlGainTest, templateInstantiations) {
  Gain<> g1{};
  Gain<int> g2{};
  Gain<int,int> g3{};
  Gain<double,int> g4{};
  Gain<int,double> g5{};
  Gain<> g6{1.0};
  Gain<> g8{1.0, 2.0,-2.0};
  Gain<Matrix<2,2>,Matrix<2,2>,true> g9{};
  Gain<Matrix<1,3>,Matrix<3,3>> g10{};
  Gain<Matrix<1,3>,Matrix<1,3>,true> g11{};

  EXPECT_TRUE(true);
}


TEST(controlGainTest, simpleDoubleGain) {
  Gain<> g1{2.5};
  Constant<> c1{1.3};

  g1.getIn().connect(c1.getOut());
  c1.run();
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 3.25);

  g1.disable();
  g1.setGain(4.0);
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 1.3);

  g1.enable();
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 5.2);

  EXPECT_EQ (c1.getOut().getSignal().getTimestamp(), g1.getOut().getSignal().getTimestamp());
}


TEST(controlGainTest, simpleIntGain) {
  Gain<int,int> g1{2};
  Constant<int> c1{1};

  g1.getIn().connect(c1.getOut());
  c1.run();
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 2);

  g1.disable();
  g1.setGain(4.0);
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 1);

  g1.enable();
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 4);

  EXPECT_EQ (c1.getOut().getSignal().getTimestamp(), g1.getOut().getSignal().getTimestamp());
}


TEST(controlGainTest, smoothChangingDoubleGain) {
  Gain<> g1{};
  Constant<> c1{3.1};

  g1.getIn().connect(c1.getOut());
  c1.run();
  g1.setGain(1.8);
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 5.58);

  g1.enableSmoothChange(true); // gainDiff is 0 per default, so same results expected.
  g1.run();
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 5.58);
  g1.setGain(1.5);
  g1.run();
  g1.run();
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 5.58);

  // gain is at 1.8 and targetGain at 1.5 since smooth change is on!
  // => after one run gainDiff 0.5 will be subtracted, but this would overshoot the targetGain.
  // so gain will go to targeGain namely 1.5, so 3.1*1.5 = 4.65
  g1.setGainDiff(0.5);
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 4.65);
}


TEST(controlGainTest, smoothChangingIntGain) {
  Gain<int,int> g1{};
  Constant<int> c1{3};

  g1.getIn().connect(c1.getOut());
  c1.run();
  g1.setGain(3);
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 9);

  g1.enableSmoothChange(true); // gainDiff is 0 per default, so same results expected.
  g1.run();
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 9);
  g1.setGain(1);
  g1.run();
  g1.run();
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 9);

  // gain is at 3 and targetGain at 1 since smooth change is on!
  // => after one run gainDiff 1 will be subtracted.
  // so gain will go to 2; so 2*3 = 6
  g1.setGainDiff(1);
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 6);
}


TEST(controlGainTest, smoothChangingDoubleGainRealWorldTest) {
  Gain<> g1{5.6};
  Constant<> c1{1.4};
  g1.getIn().connect(c1.getOut());
  c1.run();

  g1.enableSmoothChange(true);
  g1.setGain(1.5); // which is targetGain in case smoothChange is on.
  g1.setGainDiff(0.01);
  g1.run(); // gain at 5.59
  g1.run(); // gain at 5.58
  g1.run(); // gain at 5.57
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 7.798); // 1,4 * 5.57

  g1.setGain(5.5);
  for(int i = 0 ; i< 7; i++){
    g1.run();
  }
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 7.7); // 1,4 * 5.5
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 7.7);

  g1.setGain(5.6);
  for(int i = 0 ; i< 10; i++){
    g1.run();
  }
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 7.84); // 1,4 * 5.6
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 7.84);
}


TEST(controlGainTest, minMaxDoubleGain) {
  Gain<> g1{2.5};
  Constant<> c1{1.3};

  g1.getIn().connect(c1.getOut());
  c1.run();
  g1.setMaxGain(1000);
  g1.setGain(10000); // should not change gain since higher than set maxGain
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 3.25);

  g1.setMinGain(-1000);
  g1.setGain(-10000); // should not change gain since lower than set minGain
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 3.25);

  g1.setMaxGain(100000);
  g1.setGain(10000);
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 13000);

  g1.setMinGain(-100000);
  g1.setGain(-10000);
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), -13000);
}


TEST(controlGainTest, minMaxIntGain) {
  Gain<int,int> g1{3};
  Constant<int> c1{1};

  g1.getIn().connect(c1.getOut());
  c1.run();
  g1.setMaxGain(1000);
  g1.setGain(10000); // should not change gain since higher than set maxGain
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 3);

  g1.setMinGain(-1000);
  g1.setGain(-10000); // should not change gain since lower than set minGain
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 3);

  g1.setMaxGain(100000);
  g1.setGain(10000);
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 10000);

  g1.setMinGain(-100000);
  g1.setGain(-10000);
  g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), -10000);
}


TEST(controlGainTest, smoothChangingDoubleGainMinMaxLimits) {
  Gain<> g1{8.5};
  Constant<> c1{5.2};
  g1.getIn().connect(c1.getOut());
  c1.run();

  g1.enableSmoothChange(true);
  g1.setGain(1.0);
  g1.setGainDiff(0.1);
  g1.setMinGain(8.0);
  for(int i = 0 ; i< 10; i++){
    g1.run(); // must not go below minGain
  }

  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 41.6);

  g1.setGain(10.0);
  g1.setGainDiff(0.1);
  g1.setMaxGain(8.5);
  for(int i = 0 ; i< 10; i++){
    g1.run(); // must not go above maxGain
  }

  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(), 44.2);
}


TEST(controlGainTest, simpleVectorGain1) {
  Constant<Matrix<1,2,double>> c1({1.0,2.0});
  Gain<Matrix<1,2,double>,Matrix<2,2,double>> g1({1,2,3,4});
  g1.getIn().connect(c1.getOut());
  c1.run();
  g1.run();
  // expecting scalar product  [1;2] x [1,2;3,4] = [5;11]
  auto res = g1.getOut().getSignal().getValue();
  EXPECT_DOUBLE_EQ (res[0],5);
  EXPECT_DOUBLE_EQ (res[1],11);
}


TEST(controlGainTest, simpleMatrixGain1) {
  Matrix<2,2> gM{2,0,1,2};
  Gain<Matrix<2,2>,Matrix<2,2>> g1{gM};
  Matrix<2,2> m1{1,2,3,4};
  Constant<Matrix<2,2>> c1{m1};
  g1.getIn().connect(c1.getOut());
  c1.run();
  g1.run();
  // expecting vector product [1,2;3,4] x [2,0;1,2] = [2,4;7,10]
  Matrix<2,2> res = g1.getOut().getSignal().getValue();
  EXPECT_DOUBLE_EQ (res[0],2);
  EXPECT_DOUBLE_EQ (res[1],4);
  EXPECT_DOUBLE_EQ (res[2],7);
  EXPECT_DOUBLE_EQ (res[3],10);
}


TEST(controlGainTest, simpleMatrixElementWiseGain1) {
  Matrix<2,2> gM{1,2,3,4};
  Gain<Matrix<2,2>,Matrix<2,2>,true> g1{gM};
  g1.enable();
  Matrix<2,2> m1{2,0,1,2};
  Constant<Matrix<2,2>> c1{m1};
  g1.getIn().connect(c1.getOut());
  c1.run();
  g1.run();
  // expecting element wise product [2,0;1,2]  [1,2;3,4] => [2,0;3,8]
  Matrix<2,2> res = g1.getOut().getSignal().getValue();
  EXPECT_DOUBLE_EQ (res[0],2);
  EXPECT_DOUBLE_EQ (res[1],0);
  EXPECT_DOUBLE_EQ (res[2],3);
  EXPECT_DOUBLE_EQ (res[3],8);
}


TEST(controlGainTest, simpleMatrixElementWiseGain2) {
  Matrix<3,2> gM{1,2,3, 1,4,9};
  Gain<Matrix<3,2>,Matrix<3,2>,true> g1{gM};
  Matrix<3,2> m1{2,4,6, 8,10,12};
  Constant<Matrix<3,2>> c1{m1};
  g1.getIn().connect(c1.getOut());
  c1.run();
  g1.run();
  Matrix<3,2> res = g1.getOut().getSignal().getValue();
  EXPECT_DOUBLE_EQ (res[0],2);
  EXPECT_DOUBLE_EQ (res[1],8);
  EXPECT_DOUBLE_EQ (res[2],18);
  EXPECT_DOUBLE_EQ (res[3],8);
  EXPECT_DOUBLE_EQ (res[4],40);
  EXPECT_DOUBLE_EQ (res[5],108);
}


TEST(controlGainTest, smoothChangingMatrixGainRealWorldTest) {
  Matrix<3,3> gM{1,2,3, 4,5,6, 10,9,8};
  Gain<Matrix<3,3>,Matrix<3,3>> g1{gM};
  Matrix<3,3> m1{1,2,3, 4,5,6, 7,8,9};
  Constant<Matrix<3,3>> c1{m1};
  g1.getIn().connect(c1.getOut());
  c1.run();
  g1.enableSmoothChange(true);
  g1.setGain(Matrix<3,3>{2,3,4, 5,6,7, 11, 10, 9}); // which is targetGain in case smoothChange is on.
  g1.setGainDiff(Matrix<3,3>{0.1,0.1,0.1, 0.1,0.1,0.1, 0.1,0.1,0.1});
  g1.run(); // gain at [1.1,2.1,3.1, 4.5,5.1,6.1, 10.1,9.1,8.1]
  g1.run();
  g1.run(); // gain at [1.3,2.3,3.3, 4.3,5.3,6.3, 10.3,9.3,8.3]
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[0], 33.6); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[1], 40.5); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[2], 47.4); 
  EXPECT_TRUE(Utils::compareApprox(g1.getOut().getSignal().getValue()[3], 69.599, 0.001));
  EXPECT_TRUE(Utils::compareApprox(g1.getOut().getSignal().getValue()[4], 85.499, 0.001));
  EXPECT_TRUE(Utils::compareApprox(g1.getOut().getSignal().getValue()[5], 101.399, 0.001));
  EXPECT_TRUE(Utils::compareApprox(g1.getOut().getSignal().getValue()[6], 105.599, 0.001));
  EXPECT_TRUE(Utils::compareApprox(g1.getOut().getSignal().getValue()[7], 133.5, 0.001));
  EXPECT_TRUE(Utils::compareApprox(g1.getOut().getSignal().getValue()[8], 161.399, 0.001));

  for(int i = 0 ; i< 7; i++){
    g1.run();
  }

  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[0], 42); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[1], 51); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[2], 60); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[3], 78); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[4], 96);
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[5], 114);
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[6], 114); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[7], 144);
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[8], 174);

  g1.run(); // should not change anything since reached targetGain
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[0], 42); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[1], 51); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[2], 60); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[3], 78); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[4], 96);
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[5], 114);
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[6], 114); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[7], 144);
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[8], 174);

  g1.setGain(Matrix<3,3>{1,2,3, 4,5,6, 10,9,8}); 

  for(int i = 0 ; i< 10; i++){
    g1.run();
  }

  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[0], 30); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[1], 36); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[2], 42); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[3], 66); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[4], 81);
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[5], 96);
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[6], 102); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[7], 129);
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[8], 156);

  g1.run(); // should not change anything since reached targetGain
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[0], 30); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[1], 36); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[2], 42); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[3], 66); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[4], 81);
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[5], 96);
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[6], 102); 
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[7], 129);
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[8], 156);
}


TEST(controlGainTest, smoothChangingMatrixGainMinMaxLimits) {
  Matrix<2,2> gM{1,2,3,4};
  Gain<Matrix<2,2>,Matrix<2,2>> g1{gM};
  Matrix<2,2> m1{1,2,3,4};
  Constant<Matrix<2,2>> c1{m1};
  g1.getIn().connect(c1.getOut());
  c1.run();
  g1.enableSmoothChange(true);
  g1.setGain(Matrix<2,2>{100,100,100,100});
  g1.setGainDiff(Matrix<2,2>{0.5,0.5,0.5,0.5});
  g1.setMaxGain(Matrix<2,2>{2,3,4,5});
  for(int i = 0 ; i< 10; i++){
    g1.run(); // must not go above maxGain
  }
  Matrix<2,2> res = g1.getOut().getSignal().getValue();
  EXPECT_DOUBLE_EQ (res[0],11);
  EXPECT_DOUBLE_EQ (res[1],16);
  EXPECT_DOUBLE_EQ (res[2],19);
  EXPECT_DOUBLE_EQ (res[3],28);  

  g1.setGain(Matrix<2,2>{-100,-100,-100,-100});
  g1.setMinGain(Matrix<2,2>{-2,-1,0,1});
  for(int i = 0 ; i< 10; i++){
    g1.run(); // must not go below minGain
  }
  Matrix<2,2> res2 = g1.getOut().getSignal().getValue();
  EXPECT_DOUBLE_EQ (res2[0],-5);
  EXPECT_DOUBLE_EQ (res2[1],-8);
  EXPECT_DOUBLE_EQ (res2[2],3);
  EXPECT_DOUBLE_EQ (res2[3],4);
}

TEST(controlGainTest, parabolicDouble) {
  Gain<> g1{5};
  Constant<> c1{1};
  g1.getIn().connect(c1.getOut());
  g1.enableParabolicGain(true);
  g1.setParabolicGainParams(2);
  c1.run(); g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(),5);
  c1.setValue(-1);
  c1.run(); g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(),-5);
  c1.setValue(0);
  c1.run(); g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(),0);
  c1.setValue(2);
  c1.run(); g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(),10);
  c1.setValue(5);
  c1.run(); g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(),20);
  c1.setValue(-5);
  c1.run(); g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(),-20);
  c1.setValue(1);
  g1.setGain(-5);
  c1.run(); g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(),-5);
  c1.setValue(-1);
  c1.run(); g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(),5);
  c1.setValue(5);
  c1.run(); g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(),-20);
  c1.setValue(-5);
  c1.run(); g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue(),20);
}

TEST(controlGainTest, parabolicVector1) {
  Constant<Vector2> c1({1,4});
  Gain<Vector2,double> g1{5};
  g1.getIn().connect(c1.getOut());
  g1.enableParabolicGain(true);
  g1.setParabolicGainParams({3,3});
  c1.run(); g1.run();
  EXPECT_DOUBLE_EQ (g1.getOut().getSignal().getValue()[0],5);
  EXPECT_TRUE(Utils::compareApprox(g1.getOut().getSignal().getValue()[1], 19.365, 0.001));
}

TEST(controlGainTest, parabolicVector2) {
  Constant<Vector2> c1({1,3});
  Gain<Vector2,Vector2,true> g1({3,4});
  g1.getIn().connect(c1.getOut());
  g1.enableParabolicGain(true);
  g1.setParabolicGainParams({2,2});
  c1.run(); g1.run();
  EXPECT_TRUE(Utils::compareApprox(g1.getOut().getSignal().getValue()[0], 3, 0.001));
  EXPECT_TRUE(Utils::compareApprox(g1.getOut().getSignal().getValue()[1], 11.314, 0.001));
}

TEST(controlGainTest, printGain) {
  Gain<> g{10.5};
  Constant<> c{5.2};
  g.setName("myGain");
  g.getIn().connect(c.getOut());
  c.run();

  g.enableSmoothChange(true);
  g.setGain(2.0);
  g.setGainDiff(0.1);
  g.setMaxGain(8.2);
  g.setMinGain(-8.0);
  g.enableParabolicGain(true);
  g.setParabolicGainParams(0.2);
  g.run(); // sets gain to 8.2 since above maxGain.
  
  std::stringstream sstream{};
  sstream << g;
  std::string str1 = "Block Gain: 'myGain' is enabled=1, gain=8.2, smoothChange=1, minGain=-8, maxGain=8.2, targetGain=2, gainDiff=0.1, parabolic=1, parabolicSwitchPoint=0.2";
  std::string str2 = sstream.str();
  EXPECT_STREQ (str1.c_str(), str2.c_str());
}
