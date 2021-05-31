#include <eeros/control/Block.hpp>
#include <eeros/control/Block1i.hpp>
#include <eeros/control/Block1o.hpp>
#include <eeros/control/Block1i1o.hpp>
#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>

#include <gtest/gtest.h>


using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;


TEST(controlBlockTest, BlockFeatures) {
  class BlockStub : public Block {
   public:
    void run() {}
  };

  BlockStub block{};
  EXPECT_STREQ (block.getName().c_str(), "");

  block.setName("THE test Block");
  EXPECT_STREQ (block.getName().c_str(), "THE test Block");
}


TEST(controlBlockTest, Block1iFeatures) {
  class Block1iStub : public Block1i<> {
   public:
    void run() {}
  };

  Block1iStub block1i{};
  EXPECT_EQ(typeid(block1i.getIn()), typeid(Input<double>));
}


TEST(controlBlockTest, Block1oFeatures) {
  class Block1oStub : public Block1o<> {
   public:
    void run() {}
  };

  Block1oStub block1o{};
  EXPECT_EQ(typeid(block1o.getOut()), typeid(Output<double>));
}


TEST(controlBlockTest, Block1i1oFeatures) {
  class Block1i1oStub : public Block1i1o<> {
   public:
    void run() {}
  };

  Block1i1oStub block1i1o{};
  EXPECT_EQ(typeid(block1i1o.getIn()), typeid(Input<double>));
  EXPECT_EQ(typeid(block1i1o.getOut()), typeid(Output<double>));
}

TEST(controlBlockTest, BlockioFeatures) {
  class BlockioStub1 : public Blockio<2,2> { };
  BlockioStub1 s1{};
  EXPECT_EQ(typeid(s1.getIn(0)), typeid(Input<double>));
  EXPECT_EQ(typeid(s1.getIn(1)), typeid(Input<double>));
  EXPECT_EQ(typeid(s1.getOut(0)), typeid(Output<double>));
  EXPECT_EQ(typeid(s1.getOut(1)), typeid(Output<double>));
  
  class BlockioStub2 : public Blockio<2,2,Vector2> { };
  BlockioStub2 s2{};
  EXPECT_EQ(typeid(s2.getIn(0)), typeid(Input<Vector2>));
  EXPECT_EQ(typeid(s2.getIn(1)), typeid(Input<Vector2>));
  EXPECT_EQ(typeid(s2.getOut(0)), typeid(Output<Vector2>));
  EXPECT_EQ(typeid(s2.getOut(1)), typeid(Output<Vector2>));
  
  class BlockioStub3 : public Blockio<2,2,Vector2,double> { };
  BlockioStub3 s3{};
  EXPECT_EQ(typeid(s3.getIn(0)), typeid(Input<Vector2>));
  EXPECT_EQ(typeid(s3.getIn(1)), typeid(Input<Vector2>));
  EXPECT_EQ(typeid(s3.getOut(0)), typeid(Output<double>));
  EXPECT_EQ(typeid(s3.getOut(1)), typeid(Output<double>));

  class BlockioStub4 : public Blockio<2,1,double,Vector2> { };
  BlockioStub4 s4{};
  EXPECT_EQ(typeid(s4.getIn(0)), typeid(Input<double>));
  EXPECT_EQ(typeid(s4.getIn(1)), typeid(Input<double>));
  EXPECT_EQ(typeid(s4.getOut()), typeid(Output<Vector2>));
  
  class BlockioStub5 : public Blockio<1,2,double,Vector2> { };
  BlockioStub5 s5{};
  EXPECT_EQ(typeid(s5.getIn()), typeid(Input<double>));
  EXPECT_EQ(typeid(s5.getOut(0)), typeid(Output<Vector2>));
  EXPECT_EQ(typeid(s5.getOut(1)), typeid(Output<Vector2>));
  
  class BlockioStub6 : public Blockio<1,1> { };
  BlockioStub6 s6{};
  EXPECT_EQ(typeid(s6.getIn()), typeid(Input<double>));
  EXPECT_EQ(typeid(s6.getOut()), typeid(Output<double>));

  class BlockioStub7 : public Blockio<1,0,Vector2> { };
  BlockioStub7 s7{};
  EXPECT_EQ(typeid(s7.getIn()), typeid(Input<Vector2>));

  class BlockioStub8 : public Blockio<0,1,double,Vector2> { };
  BlockioStub8 s8{};
  EXPECT_EQ(typeid(s8.getOut()), typeid(Output<Vector2>));
}

TEST(controlBlockTest, BlockioFunction) {
  class BlockioStub1 : public Blockio<2,2> { };
  BlockioStub1 s1{};
  EXPECT_EQ(typeid(s1.getIn(0)), typeid(Input<double>));
  EXPECT_EQ(typeid(s1.getIn(1)), typeid(Input<double>));
  EXPECT_EQ(typeid(s1.getOut(0)), typeid(Output<double>));
  EXPECT_EQ(typeid(s1.getOut(1)), typeid(Output<double>));
}

