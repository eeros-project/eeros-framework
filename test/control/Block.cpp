#include <eeros/control/Block.hpp>
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


TEST(controlBlockTest, Block0i0oFeatures) {
  class Block0i0oStub : public Blockio<0,0> {
   public:
    void run() {}
  };

  Block0i0oStub block{};
}


TEST(controlBlockTest, Block0i1oFeatures) {
  class Block0i1oStub : public Blockio<0,1> {
   public:
    void run() {}
  };

  Block0i1oStub block{};
  EXPECT_EQ(typeid(block.getOut()), typeid(Output<double>));
}


TEST(controlBlockTest, Block0i2oFeatures) {
  class Block0i2oStub : public Blockio<0,2> {
   public:
    void run() {}
  };

  Block0i2oStub block{};
  block.setName("b1");
  EXPECT_EQ(typeid(block.getOut(0)), typeid(Output<double>));
  EXPECT_EQ(typeid(block.getOut(1)), typeid(Output<double>));
  try {
    EXPECT_EQ(typeid(block.getOut(2)), typeid(Output<double>));
    FAIL();
  } catch(eeros::Fault const & err) {
    EXPECT_EQ(err.what(), std::string("Trying to get inexistent element of output vector in block 'b1'"));
  }
}


TEST(controlBlockTest, Block1i0oFeatures) {
  class Block1i0oStub : public Blockio<1,0> {
   public:
    void run() {}
  };

  Block1i0oStub block{};
  EXPECT_EQ(typeid(block.getIn()), typeid(Input<double>));
}


TEST(controlBlockTest, Block1i1oFeatures) {
  class Block1i1oStub : public Blockio<1,1> {
   public:
    void run() {}
  };

  Block1i1oStub block{};
  EXPECT_EQ(typeid(block.getIn()), typeid(Input<double>));
  EXPECT_EQ(typeid(block.getOut()), typeid(Output<double>));
}


TEST(controlBlockTest, Block1i2oFeatures) {
  class Block1i2oStub : public Blockio<1,2> {
   public:
    void run() {}
  };

  Block1i2oStub block{};
  block.setName("b2");
  EXPECT_EQ(typeid(block.getIn()), typeid(Input<double>));
  EXPECT_EQ(typeid(block.getOut(0)), typeid(Output<double>));
  EXPECT_EQ(typeid(block.getOut(1)), typeid(Output<double>));
  try {
    EXPECT_EQ(typeid(block.getOut(2)), typeid(Output<double>));
    FAIL();
  } catch(eeros::Fault const & err) {
    EXPECT_EQ(err.what(), std::string("Trying to get inexistent element of output vector in block 'b2'"));
  }
}


TEST(controlBlockTest, Block2i0oFeatures) {
  class Block2i0oStub : public Blockio<2,0> {
   public:
    void run() {}
  };

  Block2i0oStub block{};
  block.setName("b3");
  EXPECT_EQ(typeid(block.getIn(0)), typeid(Input<double>));
  EXPECT_EQ(typeid(block.getIn(1)), typeid(Input<double>));
  try {
    EXPECT_EQ(typeid(block.getIn(2)), typeid(Input<double>));
    FAIL();
  } catch(eeros::Fault const & err) {
    EXPECT_EQ(err.what(), std::string("Trying to get inexistent element of input vector in block 'b3'"));
  }
}


TEST(controlBlockTest, Block2i1oFeatures) {
  class Block2i1oStub : public Blockio<2,1> {
   public:
    void run() {}
  };

  Block2i1oStub block{};
  block.setName("b4");
  EXPECT_EQ(typeid(block.getIn(0)), typeid(Input<double>));
  EXPECT_EQ(typeid(block.getIn(1)), typeid(Input<double>));
  try {
    EXPECT_EQ(typeid(block.getIn(2)), typeid(Input<double>));
    FAIL();
  } catch(eeros::Fault const & err) {
    EXPECT_EQ(err.what(), std::string("Trying to get inexistent element of input vector in block 'b4'"));
  }
  EXPECT_EQ(typeid(block.getOut()), typeid(Output<double>));
}


TEST(controlBlockTest, Block2i2oFeatures) {
  class Block2i2oStub : public Blockio<2,2> {
   public:
    void run() {}
  };

  Block2i2oStub block{};
  block.setName("b5");
  EXPECT_EQ(typeid(block.getIn(0)), typeid(Input<double>));
  EXPECT_EQ(typeid(block.getIn(1)), typeid(Input<double>));
  try {
    EXPECT_EQ(typeid(block.getIn(2)), typeid(Input<double>));
    FAIL();
  } catch(eeros::Fault const & err) {
    EXPECT_EQ(err.what(), std::string("Trying to get inexistent element of input vector in block 'b5'"));
  }
  EXPECT_EQ(typeid(block.getOut(0)), typeid(Output<double>));
  EXPECT_EQ(typeid(block.getOut(1)), typeid(Output<double>));
  try {
    EXPECT_EQ(typeid(block.getOut(2)), typeid(Output<double>));
    FAIL();
  } catch(eeros::Fault const & err) {
    EXPECT_EQ(err.what(), std::string("Trying to get inexistent element of output vector in block 'b5'"));
  }
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
