#include <eeros/control/Block.hpp>
#include <eeros/control/Block1i.hpp>
#include <eeros/control/Block1o.hpp>
#include <eeros/control/Block1i1o.hpp>

#include <gtest/gtest.h>


using namespace eeros;
using namespace eeros::control;


TEST(controlBlockTest, BlockFeatures) {
  class BlockStub : public Block {
   public:
    void run() {}
  };

  BlockStub block{};
  ASSERT_STREQ (block.getName().c_str(), "");

  block.setName("THE test Block");
  ASSERT_STREQ (block.getName().c_str(), "THE test Block");
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
