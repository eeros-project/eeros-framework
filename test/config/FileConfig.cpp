#include <eeros/config/FileConfig.hpp>

#include <gtest/gtest.h>
#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>

namespace {
	class TestConfig : public eeros::config::FileConfig {
	public:
		TestConfig(const char *name) : FileConfig(name) {
			add("value1", value1);
			add("value2", value2);
			add("str", str);
		}
		
		virtual void loadDefaults() {
			value1 = 17;
			value2 = 584.25;
			str = "test string";
		}
		
		virtual void zero() {
			value1 = 0;
			value2 = 0.0;
			str = "";
		}
		
		int value1;
		double value2;
		std::string str;
	};
}

TEST(configFile, integer) {
	TestConfig config("defaultConfig.txt");
	config.loadDefaults();
	config.save();
	EXPECT_EQ(config.value1, 17);
	config.zero();
	EXPECT_EQ(config.value1, 0);
	config.load();
	EXPECT_EQ(config.value1, 17);
	config.value1 += 10;
	config.save("anotherConfig.txt");
	config.load();
	EXPECT_EQ(config.value1, 17);
	config.load("anotherConfig.txt");
	EXPECT_EQ(config.value1, 27);
}

TEST(configFile, string) {
	TestConfig config("defaultConfig.txt");
	config.loadDefaults();
	config.save();
	EXPECT_EQ(config.str, std::string("test string"));
	config.zero();
	EXPECT_EQ(config.str, std::string(""));
	config.load();
	EXPECT_EQ(config.str, std::string("test string"));
	config.str += " 1";
	config.save("anotherConfig.txt");
	config.load();
	EXPECT_EQ(config.str, std::string("test string"));
	config.load("anotherConfig.txt");
	EXPECT_EQ(config.str, std::string("test string 1"));
}

