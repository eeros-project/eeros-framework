#include <eeros/control/Block.hpp>

using namespace eeros::control;

void Block::setName(std::string name) {
	this->name = name;
}

std::string Block::getName() const {
	return name;
}