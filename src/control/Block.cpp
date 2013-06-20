#include <eeros/control/Block.hpp>

void Block::setName(std::string name) {
	this->name = name;
}

std::string Block::getName() {
	return name;
}