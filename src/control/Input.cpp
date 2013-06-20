#include <eeros/control/Input.hpp>

bool Input::connect(Output& output) {
// 	if(output) {
		connectedOutput = &output;
		return true;
// 	}
// 	return false;
}

void Input::disconnect() {
	connectedOutput = 0;
}

bool Input::isConnected() {
	return static_cast<bool>(connectedOutput);
}
