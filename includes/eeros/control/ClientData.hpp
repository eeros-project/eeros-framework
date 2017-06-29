#ifndef ORG_EEROS_CONTROL_CLIENTDATA_HPP_
#define ORG_EEROS_CONTROL_CLIENTDATA_HPP_

#include <eeros/control/Block1i1o.hpp>
#include <eeros/sockets/SocketClient.hpp>
#include <eeros/math/Matrix.hpp>
// #include <eeros/socket/Client.hpp>
// #include <eeros/control/Input.hpp>
// #include <eeros/control/Output.hpp>
#include <array>
#include <atomic>
// #include "../types.hpp"
// #include "../../socket/Client.hpp"


namespace eeros {
	namespace control {
	
		class ClientData: public eeros::control::Block {
			
		public:
			ClientData(Client* clientThread);
			
			virtual eeros::control::Input<Vector4>& getIn();
			virtual eeros::control::Output<Vector4>& getOut();
			
			virtual void run();
			
		protected:
			eeros::control::Input<Vector4> in;
			eeros::control::Output<Vector4> out;
			Client* client;
		};
	};
}

#endif /* ORG_EEROS_CONTROL_CLIENTDATA_HPP_ */