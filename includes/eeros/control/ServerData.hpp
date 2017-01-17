#ifndef CH_NTB_EEDURO_DELTA_SERVERDATA_HPP_
#define CH_NTB_EEDURO_DELTA_SERVERDATA_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <array>
#include <atomic>
#include "../types.hpp"
#include "../constants.hpp"
#include "../../socket/Server.hpp"

namespace eeduro {
	namespace delta {
	
		class ServerData: public eeros::control::Block {
			
		public:
			ServerData(Server* serverThread);
			
			virtual eeros::control::Input<AxisVector>& getIn();
			virtual eeros::control::Output<AxisVector>& getOut();
			
			virtual void run();
			
		protected:
			eeros::control::Input<AxisVector> in;
			eeros::control::Output<AxisVector> out;
			Server* server;
		};
	};
}

#endif /* CH_NTB_EEDURO_DELTA_SERVERDATA_HPP_ */