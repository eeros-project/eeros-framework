#ifndef CH_NTB_EEDURO_DELTA_SERVERDATA_HPP_
#define CH_NTB_EEDURO_DELTA_SERVERDATA_HPP_

#include <eeros/control/Block1i1o.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <array>
#include <atomic>
#include <iostream>
#include <eeros/core/System.hpp>
#include <eeros/sockets/SocketServer.hpp>

namespace eeros {
	namespace control {
	
		template < uint8_t BufLen = 32, typename T = double, typename SigType = double >
		class SocketData: public eeros::control::Block1i1o<SigType> {
			
		public:
			SocketData(uint16_t port, double periodInSec = 0.01){
				server =  new eeros::sockets::SocketServer<BufLen, T>(9876, 0.01);	//Port;
			}
			
			~SocketData(){
				server->stop();
			}
			
			virtual eeros::control::Input<SigType>& getIn(){
				return in;
			}
			
			virtual eeros::control::Output<SigType>& getOut(){
				return out;
			}
			
			virtual void run() {
				timestamp_t time = System::getTimeNs();
				SigType output; 
				
				// Get data from other robot
				
				std::array<T, BufLen>& getData = server->getReceiveBuffer();
				for(int i = 0; i < (sizeof(SigType))/sizeof(T); i++){
					output(i) = getData[i];
				}
				
				
				// Send data to other robot
				for(int i=0; i < (sizeof(SigType))/sizeof(T);i++){
					sendData[i] = in.getSignal().getValue()(i);
				}
				server->setSendBuffer(sendData);
				
				out.getSignal().setValue(output);
				out.getSignal().setTimestamp(time);
			}
			
		protected:
			eeros::control::Input<SigType> in;
			eeros::control::Output<SigType> out;
			eeros::sockets::SocketServer<BufLen, T>* server;
			
		private: 
			std::array<T, BufLen> sendData;
		};
	};
}

#endif /* CH_NTB_EEDURO_DELTA_SERVERDATA_HPP_ */