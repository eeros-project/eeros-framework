#ifndef ORG_EEROS_CONTROL_SOCKETDATA_HPP_
#define ORG_EEROS_CONTROL_SOCKETDATA_HPP_

#include <eeros/control/Block1i1o.hpp>
#include <array>
#include <eeros/core/System.hpp>
#include <eeros/sockets/SocketServer.hpp>

namespace eeros {
	namespace control {
	
		template < uint8_t BufLen = 32, typename T = double, typename SigType = double >
		class SocketData: public Block1i1o<SigType> {
			
		public:
			SocketData(uint16_t port, double period = 0.01){
				server =  new eeros::sockets::SocketServer<BufLen, T>(port, period);
			}
			
			~SocketData(){
				server->stop();
			}
						
			virtual void run() {
				// receive
				SigType output; 
				std::array<T, BufLen>& getData = server->getReceiveBuffer();
				for(int i = 0; i < (sizeof(SigType))/sizeof(T); i++){
					output(i) = getData[i];
				}
				
				// send
				for(int i = 0; i < (sizeof(SigType))/sizeof(T); i++){
					sendData[i] = this->in.getSignal().getValue()(i);
				}
				server->setSendBuffer(sendData);
				
				this->out.getSignal().setValue(output);
				timestamp_t time = System::getTimeNs();
				this->out.getSignal().setTimestamp(time);
			}
			
		protected:
			eeros::sockets::SocketServer<BufLen, T>* server;
			
		private: 
			std::array<T, BufLen> sendData;
		};
	};
}

#endif /* ORG_EEROS_CONTROL_SOCKETDATA_HPP_ */