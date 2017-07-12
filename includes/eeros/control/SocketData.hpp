#ifndef ORG_EEROS_CONTROL_SOCKETDATA_HPP_
#define ORG_EEROS_CONTROL_SOCKETDATA_HPP_

#include <eeros/control/Block1i1o.hpp>
#include <eeros/math/Matrix.hpp>
#include <array>
#include <eeros/core/System.hpp>
#include <eeros/sockets/SocketServer.hpp>

namespace eeros {
	namespace control {
	
		template < uint8_t BufLen = 32, typename T = double, typename SigType = eeros::math::Vector2>
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
				if (this->in.isConnected()) {
					for(int i = 0; i < (sizeof(SigType))/sizeof(T); i++){
						sendData[i] = this->in.getSignal().getValue()(i);
					}
					server->setSendBuffer(sendData);
				}
				
				this->out.getSignal().setValue(output);
				timestamp_t time = System::getTimeNs();
				this->out.getSignal().setTimestamp(time);
			}
			
			template <typename X, typename Y>
			friend std::ostream& operator<<(std::ostream& os, SocketData<32,X,Y>& s);

		protected:
			eeros::sockets::SocketServer<BufLen, T>* server;
			std::array<T, BufLen> sendData;
		};
		
		template < uint8_t BufLen, typename T>
		class SocketData<BufLen, T, double>: public Block1i1o<double> {
			
		public:
			SocketData(uint16_t port, double period = 0.01){
				server =  new eeros::sockets::SocketServer<BufLen, T>(port, period);
			}
			
			~SocketData(){
				server->stop();
			}
						
			virtual void run() {
				// receive
				double output; 
				std::array<T, BufLen>& getData = server->getReceiveBuffer();
				output = getData[0];
				
				// send
				if (this->in.isConnected()) {
					sendData[0] = this->in.getSignal().getValue();
					server->setSendBuffer(sendData);
				}
				
				this->out.getSignal().setValue(output);
				timestamp_t time = System::getTimeNs();
				this->out.getSignal().setTimestamp(time);
			}
			
		protected:
			eeros::sockets::SocketServer<BufLen, T>* server; 
			std::array<T, BufLen> sendData;
		};

		/********** Print functions **********/
		template <typename X, typename Y>
		std::ostream& operator<<(std::ostream& os, SocketData<32,X,Y>& s) {
			os << "Block socket data: '" << s.getName() << "'  = "; 
		}

	};
}

#endif /* ORG_EEROS_CONTROL_SOCKETDATA_HPP_ */