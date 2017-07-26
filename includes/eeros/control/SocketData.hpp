#ifndef ORG_EEROS_CONTROL_SOCKETDATA_HPP_
#define ORG_EEROS_CONTROL_SOCKETDATA_HPP_

#include <eeros/control/Block1i1o.hpp>
#include <eeros/math/Matrix.hpp>
#include <array>
#include <eeros/core/System.hpp>
#include <eeros/sockets/SocketServer.hpp>

namespace eeros {
	namespace control {
	
		template < typename SigInType, typename SigOutType, typename Enable = void >
		class SocketData: public Block1i1o<SigInType, SigOutType> { };
		
		template < typename SigInType, typename SigOutType >
		class SocketData<SigInType, SigOutType, 
			typename std::enable_if<std::is_compound<SigInType>::value && std::is_compound<SigOutType>::value>::type> 
			: public Block1i1o<SigInType, SigOutType> {			
		public:
			SocketData(uint16_t port, double period = 0.01) {
				bufInLen = sizeof(SigInType) / sizeof(SigInValueType);
				bufOutLen = sizeof(SigOutType) / sizeof(SigOutValueType);
				server =  new eeros::sockets::SocketServer<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>(port, period);
			}
			
			~SocketData() {server->stop();}
						
			virtual void run() {
				// receive
				SigOutType output; 
				std::array<SigOutValueType, sizeof(SigOutType) / sizeof(SigOutValueType)>& getData = server->getReceiveBuffer();
				for (int i = 0; i < bufOutLen; i++) output(i) = getData[i];
				
				// send
				if (this->in.isConnected()) {
					for(int i = 0; i < bufInLen; i++) sendData[i] = this->in.getSignal().getValue()(i);
					server->setSendBuffer(sendData);
				}
				
				this->out.getSignal().setValue(output);
				timestamp_t time = System::getTimeNs();
				this->out.getSignal().setTimestamp(time);
			}
			
			template <typename X, typename Y>
			friend std::ostream& operator<<(std::ostream& os, SocketData<X,Y>& s);

		protected:
			typedef typename SigInType::value_type SigInValueType;
			typedef typename SigOutType::value_type SigOutValueType;
			eeros::sockets::SocketServer<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>* server;
			std::array<SigInValueType, sizeof(SigInType) / sizeof(SigInValueType)> sendData;
			uint32_t bufInLen, bufOutLen;
		};

		template < typename SigInType, typename SigOutType >
		class SocketData<SigInType, SigOutType, 
			typename std::enable_if<std::is_arithmetic<SigInType>::value && std::is_compound<SigOutType>::value>::type> 
			: public Block1i1o<SigInType, SigOutType> {			
		public:
			SocketData(uint16_t port, double period = 0.01) {
				bufInLen = 1;
				bufOutLen = sizeof(SigOutType) / sizeof(SigOutValueType);
				server =  new eeros::sockets::SocketServer<1, SigInType, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>(port, period);
			}
			
			~SocketData() {server->stop();}
						
			virtual void run() {
				// receive
				SigOutType output; 
				std::array<SigOutValueType, sizeof(SigOutType) / sizeof(SigOutValueType)>& getData = server->getReceiveBuffer();
				for (int i = 0; i < bufOutLen; i++) output(i) = getData[i];
				
				// send
				if (this->in.isConnected()) {
					sendData[0] = this->in.getSignal().getValue();
					server->setSendBuffer(sendData);
				}
				
				this->out.getSignal().setValue(output);
				timestamp_t time = System::getTimeNs();
				this->out.getSignal().setTimestamp(time);
			}
			
			template <typename X, typename Y>
			friend std::ostream& operator<<(std::ostream& os, SocketData<X,Y>& s);

		protected:
			typedef typename SigOutType::value_type SigOutValueType;
			eeros::sockets::SocketServer<1, SigInType, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>* server;
			std::array<SigInType, 1> sendData;;
			uint32_t bufInLen, bufOutLen;
		};

		template < typename SigInType, typename SigOutType >
		class SocketData<SigInType, SigOutType, 
			typename std::enable_if<std::is_compound<SigInType>::value && std::is_arithmetic<SigOutType>::value>::type> 
			: public Block1i1o<SigInType, SigOutType> {			
		public:
			SocketData(uint16_t port, double period = 0.01) {
				bufInLen = sizeof(SigInType) / sizeof(SigInValueType);
				bufOutLen = 1;
				server =  new eeros::sockets::SocketServer<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, 1, SigOutType>(port, period);
			}
			
			~SocketData() {server->stop();}
						
			virtual void run() {
				// receive
				SigOutType output; 
				std::array<SigOutType, 1>& getData = server->getReceiveBuffer();
				output = getData[0];
				
				// send
				if (this->in.isConnected()) {
					for(int i = 0; i < bufInLen; i++) sendData[i] = this->in.getSignal().getValue()(i);
					server->setSendBuffer(sendData);
				}
				
				this->out.getSignal().setValue(output);
				timestamp_t time = System::getTimeNs();
				this->out.getSignal().setTimestamp(time);
			}
			
			template <typename X, typename Y>
			friend std::ostream& operator<<(std::ostream& os, SocketData<X,Y>& s);

		protected:
			typedef typename SigInType::value_type SigInValueType;
			eeros::sockets::SocketServer<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, 1, SigOutType>* server;
			std::array<SigInValueType, sizeof(SigInType) / sizeof(SigInValueType)> sendData;
			uint32_t bufInLen, bufOutLen;
		};

		template < typename SigInType, typename SigOutType >
		class SocketData<SigInType, SigOutType, 
			typename std::enable_if<std::is_compound<SigInType>::value && std::is_same<SigOutType, std::nullptr_t>::value>::type> 
			: public Block1i1o<SigInType> {			
		public:
			SocketData(uint16_t port, double period = 0.01) {
				bufInLen = sizeof(SigInType) / sizeof(SigInValueType);
				server =  new eeros::sockets::SocketServer<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, 0, std::nullptr_t>(port, period);
			}
			
			~SocketData() {server->stop();}
						
			virtual void run() {
				// send
				if (this->in.isConnected()) {
					for(int i = 0; i < bufInLen; i++) sendData[i] = this->in.getSignal().getValue()(i);
					server->setSendBuffer(sendData);
				}
			}
			
			template <typename X, typename Y>
			friend std::ostream& operator<<(std::ostream& os, SocketData<X,Y>& s);

		protected:
			typedef typename SigInType::value_type SigInValueType;
			eeros::sockets::SocketServer<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, 0, std::nullptr_t>* server;
			std::array<SigInValueType, sizeof(SigInType) / sizeof(SigInValueType)> sendData;
			uint32_t bufInLen;
		};
	
		template < typename SigInType, typename SigOutType >
		class SocketData<SigInType, SigOutType, 
			typename std::enable_if<std::is_same<SigInType, std::nullptr_t>::value && std::is_compound<SigOutType>::value>::type> 
			: public Block1i1o<SigOutType> {			
		public:
			SocketData(uint16_t port, double period = 0.01) {
				bufOutLen = sizeof(SigOutType) / sizeof(SigOutValueType);
				server =  new eeros::sockets::SocketServer<0, std::nullptr_t, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>(port, period);
			}
			
			~SocketData() {server->stop();}
						
			virtual void run() {
				// receive
				SigOutType output; 
				std::array<SigOutValueType, sizeof(SigOutType) / sizeof(SigOutValueType)>& getData = server->getReceiveBuffer();
				for (int i = 0; i < bufOutLen; i++) output(i) = getData[i];
								
				this->out.getSignal().setValue(output);
				timestamp_t time = System::getTimeNs();
				this->out.getSignal().setTimestamp(time);
			}
			
			template <typename X, typename Y>
			friend std::ostream& operator<<(std::ostream& os, SocketData<X,Y>& s);

		protected:
			typedef typename SigOutType::value_type SigOutValueType;
			eeros::sockets::SocketServer<0, std::nullptr_t, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>* server;
			uint32_t bufOutLen;
		};

		/********** Print functions **********/
		template <typename X, typename Y>
		std::ostream& operator<<(std::ostream& os, SocketData<X,Y>& s) {
			os << "Block socket data: '" << s.getName() << "'"; 
		}

	};
}

#endif /* ORG_EEROS_CONTROL_SOCKETDATA_HPP_ */