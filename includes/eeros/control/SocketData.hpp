#ifndef ORG_EEROS_CONTROL_SOCKETDATA_HPP_
#define ORG_EEROS_CONTROL_SOCKETDATA_HPP_

#include <eeros/control/Block1i1o.hpp>
#include <eeros/math/Matrix.hpp>
#include <array>
#include <eeros/core/System.hpp>
#include <eeros/sockets/SocketServer.hpp>
#include <eeros/sockets/SocketClient.hpp>

namespace eeros {
	namespace control {
	
		template < typename SigInType, typename SigOutType, typename Enable = void >
		class SocketData: public Block1i1o<SigInType, SigOutType> { };
		
		template < typename SigInType, typename SigOutType>
		class SocketData<SigInType, SigOutType, 
			typename std::enable_if<std::is_compound<SigInType>::value && std::is_compound<SigOutType>::value>::type> 
			: public Block1i1o<SigInType, SigOutType> {			
		public:
			SocketData(std::string serverIP, uint16_t port, double period = 0.01, double timeout = 1.0) {
				bufInLen = sizeof(SigInType) / sizeof(SigInValueType);
				bufOutLen = sizeof(SigOutType) / sizeof(SigOutValueType);
				isServer = serverIP.empty();
				if (isServer)
					server = new eeros::sockets::SocketServer<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>(port, period, timeout);
				else 
					client = new eeros::sockets::SocketClient<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>(serverIP, port, period, timeout);;
			}
			
			~SocketData() {if (isServer) server->stop(); else client->stop();}
						
			virtual void run() {
				// receive
				SigOutType output; 
				if (isServer) {
					std::array<SigOutValueType, sizeof(SigOutType) / sizeof(SigOutValueType)>& getData = server->getReceiveBuffer();
					for (uint32_t i = 0; i < bufOutLen; i++) output(i) = getData[i];
				} else {
					std::array<SigOutValueType, sizeof(SigOutType) / sizeof(SigOutValueType)>& getData = client->getReceiveBuffer();
					for (uint32_t i = 0; i < bufOutLen; i++) output(i) = getData[i];
				}
				// send
				if (this->in.isConnected()) {
					for(uint32_t i = 0; i < bufInLen; i++) sendData[i] = this->in.getSignal().getValue()(i);
					if (isServer) server->setSendBuffer(sendData);
					else client->setSendBuffer(sendData);
				}
				
				this->out.getSignal().setValue(output);
				timestamp_t time = System::getTimeNs();
				this->out.getSignal().setTimestamp(time);
			}
			
			virtual bool isNew() {
				if (isServer) return server->newData; else return client->newData;
			}

			virtual void resetNew() {
				if (isServer) server->newData = false; else client->newData = false;
			}
			
			virtual bool isConnected() {
				if (isServer) return server->isConnected();
				else return client->isConnected();
			}
			
			template <typename X, typename Y>
			friend std::ostream& operator<<(std::ostream& os, SocketData<X,Y>& s);

		protected:
			typedef typename SigInType::value_type SigInValueType;
			typedef typename SigOutType::value_type SigOutValueType;
			eeros::sockets::SocketServer<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>* server;
			eeros::sockets::SocketClient<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>* client;
			std::array<SigInValueType, sizeof(SigInType) / sizeof(SigInValueType)> sendData;
			uint32_t bufInLen, bufOutLen;
			bool isServer;
		};

		template < typename SigInType, typename SigOutType >
		class SocketData<SigInType, SigOutType,
			typename std::enable_if<std::is_arithmetic<SigInType>::value && std::is_compound<SigOutType>::value>::type> 
			: public Block1i1o<SigInType, SigOutType> {			
		public:
			SocketData(std::string serverIP, uint16_t port, double period = 0.01, double timeout = 1.0) {
				bufInLen = 1;
				bufOutLen = sizeof(SigOutType) / sizeof(SigOutValueType);
				isServer = serverIP.empty();
				if (isServer)
					server =  new eeros::sockets::SocketServer<1, SigInType, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>(port, period, timeout);
				else
					client =  new eeros::sockets::SocketClient<1, SigInType, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>(serverIP, port, period, timeout);
			}
			
			~SocketData() {if (isServer) server->stop(); else client->stop();}
						
			virtual void run() {
				// receive
				SigOutType output; 
				if (isServer) {
					std::array<SigOutValueType, sizeof(SigOutType) / sizeof(SigOutValueType)>& getData = server->getReceiveBuffer();
					for (int i = 0; i < bufOutLen; i++) output(i) = getData[i];
				} else {
					std::array<SigOutValueType, sizeof(SigOutType) / sizeof(SigOutValueType)>& getData = client->getReceiveBuffer();
					for (int i = 0; i < bufOutLen; i++) output(i) = getData[i];
				}
				
				// send
				if (this->in.isConnected()) {
					sendData[0] = this->in.getSignal().getValue();
					if (isServer) server->setSendBuffer(sendData);
					else client->setSendBuffer(sendData);
				}
				
				this->out.getSignal().setValue(output);
				timestamp_t time = System::getTimeNs();
				this->out.getSignal().setTimestamp(time);
			}
			
			virtual bool isNew() {
				if (isServer) return server->newData; else return client->newData;
			}

			virtual void resetNew() {
				if (isServer) server->newData = false; else client->newData = false;
			}
			
			virtual bool isConnected() {
				if (isServer) return server->isConnected();
				else return client->isConnected();
			}
			
			template <typename X, typename Y>
			friend std::ostream& operator<<(std::ostream& os, SocketData<X,Y>& s);

		protected:
			typedef typename SigOutType::value_type SigOutValueType;
			eeros::sockets::SocketServer<1, SigInType, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>* server;
			eeros::sockets::SocketClient<1, SigInType, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>* client;
			std::array<SigInType, 1> sendData;;
			uint32_t bufInLen, bufOutLen;
			bool isServer;
		};

		template < typename SigInType, typename SigOutType >
		class SocketData<SigInType, SigOutType,
			typename std::enable_if<std::is_compound<SigInType>::value && std::is_arithmetic<SigOutType>::value>::type> 
			: public Block1i1o<SigInType, SigOutType> {			
		public:
			SocketData(std::string serverIP, uint16_t port, double period = 0.01, double timeout = 1.0) {
				bufInLen = sizeof(SigInType) / sizeof(SigInValueType);
				bufOutLen = 1;
				isServer = serverIP.empty();
				if (isServer)
					server =  new eeros::sockets::SocketServer<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, 1, SigOutType>(port, period, timeout);
				else
					client =  new eeros::sockets::SocketClient<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, 1, SigOutType>(serverIP, port, period, timeout);
			}
			
			~SocketData() {if (isServer) server->stop(); else client->stop();}
						
			virtual void run() {
				// receive
				SigOutType output; 
				if (isServer) {
					std::array<SigOutType, 1>& getData = server->getReceiveBuffer();
					output = getData[0];
				} else {
					std::array<SigOutType, 1>& getData = client->getReceiveBuffer();
					output = getData[0];
				}
				
				// send
				if (this->in.isConnected()) {
					for(int i = 0; i < bufInLen; i++) sendData[i] = this->in.getSignal().getValue()(i);
					if (isServer) server->setSendBuffer(sendData);
					else client->setSendBuffer(sendData);
				}
				
				this->out.getSignal().setValue(output);
				timestamp_t time = System::getTimeNs();
				this->out.getSignal().setTimestamp(time);
			}
			
			virtual bool isNew() {
				if (isServer) return server->newData; else return client->newData;
			}

			virtual void resetNew() {
				if (isServer) server->newData = false; else client->newData = false;
			}
			
			virtual bool isConnected() {
				if (isServer) return server->isConnected();
				else return client->isConnected();
			}
			
			template <typename X, typename Y>
			friend std::ostream& operator<<(std::ostream& os, SocketData<X,Y>& s);

		protected:
			typedef typename SigInType::value_type SigInValueType;
			eeros::sockets::SocketServer<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, 1, SigOutType>* server;
			eeros::sockets::SocketClient<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, 1, SigOutType>* client;
			std::array<SigInValueType, sizeof(SigInType) / sizeof(SigInValueType)> sendData;
			uint32_t bufInLen, bufOutLen;
			bool isServer;
		};

		template < typename SigInType, typename SigOutType >
		class SocketData<SigInType, SigOutType,
			typename std::enable_if<std::is_arithmetic<SigInType>::value && std::is_arithmetic<SigOutType>::value>::type> 
			: public Block1i1o<SigInType, SigOutType> {			
		public:
			SocketData(std::string serverIP, uint16_t port, double period = 0.01, double timeout = 1.0) {
				bufInLen = 1;
				bufOutLen = 1;
				isServer = serverIP.empty();
				if (isServer)
					server =  new eeros::sockets::SocketServer<1, SigInType, 1, SigOutType>(port, period, timeout);
				else
					client =  new eeros::sockets::SocketClient<1, SigInType, 1, SigOutType>(serverIP, port, period, timeout);
			}
			
			~SocketData() {if (isServer) server->stop(); else client->stop();}
						
			virtual void run() {
				// receive
				SigOutType output; 
				if (isServer) {
					std::array<SigOutType, 1>& getData = server->getReceiveBuffer();
					output = getData[0];
				} else {
					std::array<SigOutType, 1>& getData = client->getReceiveBuffer();
					output = getData[0];
				}
				
				// send
				if (this->in.isConnected()) {
					sendData[0] = this->in.getSignal().getValue();
					if (isServer) server->setSendBuffer(sendData);
					else client->setSendBuffer(sendData);
				}
				
				this->out.getSignal().setValue(output);
				timestamp_t time = System::getTimeNs();
				this->out.getSignal().setTimestamp(time);
			}
			
			virtual bool isNew() {
				if (isServer) return server->newData; else return client->newData;
			}

			virtual void resetNew() {
				if (isServer) server->newData = false; else client->newData = false;
			}
			
			virtual bool isConnected() {
				if (isServer) return server->isConnected();
				else return client->isConnected();
			}
			
			template <typename X, typename Y>
			friend std::ostream& operator<<(std::ostream& os, SocketData<X,Y>& s);

		protected:
			eeros::sockets::SocketServer<1, SigInType, 1, SigOutType>* server;
			eeros::sockets::SocketClient<1, SigInType, 1, SigOutType>* client;
			std::array<SigInType, 1> sendData;
			uint32_t bufInLen, bufOutLen;
			bool isServer;
		};

		template < typename SigInType, typename SigOutType >
		class SocketData<SigInType, SigOutType,
			typename std::enable_if<std::is_compound<SigInType>::value && std::is_same<SigOutType, std::nullptr_t>::value>::type> 
			: public Block1i1o<SigInType> {			
		public:
			SocketData(std::string serverIP, uint16_t port, double period = 0.01, double timeout = 1.0) {
				bufInLen = sizeof(SigInType) / sizeof(SigInValueType);
				isServer = serverIP.empty();
				if (isServer)
					server =  new eeros::sockets::SocketServer<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, 0, std::nullptr_t>(port, period, timeout);
				else 
					client =  new eeros::sockets::SocketClient<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, 0, std::nullptr_t>(serverIP, port, period, timeout);
			}
			
			~SocketData() {if (isServer) server->stop(); else client->stop();}
						
			virtual void run() {
				// send
				if (this->in.isConnected()) {
					for(int i = 0; i < bufInLen; i++) sendData[i] = this->in.getSignal().getValue()(i);
					if (isServer) server->setSendBuffer(sendData);
					else client->setSendBuffer(sendData);
				}
			}
			
			virtual bool isConnected() {
				if (isServer) return server->isConnected();
				else return client->isConnected();
			}
			
			template <typename X, typename Y>
			friend std::ostream& operator<<(std::ostream& os, SocketData<X,Y>& s);

		protected:
			typedef typename SigInType::value_type SigInValueType;
			eeros::sockets::SocketServer<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, 0, std::nullptr_t>* server;
			eeros::sockets::SocketClient<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, 0, std::nullptr_t>* client;
			std::array<SigInValueType, sizeof(SigInType) / sizeof(SigInValueType)> sendData;
			uint32_t bufInLen;
			bool isServer;
		};
	
		template < typename SigInType, typename SigOutType >
		class SocketData<SigInType, SigOutType,
			typename std::enable_if<std::is_same<SigInType, std::nullptr_t>::value && std::is_compound<SigOutType>::value>::type> 
			: public Block1i1o<SigOutType> {			
		public:
			SocketData(std::string serverIP, uint16_t port, double period = 0.01, double timeout = 1.0) {
				bufOutLen = sizeof(SigOutType) / sizeof(SigOutValueType);
				isServer = serverIP.empty();
				if (isServer) 
					server =  new eeros::sockets::SocketServer<0, std::nullptr_t, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>(port, period, timeout);
				else
					client =  new eeros::sockets::SocketClient<0, std::nullptr_t, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>(serverIP, port, period, timeout);
			}
			
			~SocketData() {if (isServer) server->stop(); else client->stop();}
						
			virtual void run() {
				// receive
				SigOutType output; 
				if (isServer) {
					std::array<SigOutValueType, sizeof(SigOutType) / sizeof(SigOutValueType)>& getData = server->getReceiveBuffer();
					for (int i = 0; i < bufOutLen; i++) output(i) = getData[i];
				} else {
					std::array<SigOutValueType, sizeof(SigOutType) / sizeof(SigOutValueType)>& getData = client->getReceiveBuffer();
					for (int i = 0; i < bufOutLen; i++) output(i) = getData[i];
				}
								
				this->out.getSignal().setValue(output);
				timestamp_t time = System::getTimeNs();
				this->out.getSignal().setTimestamp(time);
			}
			
			virtual bool isNew() {
				if (isServer) return server->newData; else return client->newData;
			}

			virtual void resetNew() {
				if (isServer) server->newData = false; else client->newData = false;
			}
			
			virtual bool isConnected() {
				if (isServer) return server->isConnected();
				else return client->isConnected();
			}
			
			template <typename X, typename Y>
			friend std::ostream& operator<<(std::ostream& os, SocketData<X,Y>& s);

		protected:
			typedef typename SigOutType::value_type SigOutValueType;
			eeros::sockets::SocketServer<0, std::nullptr_t, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>* server;
			eeros::sockets::SocketClient<0, std::nullptr_t, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>* client;
			uint32_t bufOutLen;
			bool isServer;
		};

		template < typename SigInType, typename SigOutType >
		class SocketData<SigInType, SigOutType,
			typename std::enable_if<std::is_arithmetic<SigInType>::value && std::is_same<SigOutType, std::nullptr_t>::value>::type> 
			: public Block1i1o<SigInType> {			
		public:
			SocketData(std::string serverIP, uint16_t port, double period = 0.01, double timeout = 1.0) {
				bufInLen = 1;
				isServer = serverIP.empty();
				if (isServer)
					server =  new eeros::sockets::SocketServer<1, SigInType, 0, std::nullptr_t>(port, period, timeout);
				else 
					client =  new eeros::sockets::SocketClient<1, SigInType, 0, std::nullptr_t>(serverIP, port, period, timeout);
			}
			
			~SocketData() {if (isServer) server->stop(); else client->stop();}
						
			virtual void run() {
				// send
				if (this->in.isConnected()) {
					sendData[0] = this->in.getSignal().getValue();
					if (isServer) server->setSendBuffer(sendData);
					else client->setSendBuffer(sendData);
				}
			}
			
			virtual bool isConnected() {
				if (isServer) return server->isConnected();
				else return client->isConnected();
			}
			
			template <typename X, typename Y>
			friend std::ostream& operator<<(std::ostream& os, SocketData<X,Y>& s);

		protected:
			eeros::sockets::SocketServer<1, SigInType, 0, std::nullptr_t>* server;
			eeros::sockets::SocketClient<1, SigInType, 0, std::nullptr_t>* client;
			std::array<SigInType, 1> sendData;
			uint32_t bufInLen;
			bool isServer;
		};
	
		template < typename SigInType, typename SigOutType >
		class SocketData<SigInType, SigOutType,
			typename std::enable_if<std::is_same<SigInType, std::nullptr_t>::value && std::is_arithmetic<SigOutType>::value>::type> 
			: public Block1i1o<SigOutType> {			
		public:
			SocketData(std::string serverIP, uint16_t port, double period = 0.01, double timeout = 1.0) {
				bufOutLen = 1;
				isServer = serverIP.empty();
				if (isServer) 
					server =  new eeros::sockets::SocketServer<0, std::nullptr_t, 1, SigOutType>(port, period, timeout);
				else
					client =  new eeros::sockets::SocketClient<0, std::nullptr_t, 1, SigOutType>(serverIP, port, period, timeout);
			}
			
			~SocketData() {if (isServer) server->stop(); else client->stop();}
						
			virtual void run() {
				// receive
				SigOutType output; 
				if (isServer) {
					std::array<SigOutType, 1>& getData = server->getReceiveBuffer();
					output = getData[0];
				} else {
					std::array<SigOutType, 1>& getData = client->getReceiveBuffer();
					output = getData[0];
				}
								
				this->out.getSignal().setValue(output);
				timestamp_t time = System::getTimeNs();
				this->out.getSignal().setTimestamp(time);
			}
			
			virtual bool isNew() {
				if (isServer) return server->newData; else return client->newData;
			}

			virtual void resetNew() {
				if (isServer) server->newData = false; else client->newData = false;
			}
			
			virtual bool isConnected() {
				if (isServer) return server->isConnected();
				else return client->isConnected();
			}
			
			template <typename X, typename Y>
			friend std::ostream& operator<<(std::ostream& os, SocketData<X,Y>& s);

		protected:
			eeros::sockets::SocketServer<0, std::nullptr_t, 1, SigOutType>* server;
			eeros::sockets::SocketClient<0, std::nullptr_t, 1, SigOutType>* client;
			uint32_t bufOutLen;
			bool isServer;
		};

		/********** Print functions **********/
		template <typename X, typename Y>
		std::ostream& operator<<(std::ostream& os, SocketData<X,Y>& s) {
			os << "Block socket data: '" << s.getName() << "'"; 
            return os;
		}

	};
}

#endif /* ORG_EEROS_CONTROL_SOCKETDATA_HPP_ */