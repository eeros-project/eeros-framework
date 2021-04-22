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
  
using namespace sockets;

template < typename SigInType, typename SigOutType, typename Enable = void >
class SocketData: public Block1i1o<SigInType, SigOutType> { };

/**
 * This class allows to deliver a signal over a socket connection. While one end of the connection 
 * acts as a server the other side will connect as a client. Various signal types can be sent over
 * such a connection, notably basic types such as int and doubles and composite types as matrices. 
 * The connection is established automatically as soon as a server is alive an a client is starting.
 * When one of the two partners stops, the connection is broken. It is automatically re-
 * establed as soon as both are running again.
 * 
 * @tparam SigInType - type of the input signal (double - default type)
 * @tparam SigOutType - type of the output signal (double - default type)
 * @since v1.0
 */

template < typename SigInType, typename SigOutType>
class SocketData<SigInType, SigOutType, 
  typename std::enable_if<std::is_compound<SigInType>::value && std::is_compound<SigOutType>::value>::type> 
      : public Block1i1o<SigInType, SigOutType> {
 public:
   
  /**
   * Creates a SocketData block. This block can be configured either as server or client.
   * 
   * @param serverIP - IP number of the server, if left empty, block acts as a client
   * @param port - port number, server and client must be opened on the same port number
   * @param period - period in s which the thread polls the connection
   * @param timeout - connection timeout time in s 
   */
  SocketData(std::string serverIP, uint16_t port, double period = 0.01, double timeout = 1.0) {
    bufInLen = sizeof(SigInType) / sizeof(SigInValueType);
    bufOutLen = sizeof(SigOutType) / sizeof(SigOutValueType);
    isServer = serverIP.empty();
    if (isServer)
      server = new SocketServer<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>(port, period, timeout);
    else 
      client = new SocketClient<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>(serverIP, port, period, timeout);;
  }
  
  /**
   * Disabling use of copy constructor because the block should never be copied unintentionally.
   */
  SocketData(const SocketData& s) = delete; 

  /**
   * Destructs the block, stops the server or client.
   */
  virtual ~SocketData() {if (isServer) server->stop(); else client->stop();}
        
  /**
   * Runs the transceiving algorithm.
   *
   */
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
  
  /**
   * A socket data block continously tries to get data from the block it is 
   * connected to. This other block might temporarily stop sending data. 
   * With this function you can query if new data has arrived. It will 
   * return true, if the block has received new data. In order to set the
   * flag to false, you have to use the \ref resetNew() function.
   * @see resetNew()
   * 
   * return true, if new data has arrived
   */
  virtual bool isNew() {
    if (isServer) return server->newData; else return client->newData;
  }

  /**
   * Use this function to reset the new data flag back to false.
   */
  virtual void resetNew() {
    if (isServer) server->newData = false; else client->newData = false;
  }
  
  /**
   * A socket data block continously tries to establish a connection to
   * its associated socket data block. Use this function to query if such a 
   * connection has been made.
   * 
   * @return true, if connected
   */
  virtual bool isConnected() {
    if (isServer) return server->isConnected();
    else return client->isConnected();
  }
  
  /*
   * Friend operator overload to give the operator overload outside
   * the class access to the private fields.
   */
  template <typename X, typename Y>
  friend std::ostream& operator<<(std::ostream& os, SocketData<X,Y>& s);

 private:
  typedef typename SigInType::value_type SigInValueType;
  typedef typename SigOutType::value_type SigOutValueType;
  SocketServer<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>* server;
  SocketClient<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>* client;
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
      server =  new SocketServer<1, SigInType, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>(port, period, timeout);
    else
      client =  new SocketClient<1, SigInType, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>(serverIP, port, period, timeout);
  }
  
  virtual ~SocketData() {if (isServer) server->stop(); else client->stop();}
        
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

 private:
  typedef typename SigOutType::value_type SigOutValueType;
  SocketServer<1, SigInType, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>* server;
  SocketClient<1, SigInType, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>* client;
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
      server =  new SocketServer<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, 1, SigOutType>(port, period, timeout);
    else
      client =  new SocketClient<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, 1, SigOutType>(serverIP, port, period, timeout);
  }
  
  virtual ~SocketData() {if (isServer) server->stop(); else client->stop();}
        
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

 private:
  typedef typename SigInType::value_type SigInValueType;
  SocketServer<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, 1, SigOutType>* server;
  SocketClient<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, 1, SigOutType>* client;
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
      server =  new SocketServer<1, SigInType, 1, SigOutType>(port, period, timeout);
    else
      client =  new SocketClient<1, SigInType, 1, SigOutType>(serverIP, port, period, timeout);
  }
  
  virtual ~SocketData() {if (isServer) server->stop(); else client->stop();}
        
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

 private:
  SocketServer<1, SigInType, 1, SigOutType>* server;
  SocketClient<1, SigInType, 1, SigOutType>* client;
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
      server =  new SocketServer<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, 0, std::nullptr_t>(port, period, timeout);
    else 
      client =  new SocketClient<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, 0, std::nullptr_t>(serverIP, port, period, timeout);
  }
  
  virtual ~SocketData() {if (isServer) server->stop(); else client->stop();}
        
  virtual void run() {
    // send
    if (this->in.isConnected()) {
      for(uint32_t i = 0; i < bufInLen; i++) sendData[i] = this->in.getSignal().getValue()(i);
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

 private:
  typedef typename SigInType::value_type SigInValueType;
  SocketServer<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, 0, std::nullptr_t>* server;
  SocketClient<sizeof(SigInType) / sizeof(SigInValueType), SigInValueType, 0, std::nullptr_t>* client;
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
      server =  new SocketServer<0, std::nullptr_t, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>(port, period, timeout);
    else
      client =  new SocketClient<0, std::nullptr_t, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>(serverIP, port, period, timeout);
  }
  
  virtual ~SocketData() {if (isServer) server->stop(); else client->stop();}
        
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

 private:
  typedef typename SigOutType::value_type SigOutValueType;
  SocketServer<0, std::nullptr_t, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>* server;
  SocketClient<0, std::nullptr_t, sizeof(SigOutType) / sizeof(SigOutValueType), SigOutValueType>* client;
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
      server =  new SocketServer<1, SigInType, 0, std::nullptr_t>(port, period, timeout);
    else 
      client =  new SocketClient<1, SigInType, 0, std::nullptr_t>(serverIP, port, period, timeout);
  }
  
  virtual ~SocketData() {if (isServer) server->stop(); else client->stop();}
        
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

 private:
  SocketServer<1, SigInType, 0, std::nullptr_t>* server;
  SocketClient<1, SigInType, 0, std::nullptr_t>* client;
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
      server =  new SocketServer<0, std::nullptr_t, 1, SigOutType>(port, period, timeout);
    else
      client =  new SocketClient<0, std::nullptr_t, 1, SigOutType>(serverIP, port, period, timeout);
  }
  
  virtual ~SocketData() {if (isServer) server->stop(); else client->stop();}
        
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

 private:
  SocketServer<0, std::nullptr_t, 1, SigOutType>* server;
  SocketClient<0, std::nullptr_t, 1, SigOutType>* client;
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
