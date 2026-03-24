#ifndef ORG_EEROS_CONTROL_SOCKETDATA_HPP_
#define ORG_EEROS_CONTROL_SOCKETDATA_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/System.hpp>
#include <eeros/sockets/SocketServer.hpp>
#include <eeros/sockets/SocketClient.hpp>

#include <array>
#include <concepts>
#include <memory>
#include <ostream>

namespace eeros::control {

using namespace sockets;

// concepts
template<typename T>
concept Compound = std::is_compound_v<T>;
template<typename T>
concept Arithmetic = std::is_arithmetic_v<T>;
template<typename T>
concept NullType = std::is_same_v<T, std::nullptr_t>;

// value type helper
template<typename T>
struct value_type_of { using type = T; };
template<Compound T>
struct value_type_of<T> { using type = typename T::value_type; };
template<>
struct value_type_of<std::nullptr_t> { using type = std::nullptr_t; };
template<typename T>
using value_type_of_t = typename value_type_of<T>::type;

// buffer size helper
template<typename T>
constexpr std::size_t buf_size() {
  if constexpr (NullType<T>) return 0;
  else if constexpr (Arithmetic<T>) return 1;
  else return T::size();
}

// base class selector

template<typename In, typename Out>
struct BlockioBase {
  using type = Blockio<1, 1, In, Out>;
};
template<typename In>
struct BlockioBase<In, std::nullptr_t> {
  using type = Blockio<1, 1, In>;
};
template<typename Out>
struct BlockioBase<std::nullptr_t, Out> {
  using type = Blockio<1, 1, Out>;
};
template<typename In, typename Out>
using BlockioBase_t = typename BlockioBase<In, Out>::type;


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

template<typename SigInType, typename SigOutType>
class SocketData : public BlockioBase_t<SigInType, SigOutType> {
  static constexpr std::size_t InN  = buf_size<SigInType>();
  static constexpr std::size_t OutN = buf_size<SigOutType>();
  using InVal  = value_type_of_t<SigInType>;
  using OutVal = value_type_of_t<SigOutType>;
  using Server = SocketServer<InN, InVal, OutN, OutVal>;
  using Client = SocketClient<InN, InVal, OutN, OutVal>;

 public:
  /**
   * Creates a SocketData block acting as server (empty serverIP) or client.
   *
   * @param serverIP  IP of the server; empty → act as server
   * @param port      Port number
   * @param period    Poll period in seconds
   * @param timeout   Connection timeout in seconds
   */
  SocketData(std::string serverIP, uint16_t port,
             double period = 0.01, double timeout = 1.0)
    : isServer(serverIP.empty()) {
    if (isServer)
      server = std::make_unique<Server>(port, period, timeout);
    else
      client = std::make_unique<Client>(serverIP, port, period, timeout);
  }

  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  SocketData(const SocketData&) = delete;
  SocketData& operator=(const SocketData&) = delete;

  /**
   * Destructs the block, stops the server or client.
   */
  ~SocketData() override {
    if (isServer) {if (server) server->stop();}
    else {if (client) client->stop();}
  }

  /**
   * Runs the transceiving algorithm.
   *
   */
  void run() override {
    // receive
    if constexpr (!NullType<SigOutType>) {
      SigOutType output{};
      auto& buf = isServer ? server->getReceiveBuffer()
                           : client->getReceiveBuffer();
      if constexpr (Arithmetic<SigOutType>) {
        output = buf[0];
      } else {
        for (uint32_t i = 0; i < OutN; ++i) output(i) = buf[i];
      }
      this->out.getSignal().setValue(output);
      this->out.getSignal().setTimestamp(System::getTimeNs());
    }
    // send
    if constexpr (!NullType<SigInType>) {
      if (this->in.isConnected()) {
        const auto& val = this->in.getSignal().getValue();
        if constexpr (Arithmetic<SigInType>) {
          sendData[0] = val;
        } else {
          for (uint32_t i = 0; i < InN; ++i) sendData[i] = val(i);
        }
        if (isServer) server->setSendBuffer(sendData);
        else          client->setSendBuffer(sendData);
      }
    }
  }

  /**
   * A socket data block continously tries to get data from the block it is
   * connected to. This other block might temporarily stop sending data.
   * With this function you can query if new data has arrived. It will
   * return true, if the block has received new data. In order to set the
   * flag to false, you have to use the \ref resetNew() function.
   * @see resetNew()
   *
   * @return true, if new data has arrived
   */
  [[nodiscard]] bool isNew() const requires (!NullType<SigOutType>) {
    return isServer ? server->newData : client->newData;
  }

  /**
   * Use this function to reset the new data flag back to false.
   */
  void resetNew() requires (!NullType<SigOutType>) {
    if (isServer) server->newData = false;
    else          client->newData = false;
  }

  /**
   * A socket data block continously tries to establish a connection to
   * its associated socket data block. Use this function to query if such a
   * connection has been made.
   *
   * @return true, if connected
   */
  [[nodiscard]] bool isConnected() const {
    return isServer ? server->isConnected() : client->isConnected();
  }

  /*
   * Friend operator overload to give the operator overload outside
   * the class access to the private fields.
   */
  template<typename X, typename Y>
  friend std::ostream& operator<<(std::ostream& os, const SocketData<X, Y>& s);

 private:
  bool isServer;
  std::unique_ptr<Server> server;
  std::unique_ptr<Client> client;
  std::array<InVal, InN == 0 ? 1 : InN> sendData{};  // size ≥ 1 to avoid zero-size array
};

/********** Print functions **********/
template<typename X, typename Y>
std::ostream& operator<<(std::ostream& os, const SocketData<X, Y>& s) {
  return os << "Block socket data: '" << s.getName() << "'";
}

} // namespace eeros::control

#endif // ORG_EEROS_CONTROL_SOCKETDATA_HPP_
