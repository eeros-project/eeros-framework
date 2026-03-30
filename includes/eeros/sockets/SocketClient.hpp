#ifndef ORG_EEROS_SOCKET_CLIENT_HPP_
#define ORG_EEROS_SOCKET_CLIENT_HPP_

#include <eeros/core/Thread.hpp>
#include <eeros/core/Fault.hpp>
#include <array>
#include <string>
#include <mutex>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <concepts>
#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <poll.h>

namespace eeros::sockets {

template < uint32_t BufInLen, typename inT, uint32_t BufOutLen, typename outT >
class SocketClient : public eeros::Thread {
  static constexpr bool hasOutput = (BufOutLen > 0) && !std::is_same_v<outT,std::nullptr_t>;
  static constexpr bool hasInput = (BufInLen > 0) && !std::is_same_v<inT,std::nullptr_t>;

 public:
  SocketClient(std::string serverIP, uint16_t port, double period = 0.01, double timeout = 1.0, int priority = 5)
      : Thread(priority), serverIP(std::move(serverIP)), port(port), period(period), timeout(timeout) {
    std::signal(SIGPIPE, SIG_IGN); // don't crash on broken pipe
    running = true;
    connected = false;
  }

  virtual ~SocketClient() override {
    join();
  }

  virtual void stop() {
    running = false;
    connected = false;
  }

  virtual bool isRunning() {
    return running;
  }

  virtual bool isConnected() {
    return connected;
  }

  // only available when there is an output
  template<bool Enable = hasOutput>
  auto& getReceiveBuffer() requires Enable {
      std::lock_guard<std::mutex> lock(mtx);
      return rxBuf;
  }

  // only available when there is an input
  template<bool Enable = hasInput>
  void setSendBuffer(std::array<inT, BufInLen == 0 ? 1 : BufInLen>& data) requires Enable {
      std::lock_guard<std::mutex> lock(mtx);
      txBuf = data;
  }

  std::atomic<bool> newData = false;

 private:
  virtual void run() {
    log.info() << "SocketClient thread started";
    while (running) {
      struct sockaddr_in servAddr;
      int sockfd = socket(AF_INET, SOCK_STREAM, 0);
      if (sockfd < 0) throw Fault("ERROR opening socket");

      auto server = gethostbyname(serverIP.c_str());
      if (server == NULL) {
        throw Fault("Server ip not found");
      }
      bzero((char *) &servAddr, sizeof(servAddr));
      servAddr.sin_family = AF_INET;
      bcopy((char *)server->h_addr,(char *)&servAddr.sin_addr.s_addr, server->h_length);
      servAddr.sin_port = htons(port);

      using seconds = std::chrono::duration<double, std::chrono::seconds::period>;
      auto next_cycle = std::chrono::steady_clock::now() + seconds(period);
      while (connect(sockfd, (struct sockaddr *) &servAddr, sizeof(servAddr)) < 0) {
        if (!running) return;
        std::this_thread::sleep_until(next_cycle);
        next_cycle += seconds(period);
      }
      log.info() << "Client connected to ip=" << serverIP;
      connected = true;

      while (connected) {
        std::this_thread::sleep_until(next_cycle);
        // write
        if constexpr (hasInput) {
          inT b_write[BufInLen];
          {
            std::lock_guard lock(mtx);
            for(uint32_t i = 0; i < BufInLen; i++) b_write[i] = txBuf[i];
          }
          int n = write(sockfd, b_write, BufInLen * sizeof(inT));
          if (n < 0) {
            log.trace() << "write error = " << std::strerror(errno);
            connected = false;
          }
        }
        // read
        if constexpr (hasOutput) {
          outT b_read[BufOutLen];
          size_t count = BufOutLen * sizeof(outT);
          uint8_t* ptr = (uint8_t *)b_read;
          auto endTime = std::chrono::steady_clock::now() + seconds(timeout);

          while (connected && count) {
            if (std::chrono::steady_clock::now() > endTime) {
              log.trace() << "read error = socket read timed out";
              connected = false;
              break;
            }
            int n = read(sockfd, ptr, count);
            if (n < 0) {
              log.trace() << "read error = " << std::strerror(errno);
              connected = false;
            }
            ptr += n;
            count -= n;
          }
          if (connected) {
            std::lock_guard lock(mtx);
            for (uint32_t i = 0; i < BufOutLen; ++i) rxBuf[i] = b_read[i];
            newData = true;
          } else {
            newData = false;
          }
        }
        next_cycle += seconds(period);
      }
      close(sockfd);
      // if disconnected clear receive buffer
      if constexpr (hasOutput) {
        std::lock_guard lock(mtx);
        for(uint32_t i = 0; i < BufOutLen; i++) rxBuf[i] = 0;
        newData = false;
      }
    }
  }

  std::atomic<bool> running;
  std::atomic<bool> connected;
  std::string serverIP;
  uint16_t port;
  double period;
  double timeout;	// time which thread tries to read until socket read timed out
  std::mutex mtx;
  [[no_unique_address]] std::conditional_t<hasOutput, std::array<outT, BufOutLen == 0 ? 1 : BufOutLen>, std::monostate> rxBuf{};
  [[no_unique_address]] std::conditional_t<hasInput, std::array<inT, BufInLen  == 0 ? 1 : BufInLen>, std::monostate> txBuf{};
};

}
#endif // ORG_EEROS_SOCKET_CLIENT_HPP_
