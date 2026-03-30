#ifndef ORG_EEROS_SOCKET_SERVER_HPP_
#define ORG_EEROS_SOCKET_SERVER_HPP_

#include <eeros/core/Thread.hpp>
#include <eeros/core/Fault.hpp>
#include <array>
#include <string>
#include <mutex>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <poll.h>

namespace eeros::sockets {

template < uint32_t BufInLen, typename inT, uint32_t BufOutLen, typename outT >
class SocketServer : public eeros::Thread {
  static constexpr bool hasOutput = (BufOutLen > 0) && !std::is_same_v<outT,std::nullptr_t>;
  static constexpr bool hasInput = (BufInLen > 0) && !std::is_same_v<inT,std::nullptr_t>;

 public:
  SocketServer(uint16_t port, double period = 0.01, double timeout = 1.0, int priority = 5)
     : Thread(priority), port(port), period(period), timeout(timeout) {
    std::signal(SIGPIPE, SIG_IGN);  // broken pipe → error code, not crash
    running = true;
    connected = false;
  }

  virtual ~SocketServer() {
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

  template<bool E = hasOutput>
  auto& getReceiveBuffer() requires E {
    std::lock_guard lock(mtx);
    return rxBuf;
  }

  template<bool E = hasInput>
  void setSendBuffer(std::array<inT, BufInLen == 0 ? 1 : BufInLen>& data) requires E {
    std::lock_guard lock(mtx);
    txBuf = data;
  }

  std::atomic<bool> newData = false;

 private:
  virtual void run() {
    log.info() << "SocketServer thread started";
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) throw Fault("ERROR opening socket");
    int yes = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == -1)
      throw Fault("ERROR on set socket option");
    struct sockaddr_in servAddr{};
    servAddr.sin_port = htons(port);
    servAddr.sin_family = AF_INET;
    servAddr.sin_addr.s_addr = htonl(INADDR_ANY) ;
    if (bind(sockfd, (struct sockaddr *) &servAddr, sizeof(servAddr)) < 0)
      throw Fault("ERROR on socket binding");
    socklen_t clilen;
    listen(sockfd,1);
    int newsockfd = -1;
    while (running) {
      sockaddr_in cliAddr{};
      socklen_t clilen = sizeof(cliAddr);
      while (running) {
        struct pollfd pfd{ sockfd, POLLIN, 0 };
        int ret = poll(&pfd, 1, 100);
        if (ret > 0) {
          newsockfd = accept(sockfd, (struct sockaddr *) &cliAddr,  &clilen);
          break;
        }
      }
      if (!running) break;
      if (newsockfd < 0) throw Fault("ERROR on socket accept");
      char cliName[INET6_ADDRSTRLEN];
      getnameinfo((struct sockaddr*)&cliAddr, sizeof cliAddr, cliName, sizeof(cliName), NULL, 0, NI_NUMERICHOST|NI_NUMERICSERV);
      log.info() << "Client connection from ip=" << cliName << " accepted";
      connected = true;

      // inT b_write[BufInLen]; outT b_read[BufOutLen];
      using seconds = std::chrono::duration<double, std::chrono::seconds::period>;
      auto next_cycle = std::chrono::steady_clock::now() + seconds(period);

      while (connected) {
        std::this_thread::sleep_until(next_cycle);
        // write
        if constexpr (hasInput) {
          inT b_write[BufInLen];
          {
            std::lock_guard lock(mtx);
            for(uint32_t i = 0; i < BufInLen; i++) b_write[i] = txBuf[i];
          }
          int n = write(newsockfd, b_write, BufInLen * sizeof(inT));
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
              log.trace() << "error = socket read timed out";
              connected = false;
              break;
            }
            int n = read(newsockfd, ptr, count);
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
      close(newsockfd);
      // if disconnected clear receive buffer
      if constexpr (hasOutput) {
        std::lock_guard lock(mtx);
        rxBuf.fill(outT{});
        newData = false;
      }
    }
    close(sockfd);
  }

  std::atomic<bool> running;
  std::atomic<bool> connected;
  uint16_t port;
  double period;
  double timeout;
  std::mutex mtx;
  [[no_unique_address]] std::conditional_t<hasOutput,std::array<outT, BufOutLen == 0 ? 1 : BufOutLen>, std::monostate> rxBuf{};
  [[no_unique_address]] std::conditional_t<hasInput, std::array<inT, BufInLen == 0 ? 1 : BufInLen>, std::monostate> txBuf{};
};

}
#endif // ORG_EEROS_SOCKET_SERVER_HPP_
