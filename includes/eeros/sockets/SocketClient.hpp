#ifndef ORG_EEROS_SOCKET_CLIENT_HPP_
#define ORG_EEROS_SOCKET_CLIENT_HPP_

#include <eeros/core/Thread.hpp>
#include <array>
#include <unistd.h>
#include <netdb.h> 
#include <arpa/inet.h> 		/* inet_ntoa() to format IP address */
#include <string.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <eeros/core/Fault.hpp>
#include <iostream>
#include <cstring>
#include <signal.h>
#include <mutex>

namespace eeros {
namespace sockets {

template < uint32_t BufInLen, typename inT, uint32_t BufOutLen, typename outT >
class SocketClient : public eeros::Thread {
 public:
  SocketClient(std::string serverIP, uint16_t port, double period = 0.01, double timeout = 1.0, int priority = 5) 
      : Thread(priority), serverIP(serverIP), port(port), period(period), timeout(timeout) {
    signal(SIGPIPE, sigPipeHandler);	// make sure, that a broken pipe does not stop application
    running = false;
    connected = false;
  }
  
  virtual ~SocketClient() {
    join();
  }
  
  virtual void stop() {
    running = false;
  }
  
  virtual bool isRunning() {
    return running;
  }
  
  virtual bool isConnected() {
    return connected;
  }
  
  virtual std::array<outT, BufOutLen>& getReceiveBuffer() {
    std::lock_guard<std::mutex> lock(mtx);
    return rxBuf;
  }
  
  virtual void setSendBuffer(std::array<inT, BufInLen>& data) {
    std::lock_guard<std::mutex> lock(mtx);
    txBuf = data;
  }

  bool newData = false;
  
 private:
  virtual void run() {
    log.info() << "SocketClient thread started";
    running = true;
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
        std::this_thread::sleep_until(next_cycle);
        next_cycle += seconds(period);
      }
      log.info() << "Client connected to ip=" << serverIP;
      inT b_write[BufInLen]; outT b_read[BufOutLen];							
      connected = true;
    
      while (connected) {
        std::this_thread::sleep_until(next_cycle);
        
        // write
        std::unique_lock<std::mutex> wlck(mtx);
        for(uint32_t i = 0; i < BufInLen; i++) b_write[i] = txBuf[i]; 
        wlck.unlock();
        int n = write(sockfd, b_write, BufInLen * sizeof(inT));
        if (n < 0) {
          log.trace() << "error = " << std::strerror(errno);
          connected = false;
        }
        
        // read
        size_t count = BufOutLen * sizeof(outT);
        uint8_t* ptr = (uint8_t *)b_read;
        auto endTime = std::chrono::steady_clock::now() + seconds(timeout);
        while (connected && count) {
          if (std::chrono::steady_clock::now() > endTime) {
            log.trace() << "error = socket read timed out";
            connected = false;
          }
          n = read(sockfd, ptr, count);
          if (n < 0) {
            log.trace() << "error = " << std::strerror(errno);
            connected = false;
          }
          ptr += n;
          count -= n;
        }
        std::unique_lock<std::mutex> rlck(mtx);
        for(uint32_t i = 0; i < BufOutLen; i++) rxBuf[i] = b_read[i];
        rlck.unlock();
        newData = true;
        next_cycle += seconds(period);
      }
      close(sockfd);
      // if disconnected clear receive buffer
      for(uint32_t i = 0; i < BufOutLen; i++) rxBuf[i] = 0;
      newData = true;
    }
  }
  
  bool running;
  std::string serverIP;
  uint16_t port;
  double period;
  double timeout;	// time which thread tries to read until socket read timed out
  struct hostent *server;
  int sockfd;
  bool connected;
  std::mutex mtx;
  std::array<outT, BufOutLen> rxBuf;
  std::array<inT, BufInLen> txBuf;
};

// specialization used when client doesn't receive data from its server
template < uint32_t BufInLen, typename inT >
class SocketClient<BufInLen, inT, 0, std::nullptr_t> : public eeros::Thread {
public:	
  SocketClient(std::string serverIP, uint16_t port, double period = 0.01, double timeout = 1.0, int priority = 5) 
      : Thread(priority), serverIP(serverIP), port(port), period(period), timeout(timeout) {
    signal(SIGPIPE, sigPipeHandler);	// make sure, that a broken pipe does not stop application
    running = false;
    connected = false;
  }
  
  virtual ~SocketClient() {
    join();
  }
  
  virtual void stop() {
    running = false;
  }
  
  virtual bool isRunning() {
    return running;
  }
  
  virtual bool isConnected() {
    return connected;
  }
  
  virtual void setSendBuffer(std::array<inT, BufInLen>& data) {
    std::lock_guard<std::mutex> lock(mtx);
    txBuf = data;
  }

 private:
  virtual void run() {	
    log.info() << "SocketClient thread started";
    running = true;
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
        std::this_thread::sleep_until(next_cycle);
        next_cycle += seconds(period);
      }
      log.info() << "Client connected to ip=" << serverIP;
      inT b_write[BufInLen]; 	

      connected = true;
    
      while (connected) {
        std::this_thread::sleep_until(next_cycle);
      
        // write
        std::unique_lock<std::mutex> wlck(mtx);
        for(uint32_t i = 0; i < BufInLen; i++) b_write[i] = txBuf[i]; 
        int n = write(sockfd, b_write, BufInLen * sizeof(inT));
        wlck.unlock();
        if (n < 0) {
          log.trace() << "error = " << std::strerror(errno);
          connected = false;
        }
        next_cycle += seconds(period);
      }
      close(sockfd);
    }
  }
  
  bool running;
  std::string serverIP;
  uint16_t port;
  double period;
  double timeout;	// time which thread tries to read until socket read timed out
  struct hostent *server;
  int sockfd;
  bool connected;
  std::mutex mtx;
  std::array<inT, BufInLen> txBuf;
};

}
}

#endif // ORG_EEROS_SOCKET_CLIENT_HPP_
