#ifndef ORG_EEROS_CONTROL_TRACE_HPP_
#define ORG_EEROS_CONTROL_TRACE_HPP_

#include <eeros/control/Blockio.hpp>
#include <eeros/core/Thread.hpp>
#include <eeros/logger/Logger.hpp>
#include <vector>
#include <string>
#include <fstream>
#include <chrono>
#include <atomic>
#include <unistd.h>

namespace eeros::control {

/**
 * @brief Circular-buffer trace block for recording an input signal over time.
 *
 * Captures signal values and their timestamps into a fixed-size circular
 * buffer while @ref enable()'d. When the buffer is full it wraps around,
 * always keeping the most recent @c bufLen samples.
 *
 * Use @ref getTrace() and @ref getTimestampTrace() to retrieve a
 * chronologically ordered snapshot, then write it to disk via @ref TraceWriter.
 *
 * @tparam T  Signal value type (default: @c double)
 *
 * @par Example
 * @code
 * Trace<double> trace(10000);
 * trace.getIn().connect(someBlock.getOut());
 * // ... in safety system or sequence:
 * trace.enable();
 * // ... later:
 * TraceWriter<double> writer(trace, "/tmp/my_trace");
 * writer.write();
 * @endcode
 */
template < typename T = double >
class Trace : public Blockio<1,0,T> {
 public:
  /**
   * @brief Constructs a Trace block with a circular buffer of @p bufLen samples.
   * @param bufLen  Maximum number of samples to store
   */
  explicit Trace(uint32_t bufLen) : maxBufLen(bufLen), buf(bufLen), timeBuf(bufLen) {}
  
  /**
   * Disabling use of copy constructor and copy assignment
   * because the block should never be copied unintentionally.
   */
  Trace(const Trace& s) = delete; 
  Trace& operator=(const Trace&) = delete;

  /**
   * @brief Samples the input signal into the circular buffer (if enabled).
   */
  void run() override {
    if (!running) return;
    buf[index] = this->getIn().getSignal().getValue();
    timeBuf[index] = this->getIn().getSignal().getTimestamp();
    if (++index == maxBufLen) {
      index = 0;
      cycle = true;
    }
  }
  
  /**
   * @brief Returns a snapshot of recorded values.
   *
   * If the buffer has wrapped, samples are reordered so index 0 is oldest.
   * The caller owns the returned vector.
   *
   * @return Ordered vector of signal values
   */
  [[nodiscard]] std::vector<T> getTrace() const {
    return getBuf(buf);
  }
  
  /**
   * @brief Returns a snapshot of recorded timestamps.
   *
   * @return Ordered vector of timestamps (nanoseconds)
   */
  [[nodiscard]] std::vector<timestamp_t> getTimestampTrace() const {
    return getBuf(timeBuf);
  }
  
  /**
   * @brief Returns the number of valid samples currently stored.
   * @return Sample count (up to @c maxBufLen)
   */
  [[nodiscard]] uint32_t getSize() const {return cycle ? maxBufLen : index;}
  
  /**
   * @brief Starts recording samples on each @ref run() call.
   */
  void enable() override {running = true;}
  
  /**
   * @brief Stops recording — buffer contents are preserved.
   */
  void disable() override {running = false;}
  
  const uint32_t maxBufLen;	///< Capacity of the circular buffer
  
 protected:
  uint32_t index{0}; ///< Next write position
  bool cycle{false}; ///< True once the buffer has wrapped at least once
  bool running{false}; ///< indicates whether trace runs
  std::vector<T> buf;
  std::vector<timestamp_t> timeBuf;

 private:
  template<typename U>
  std::vector<U> getBuf(const std::vector<U>& src) const {
    const uint32_t n = getSize();
    std::vector<U> out(n);
    if (cycle) {
      for (uint32_t i = 0; i < n; ++i)
        out[i] = src[(i + index) % maxBufLen];
    } else {
      std::copy(src.begin(), src.begin() + n, out.begin());
    }
    return out;
  }
};

/********** Print functions **********/
template <typename T>
std::ostream& operator<<(std::ostream& os, const Trace<T>& trace) {
  os << "Block trace: '" << trace.getName() << "'"; 
  return os;
}

/**
 * @brief Writes a @ref Trace buffer to a timestamped file on demand.
 *
 * Runs a background thread that waits for a @ref write() call, then
 * flushes the current trace snapshot to a file whose name is formed as:
 * @code
 * <fileName>_YYYY-MM-DD_HH:MM:SS
 * @endcode
 *
 * @tparam T  Signal value type — must match the associated @ref Trace
 *
 * @par Example
 * @code
 * TraceWriter<double> writer(trace, "/tmp/my_trace");
 * // ... after a fault:
 * writer.write();
 * @endcode
 */
template < typename T = double >
class TraceWriter : public eeros::Thread {
 public:
  /**
   * @brief Constructs a TraceWriter and starts its background thread.
   *
   * @param trace     Trace block to read from
   * @param name  Base path for the output file (timestamp is appended)
   * @param priority  Thread priority (default: 20)
   */
  explicit TraceWriter(Trace<T>& trace, std::string name, int priority = 20) 
      : Thread(priority), trace(trace), fileName(std::move(name)), log(logger::Logger::getLogger()) { }
  
  /**
   * @brief Stops the background thread.
   */
  ~TraceWriter() override {running = false;}

  /**
   * @brief Triggers an asynchronous write of the current trace snapshot.
   *
   * Returns immediately — the file is written by the background thread.
   */
  void write() {go = true;}
  
 private:
  std::atomic<bool> running{false};
  std::atomic<bool> go{false};
  Trace<T>& trace;
  std::string fileName;
  logger::Logger log;

  void run() override {
    running = true;
    while(running) {
      while(running && !go) usleep(1000);
      if (!running) return;
      go = false;
      
      // Build timestamped filename
      auto now = std::chrono::system_clock::now();
      auto now_t = std::chrono::system_clock::to_time_t(now);
      char timeName[32]{};
      std::strftime(timeName, sizeof(timeName), "_%Y-%m-%d_%H-%M-%S", std::localtime(&now_t));
      const std::string path = fileName + timeName;
      log.info() << "write trace file " + path;
      
      auto timeStamps = trace.getTimestampTrace();
      auto values = trace.getTrace();
      const uint32_t n = trace.getSize();
      std::ofstream file(path, std::ios::trunc);
      if (!file) {
        log.error() << "Failed to open trace file: " << path;
        continue;
      }
      file << "name = " << trace.getName() << ", size = " << n 
           << ", maxBufLen = " << trace.maxBufLen << "\n";
      for (uint32_t i = 0; i < n; i++) 
        file << timeStamps[i] << " " << values[i] << '\n';
      log.info() << "trace file written with " << n << " samples";
    }
  }
};

}

#endif /* ORG_EEROS_CONTROL_TRACE_HPP_ */
