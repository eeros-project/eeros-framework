#ifndef ORG_EEROS_LOGGER_MULTILOGWRITER_HPP_
#define ORG_EEROS_LOGGER_MULTILOGWRITER_HPP_

#include <eeros/logger/LogWriter.hpp>
#include <memory>
#include <vector>
#include <sstream>

namespace eeros::logger {

/**
 * MultiLogWriter fans out every log call to an arbitrary number of
 * child LogWriter instances, allowing simultaneous output to e.g. a
 * StreamLogWriter and a SysLogWriter.
 *
 * @since v1.4
 */
class MultiLogWriter : public LogWriter {
 public:
  /**
   * Appends a child writer. A dedicated ostringstream buffer is allocated for
   * the child at the same time. Writers are invoked in insertion order.
   *
   * @param writer - any concrete LogWriter to add as a sink
   */
  void add(std::shared_ptr<LogWriter> writer) {
    writers.push_back(std::move(writer));
    buffers.emplace_back();  // one dedicated buffer per writer
  }
   
  /**
   * Propagates a log level to every child writer so that a single
   * \ref Logger::show() call applies uniformly across all sinks.
   * Call this whenever the master visible_level changes. \ref Logger::show()
   * does this automatically.
   *
   * @param level - the new minimum visible level to set on all children
   */
  void setChildLevel(LogLevel level) {
    for (auto& w : writers)
      w->visible_level = level;
  }

 protected:
  /**
   * Starts a new log entry on every child whose visible_level permits the
   * incoming level. Each child receives its own cleared buffer and has its
   * begin() called independently, so per-child prefixes (colors, timestamps)
   * do not interfere with each other.
   *
   * @param os       - unused by MultiLogWriter itself (children use buffers_)
   * @param level    - severity level of the incoming message
   * @param category - category identifier passed through to each child
   */
  void begin(std::ostringstream& os, LogLevel level, unsigned category) override {
    for (std::size_t i = 0; i < writers.size(); ++i) {
      if (level <= writers[i]->visible_level) {
        buffers[i].str("");
        buffers[i].clear();
        writers[i]->begin(buffers[i], level, category);
      }
    }
    // capture for append calls
    currentLevel = level;
    currentCategory = category;
  }

  /**
   * Flushes the completed log entry to every eligible child. The content
   * accumulated by \ref LogEntry in \p os is appended to each child's private
   * buffer, then the child's end() is called to write to its destination.
   *
   * @param os - buffer containing the message body built up by LogEntry
   */
  void end(std::ostringstream& os) override {
    // os holds the content accumulated by LogEntry's operator<<.
    // copy it into each child's buffer then flush.
    for (std::size_t i = 0; i < writers.size(); ++i) {
      if (currentLevel <= writers[i]->visible_level) {
        buffers[i] << os.str();
        writers[i]->end(buffers[i]);
      }
    }
  }

  /**
   * Flushes a logical newline to every eligible child. Behaves identically
   * to end() but calls the child's endl() so that line-oriented destinations
   * (e.g. syslog) can treat the flush correctly.
   *
   * @param os - buffer containing the message body up to the newline point
   */
  void endl(std::ostringstream& os) override {
    for (std::size_t i = 0; i < writers.size(); ++i) {
      if (currentLevel <= writers[i]->visible_level) {
        buffers[i] << os.str();
        writers[i]->endl(buffers[i]);
      }
    }
  }

 private:
  std::vector<std::shared_ptr<LogWriter>> writers;
  std::vector<std::ostringstream> buffers;
  LogLevel currentLevel = LogLevel::TRACE;
  unsigned currentCategory = 0;
};

}

#endif /* ORG_EEROS_LOGGER_MULTILOGWRITER_HPP_ */