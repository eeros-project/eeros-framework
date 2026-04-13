#include <eeros/logger/Logger.hpp>
#include <eeros/logger/MultiLogWriter.hpp>

namespace eeros::logger {

Logger Logger::log(std::make_shared<StreamLogWriter>(std::cout));

void Logger::addWriter(std::shared_ptr<LogWriter> writer) {
  if (auto* multi = dynamic_cast<MultiLogWriter*>(log.w.get())) {
    // Already a fan-out hub — just append.
    multi->add(std::move(writer));
    return;
  }
  // Promote: wrap existing writer + new writer in a MultiLogWriter.
  auto multi = std::make_shared<MultiLogWriter>();
  multi->visible_level = log.w->visible_level;  // preserve threshold
  if (log.w) multi->add(log.w);
  multi->add(std::move(writer));
  log.w = multi;  // shared_ptr<MultiLogWriter> → shared_ptr<LogWriter> OK
                  // because full definition is visible here
}

void Logger::show(LogLevel level) {
  w->visible_level = level;
  if (auto* multi = dynamic_cast<MultiLogWriter*>(w.get()))
    multi->setChildLevel(level);
}

}
