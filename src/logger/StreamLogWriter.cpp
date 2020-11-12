#include <eeros/logger/StreamLogWriter.hpp>
#include <iomanip>
#include <chrono>

#define COLOR_RESET		"\033[0m"
#define COLOR_BLACK		"\033[22;30m"
#define COLOR_RED		"\033[22;31m"
#define COLOR_GREEN		"\033[22;32m"
#define COLOR_YELLOW		"\033[22;33m"
#define COLOR_BLUE		"\033[22;34m"
#define COLOR_MAGENTA		"\033[22;35m"
#define COLOR_CYAN		"\033[22;36m"
#define COLOR_GRAY		"\033[22;37m"
#define COLOR_DARK_GRAY		"\033[01;30m"
#define COLOR_LIGHT_RED		"\033[01;31m"
#define COLOR_LIGHT_GREEN	"\033[01;32m"
#define COLOR_LIGHT_YELLOW	"\033[01;33m"
#define COLOR_LIGHT_BLUE	"\033[01;34m"
#define COLOR_LIGHT_MAGENTA	"\033[01;35m"
#define COLOR_LIGHT_CYAN	"\033[01;36m"
#define COLOR_WHITE		"\033[01;37m"

using namespace eeros::logger;

StreamLogWriter::StreamLogWriter(std::ostream& out) : out(out), colored(true) {  }  // nothing to do

StreamLogWriter::StreamLogWriter(std::ostream& out, std::string logFile) : out(out), colored(true) {
  time_t now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), ".%Y-%m-%d.%X", &tstruct);
  fileOut.open(logFile + buf, std::ios::trunc);
}

StreamLogWriter::~StreamLogWriter() {fileOut.close();}

void StreamLogWriter::show(LogLevel level) { visible_level = level; }

void StreamLogWriter::begin(std::ostringstream& os, LogLevel level, unsigned category) {
  tm localTime;
  std::chrono::system_clock::time_point tx = std::chrono::system_clock::now();
  time_t now = std::chrono::system_clock::to_time_t(tx);
  localtime_r(&now, &localTime);
  const std::chrono::duration<double> tse = tx.time_since_epoch();
  std::chrono::seconds::rep milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(tse).count() % 1000;
  os << std::setfill('0');
  os << std::setw(4) << (1900 + localTime.tm_year) << '-' << std::setw(2) << (localTime.tm_mon + 1) << '-' << std::setw(2) << localTime.tm_mday;
  os << ' ';
  os << std::setw(2) << localTime.tm_hour << ':' << std::setw(2) << localTime.tm_min << ':' << std::setw(2) << localTime.tm_sec << ':' << std::setw(3) << milliseconds;
  os << "  ";

  if (category == 0) os << ' ';
  else {
    if (category >= 'A' && category <= 'Z') os << (char)category;
    else os << category;
  }
  os << ' ';

  if (colored) {
    switch(level) {
      case LogLevel::FATAL: os << COLOR_MAGENTA; break; // fatal
      case LogLevel::ERROR: os << COLOR_RED; break; // error
      case LogLevel::WARN: os << COLOR_YELLOW; break; // warning
      case LogLevel::INFO: os << COLOR_CYAN; break; // info
      case LogLevel::TRACE: os << COLOR_RESET; break; // trace
      default: os << COLOR_RESET; break;
    }
  }

  switch(level) {
    case LogLevel::FATAL: os << "F"; break; // fatal
    case LogLevel::ERROR: os << "E"; break; // error
    case LogLevel::WARN: os << "W"; break; // warning
    case LogLevel::INFO: os << "I"; break; // info
    case LogLevel::TRACE: os << "T"; break; // trace
    default: os << 'T'; break;
  }
  os << ":  ";
}

void StreamLogWriter::end(std::ostringstream& os) {
  if (colored) os << COLOR_RESET;
  os << std::endl;
  out << os.str();
  fileOut << os.str();
  fileOut.flush();
}


void StreamLogWriter::endl(std::ostringstream& os) {
  os << std::endl << "\t\t\t       ";
}
