#include <eeros/logger/StreamLogWriter.hpp>
#include <iomanip>

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

StreamLogWriter::StreamLogWriter(std::ostream& out) :
	out(out),
	visible_level(LogLevel::INFO),
	enabled(false),
	colored(true)
{ }	// nothing to do

void StreamLogWriter::show(LogLevel level) { visible_level = level; }

void StreamLogWriter::begin(std::ostringstream& os, LogLevel level, unsigned category) {
	enabled = (level <= visible_level);
	if (!enabled) return;
	
	using namespace std;
	
	time_t t(time(nullptr));
	tm* local(localtime(&t));

	os << setfill('0');
	os << setw(4) << (1900 + local->tm_year) << '-' << setw(2) << (local->tm_mon + 1) << '-' << setw(2) << local->tm_mday;
	os << ' ';
	os << setw(2) << local->tm_hour << ':' << setw(2) << local->tm_min << ':' << setw(2) << local->tm_sec;
	os << "  ";

	if (category == 0) os << ' ';
	else {
		if (category >= 'A' && category <= 'Z')	os << (char)category;
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
	if (!enabled) {
		os.clear();
		os.str("");
		return;
	}
	if (colored) os << COLOR_RESET;
	os << std::endl;
	out << os.str();
}


void StreamLogWriter::endl(std::ostringstream& os) {
	if (!enabled) return;
	os << std::endl << "\t\t\t   ";
}
