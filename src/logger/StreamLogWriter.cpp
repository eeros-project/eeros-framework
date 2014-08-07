#include <eeros/logger/StreamLogWriter.hpp>
using namespace eeros::logger;

#include <iomanip>

#define COLOR_RESET			"\033[0m"
#define COLOR_BLACK			"\033[22;30m"
#define COLOR_RED			"\033[22;31m"
#define COLOR_GREEN			"\033[22;32m"
#define COLOR_YELLOW		"\033[22;33m"
#define COLOR_BLUE			"\033[22;34m"
#define COLOR_MAGENTA		"\033[22;35m"
#define COLOR_CYAN			"\033[22;36m"
#define COLOR_GRAY			"\033[22;37m"
#define COLOR_DARK_GRAY		"\033[01;30m"
#define COLOR_LIGHT_RED		"\033[01;31m"
#define COLOR_LIGHT_GREEN	"\033[01;32m"
#define COLOR_LIGHT_YELLOW	"\033[01;33m"
#define COLOR_LIGHT_BLUE	"\033[01;34m"
#define COLOR_LIGHT_MAGENTA	"\033[01;35m"
#define COLOR_LIGHT_CYAN	"\033[01;36m"
#define COLOR_WHITE			"\033[01;37m"


StreamLogWriter::StreamLogWriter(std::ostream& out) : out(out), visible_level(3), enabled(false), colored(true), lock(mutex) {
	// nothing to do
}

void StreamLogWriter::show(unsigned level) { visible_level = level; }

void StreamLogWriter::begin(unsigned level, unsigned category) {
	enabled = (level <= visible_level);
	if(!enabled) return;
	
	lock.lock();
	
	using namespace std;
	
	time_t t(time(nullptr));
	tm* local(localtime(&t));

	out << setfill('0');
	out << setw(4) << (1900 + local->tm_year) << '-' << setw(2) << (local->tm_mon + 1) << '-' << setw(2) << local->tm_mday;
	out << ' ';
	out << setw(2) << local->tm_hour << ':' << setw(2) << local->tm_min << ':' << setw(2) << local->tm_sec;
	out << "  ";

	if (category == 0) {
		out << ' ';
	}
	else {
		if (category >= 'A' && category <= 'Z')
			out << (char)category;
		else
			out << category;
	}
	out << ' ';

	if(colored) {
		switch(level) {
			case 0: out << COLOR_MAGENTA; break; // fatal
			case 1: out << COLOR_RED; break; // error
			case 2: out << COLOR_YELLOW; break; // warning
			case 3: out << COLOR_CYAN; break; // info
			case 4: out << COLOR_RESET; break; // trace
			default: out << COLOR_RESET; break;
		}
	}

	switch(level) {
		case 0: out << "F"; break; // fatal
		case 1: out << "E"; break; // error
		case 2: out << "W"; break; // warning
		case 3: out << "I"; break; // info
		case 4: out << "T"; break; // trace
		default: out << level; break;
	}
	out << ":  ";
}

void StreamLogWriter::end() {
	if(!enabled) return;
	if(colored) out << COLOR_RESET;
	out << std::endl;
	lock.unlock();
}

void StreamLogWriter::endl() {
	if(!enabled) return;
	out << std::endl << "\t\t\t   ";
}

LogWriter& StreamLogWriter::operator <<(int value) {
	if(enabled)
		out << value;
	
	return *this;
}

LogWriter& StreamLogWriter::operator <<(unsigned int value) {
	if(enabled)
		out << value;
	
	return *this;
}

LogWriter& StreamLogWriter::operator <<(double value) {
	if(enabled)
		out << value;
	
	return *this;
}

LogWriter& StreamLogWriter::operator <<(const std::string& value) {
	if(enabled)
		out << value;
	
	return *this;
}

LogWriter& StreamLogWriter::operator <<(std::ostream& os) {
	if(enabled)
		out << os;
	
	return *this;
}

// LogWriter& StreamLogWriter::operator <<(const void* value) {
// 	if(enabled)
// 		out << value;
// 	
// 	return *this;
// }

LogWriter& StreamLogWriter::operator <<(void (*f)(LogWriter&)) {
	if(enabled)
		if(f != nullptr)
			f(*this);
	
	return *this;
}

