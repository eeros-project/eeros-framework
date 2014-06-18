#include <eeros/logger/SysLogWriter.hpp>
#include <syslog.h>

using namespace eeros::logger;

SysLogWriter::SysLogWriter(const std::string name) : name(name), visibleLevel(3), enabled(false), lock(mutex) {
	openlog(this->name.c_str(), LOG_CONS | LOG_PID | LOG_NDELAY, LOG_LOCAL0);
}

SysLogWriter::~SysLogWriter() {
	closelog();
}

void SysLogWriter::show(unsigned level) {
	visibleLevel = level;
}

void SysLogWriter::begin(unsigned level, unsigned category) {
	enabled = (level <= visibleLevel);
	if(!enabled) return;
	
	this->level = level;
	
	lock.lock();
	
	using namespace std;
	
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

void SysLogWriter::end() {
	if(!enabled) return;
//	out << std::endl;
	
	switch(level) {
		case 0:
			syslog(LOG_ALERT, "%s", out.str().c_str());
			break;
		case 1:
			syslog(LOG_ERR, "%s", out.str().c_str());
			break;
		case 2:
			syslog(LOG_WARNING, "%s", out.str().c_str());
			break;
		case 3:
			syslog(LOG_INFO, "%s", out.str().c_str());
			break;
		case 4:
			syslog(LOG_DEBUG, "%s", out.str().c_str());
			break;
		default:
			syslog(LOG_INFO, "%s", out.str().c_str());
			break;
	}
	
	out.seekp(0);
	out.str("");
	out.clear();
	
	lock.unlock();
}

void SysLogWriter::endl() {
	if(!enabled) return;
	out << " \u21A9 ";
}

LogWriter& SysLogWriter::operator <<(int value) {
	if(enabled)
		out << value;
	
	return *this;
}

LogWriter& SysLogWriter::operator <<(unsigned int value) {
	if(enabled)
		out << value;
	
	return *this;
}

LogWriter& SysLogWriter::operator <<(double value) {
	if(enabled)
		out << value;
	
	return *this;
}

LogWriter& SysLogWriter::operator <<(const std::string& value) {
	if(enabled)
		out << value;
	
	return *this;
}

LogWriter& SysLogWriter::operator <<(std::ostream& os) {
	if(enabled)
		out << os;
	
	return *this;
}

LogWriter& SysLogWriter::operator <<(void (*f)(LogWriter&)) {
	if(enabled)
		if(f != nullptr)
			f(*this);
	
	return *this;
}

