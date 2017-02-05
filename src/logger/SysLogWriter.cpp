#include <eeros/logger/SysLogWriter.hpp>
#include <syslog.h>

using namespace eeros::logger;

SysLogWriter::SysLogWriter(const std::string name) : 
	name(name), 
	visibleLevel(LogLevel::INFO), 
	enabled(false)
{
	openlog(this->name.c_str(), LOG_CONS | LOG_PID | LOG_NDELAY, LOG_LOCAL0);
}

SysLogWriter::~SysLogWriter() {
	closelog();
}

void SysLogWriter::show(LogLevel level) {
	visibleLevel = level;
}

void SysLogWriter::begin(std::ostringstream& os, LogLevel level, unsigned category) {
	enabled = (level <= visibleLevel);
	if(!enabled) return;
	
	this->level = level;
	
	using namespace std;
	
	if (category == 0) os << ' ';
	else {
		if (category >= 'A' && category <= 'Z')
			os << (char)category;
		else
			os << category;
	}
	os << ' ';

	switch(level) {
		case LogLevel::FATAL: os << "F"; break; // fatal
		case LogLevel::ERROR: os << "E"; break; // error
		case LogLevel::WARN: os << "W"; break; // warning
		case LogLevel::INFO: os << "I"; break; // info
		case LogLevel::TRACE: os << "T"; break; // trace
		default: os << 5; break;
	}
	os << ":  ";
	return;
}

void SysLogWriter::end(std::ostringstream& os) {
	if(!enabled) {
		os.clear();
		os.str("");
		return;
	}
	
	switch(level) {
		case LogLevel::FATAL:
			syslog(LOG_ALERT, "%s", os.str().c_str());
			break;
		case LogLevel::ERROR:
			syslog(LOG_ERR, "%s", os.str().c_str());
			break;
		case LogLevel::WARN:
			syslog(LOG_WARNING, "%s", os.str().c_str());
			break;
		case LogLevel::INFO:
			syslog(LOG_INFO, "%s", os.str().c_str());
			break;
		case LogLevel::TRACE:
			syslog(LOG_DEBUG, "%s", os.str().c_str());
			break;
		default:
			syslog(LOG_INFO, "%s", os.str().c_str());
			break;
	}
}

void SysLogWriter::endl(std::ostringstream& os) {
	if(!enabled) return;
	os << " \u21A9 ";
}