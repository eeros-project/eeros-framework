#include <eeros/logger/UILogWriter.hpp>

using namespace eeros::logger;

UILogWriter::UILogWriter(eeros::ui::BaseUI* ui) : visibleLevel(3), enabled(false), lock(mutex) {
	
}

UILogWriter::~UILogWriter() {
	
}

void UILogWriter::show(unsigned level) {
	visibleLevel = level;
}

void UILogWriter::begin(unsigned level, unsigned category) {
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

void UILogWriter::end() {
	if(!enabled) return;
	
	switch(level) {
		case 0:
// 			syslog(LOG_ALERT, "%s", out.str().c_str());
			break;
		case 1:
// 			syslog(LOG_ERR, "%s", out.str().c_str());
			break;
		case 2:
// 			syslog(LOG_WARNING, "%s", out.str().c_str());
			break;
		case 3:
// 			syslog(LOG_INFO, "%s", out.str().c_str());
			break;
		case 4:
// 			syslog(LOG_DEBUG, "%s", out.str().c_str());
			break;
		default:
// 			syslog(LOG_INFO, "%s", out.str().c_str());
			break;
	}
	
	out.seekp(0);
	out.str("");
	out.clear();
	
	lock.unlock();
}

void UILogWriter::endl() {
	if(!enabled) return;
	out << " \u21A9 ";
}

LogWriter& UILogWriter::operator <<(int value) {
	if(enabled) out << value;
	return *this;
}

LogWriter& UILogWriter::operator <<(unsigned int value) {
	if(enabled) out << value;
	return *this;
}

LogWriter& UILogWriter::operator<<(long value) {
	if(enabled) out << value;
	return *this;
}

LogWriter& UILogWriter::operator <<(double value) {
	if(enabled) out << value;
	return *this;
}

LogWriter& UILogWriter::operator <<(const std::string& value) {
	if(enabled) out << value;
	return *this;
}

LogWriter& UILogWriter::operator <<(std::ostream& os) {
	if(enabled) out << os;
	return *this;
}

LogWriter& UILogWriter::operator <<(void (*f)(LogWriter&)) {
	if(enabled && f != nullptr) f(*this);
	return *this;
}
