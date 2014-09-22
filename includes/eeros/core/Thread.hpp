#ifndef ORG_EEROS_CORE_THREAD_HPP_
#define ORG_EEROS_CORE_THREAD_HPP_

#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>

#include <thread>
#include <functional>
#include <string>

namespace eeros {
	
	class Thread {
	
	public:
		Thread();
		virtual ~Thread();
		
		virtual std::string getId() const;
		virtual void join();
		
	protected:
		Thread(std::function<void ()> t);
		
		virtual void run() = 0;
		
		std::thread t;
		eeros::logger::Logger<eeros::logger::LogWriter> log;
	};
};

#endif // ORG_EEROS_CORE_THREAD_HPP_
