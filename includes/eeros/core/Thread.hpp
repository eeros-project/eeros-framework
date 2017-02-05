#ifndef ORG_EEROS_CORE_THREAD_HPP_
#define ORG_EEROS_CORE_THREAD_HPP_

#include <eeros/logger/Logger.hpp>

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
		
		virtual void run();
		
		std::thread t;
		eeros::logger::Logger log;
	};
};

#endif // ORG_EEROS_CORE_THREAD_HPP_
