#ifndef ORG_EEROS_CORE_CONTINUALTHREAD_HPP_
#define ORG_EEROS_CORE_CONTINUALTHREAD_HPP_

#include <eeros/core/Thread.hpp>

#include <atomic>

namespace eeros {
		
	class ContinualThread : public Thread {
	
	public:
		enum status { running = 0, stopping = 1, stopped = 2, paused = 3 };
		
		ContinualThread();
		virtual ~ContinualThread();
		
		virtual status getStatus() const;
		virtual void start();
		virtual void pause();
		virtual void stop();
		
	private:
		std::atomic<status> s;
	};
};

#endif // ORG_EEROS_CORE_CONTINUALTHREAD_HPP_
