#ifndef ORG_EEROS_CORE_PERIODICTHREAD_HPP_
#define ORG_EEROS_CORE_PERIODICTHREAD_HPP_

#include <eeros/core/Thread.hpp>
#include <eeros/core/PeriodicCounter.hpp>
#include <atomic>

namespace eeros {
		
	class PeriodicThread : public Thread {
	
	public:
		enum status { running = 0, stopping = 1, stopped = 2, paused = 3 };
		
		PeriodicThread(double period, double delay = 0, bool realtime = false, status start = running, int priority = 0);
		virtual ~PeriodicThread();
		
		virtual status getStatus() const;
		virtual double getPeriod() const;
		virtual void start();
		virtual void pause();
		virtual void stop();
		
		static constexpr bool isRealtimeSupported() {
			#ifdef REALTIME_SUPPORT
				return true;
			#else
				return false;
			#endif
		}
		
		PeriodicCounter counter;
		
	private:
		double period, delay;
		bool rt;
		std::atomic<status> s;
		
		static constexpr uint64_t to_ns(double s) { return static_cast<uint64_t>(s * 1000000000); }
		static constexpr uint64_t to_us(double s) { return static_cast<uint64_t>(s * 1000000); }
		static constexpr uint64_t to_ms(double s) { return static_cast<uint64_t>(s * 1000); }
	};
};

#endif // ORG_EEROS_CORE_PERIODICTHREAD_HPP_
