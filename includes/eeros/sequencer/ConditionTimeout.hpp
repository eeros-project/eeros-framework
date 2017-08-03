#ifndef ORG_EEROS_SEQUENCER_CONDITIONTIMEOUT_HPP_
#define ORG_EEROS_SEQUENCER_CONDITIONTIMEOUT_HPP_

#include <eeros/sequencer/Condition.hpp>
#include <chrono>

namespace eeros {
	namespace sequencer {
		
		class ConditionTimeout : public Condition {
		public:
			ConditionTimeout() : timeout(0) { }
			
			bool validate() {
				if ( timeout == 0 ) return false;	// timeout not activated
				if ( started == false ) {		// first call
					resetTimeout();
					return false;
				}
	
				auto now = std::chrono::steady_clock::now();
				auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime);
				if (duration.count() > timeout*1000) return true;
				else return false;
			};
			
			void setTimeoutTime(double timeInSec) {timeout = timeInSec;}	// 0 = not set or infinite
			void resetTimeout() {
				started = true;
				startTime = std::chrono::steady_clock::now();
			}
		private:
			bool started = false;			
			std::chrono::steady_clock::time_point startTime;
			double timeout;				//0 = not set or infinite; in Seconds
		};
		
	};	//namespace sequencer
}; // namespace eeros

#endif // ORG_EEROS_SEQUENCER_CONDITIONTIMEOUT_HPP_