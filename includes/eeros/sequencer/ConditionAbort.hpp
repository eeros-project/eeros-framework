#ifndef ORG_EEROS_SEQUENCER_CONDITIONABORT_HPP_
#define ORG_EEROS_SEQUENCER_CONDITIONABORT_HPP_

#include <eeros/sequencer/Condition.hpp>

namespace eeros {
	namespace sequencer {
		
		class ConditionAbort : public Condition {
		public:
			ConditionAbort() { }
			virtual ~ConditionAbort() { }
			
			bool validate() {return abort;}
			void set() {abort = true;}
			void reset() {abort = false;}
		private:
			bool abort = false;			
		};
		
	};	//namespace sequencer
}; // namespace eeros

#endif // ORG_EEROS_SEQUENCER_CONDITIONABORT_HPP_