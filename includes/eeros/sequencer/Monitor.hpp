#ifndef ORG_EEROS_SEQUENCER_MONITOR_HPP_
#define ORG_EEROS_SEQUENCER_MONITOR_HPP_

#include <string>

namespace eeros {
	namespace sequencer {
		
		class BaseSequence;
		class Condition;
		
		enum class SequenceProp {
			resume,				// continue (only exception sequence is called, if available)
			abort,				// abort the owner sequence or step of this monitor
			restart				// restart the owner sequence or step of this monitor
		};

		class Monitor {
			friend class BaseSequence;
		public:
			Monitor(std::string name, BaseSequence* owner, Condition& condition, SequenceProp behavior = SequenceProp::resume, BaseSequence* exceptionSequence = nullptr);
			virtual ~Monitor();
			void setExceptionSequence(BaseSequence& exceptionSequence);	
			void setBehavior(SequenceProp behavior);
			SequenceProp getBehavior() const;
			BaseSequence* getOwner() const;
		protected:
			bool checkCondition();
			void startExceptionSequence();
			BaseSequence* owner;
			BaseSequence* exceptionSequence;
			Condition& condition;
			SequenceProp behavior;
			std::string name;
		};

	};	//namespace sequencer
}; // namespace eeros

#endif	// ORG_EEROS_SEQUENCER_MONITOR_HPP_