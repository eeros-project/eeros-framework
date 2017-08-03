#ifndef ORG_EEROS_SEQUENCER_MONITOR_HPP_
#define ORG_EEROS_SEQUENCER_MONITOR_HPP_

#include <string>

namespace eeros {
	namespace sequencer {
		
		class BaseSequence;
		class Condition;
		
		enum class SequenceProp {
			nothing,			//do nothing. (only exception sequence is called, if available)
			abortOwner,			//abort the owner sequence or step of this monitor
			restartOwner,			//restart the owner sequence or step of this monitor
			abortCallerofOwner,		//abort the caller of the owner of this monitor
			restartCallerOfOwner,		//restart the caller of the owner of this monitor
// 			paused,				//pause, till condition==false
		};

		class Monitor {
		public:
			Monitor(BaseSequence* owner, Condition* condition, SequenceProp behavior);
			Monitor(BaseSequence* owner, Condition* condition, SequenceProp behavior, BaseSequence* exceptionSequence);
			virtual ~Monitor();
			void setExceptionSequence(BaseSequence* exceptionSequence);	
			void setBehavior(SequenceProp behavior);
			SequenceProp getBehavior() const;
			BaseSequence* getOwner() const;
			
			bool checkCondition();
			void startExceptionSequence();
			
		protected:
			BaseSequence* owner;
			Condition* condition;
			BaseSequence* exceptionSequence;
			SequenceProp behavior;
			std::string exceptionDescription;
		private:
			
		};

	};	//namespace sequencer
}; // namespace eeros

#endif	// ORG_EEROS_SEQUENCER_MONITOR_HPP_