#ifndef ORG_EEROS_SEQUENCER_CONDITION_HPP_
#define ORG_EEROS_SEQUENCER_CONDITION_HPP_

namespace eeros {
	namespace sequencer {

		class Condition {	
		public:
			Condition() { }
			
			// return state of condition
			bool isTrue() {return validate();}
			virtual bool validate() = 0;	// has to be overwritten
		};
	};	//namespace sequencer
}; // namespace eeros

#endif //ORG_EEROS_SEQUENCER_CONDITION_HPP_