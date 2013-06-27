#ifndef ORG_EEROS_SEQUENCER_SEQUENCE_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCE_HPP_

#include <string>
#include <list>

#include <eeros/core/Runnable.hpp>

namespace eeros{
	namespace sequencer{

		class Sequencer;
		
		class Sequence : public Runnable
		{
		public:
			typedef void (Sequence::*method)();
			Sequence(std::string name, Sequencer& caller);
			std::string getName();
			void addCallBack(method callback);
			virtual void run();
			virtual void fillCallBacks() = 0;
		protected:
			Sequencer& callerThread;
		private:
			std::string sequenceName;
			std::list<method> callBacks;
		};//class Sequence

	};//namespace sequencer
};//namespace eeros

#endif //ORG_EEROS_SEQUENCER_SEQUENCE_HPP_