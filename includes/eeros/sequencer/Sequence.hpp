#ifndef ORG_EEROS_SEQUENCER_SEQUENCE_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCE_HPP_

#include <string>
#include <list>

#include <eeros/core/Runnable.hpp>

namespace eeros{
	namespace sequencer{

		class Sequencer;
		class SequenceException;

		enum { kSequenceRunning = 0, kSequenceFinished = 1, kSequenceNotStarted = 2};

				
		class Sequence : public Runnable
		{
		public:
			typedef void (Sequence::*method)();
			Sequence(std::string name, Sequencer& caller);
			~Sequence();
			std::string getName();
			void addCallBack(method callback);
			std::list<Sequence::method>::iterator findCallBack(method callback, bool setCurrent);
			void setCurrentCallBack(std::list<Sequence::method>::iterator iter);
			virtual void run();
			virtual void fillCallBacks() = 0;
			static Sequence* getSequence(std::string name);
			static void removeSequence(std::string name);
			int getState();
		protected:
			Sequencer& callerThread;
		private:
			int state;
			std::string sequenceName;
			std::list<method> callBacks;
			std::list<Sequence::method>::iterator currentCallBackIterator;
			static std::list<Sequence*> allSequences;

		};//class Sequence

	};//namespace sequencer
};//namespace eeros

#endif //ORG_EEROS_SEQUENCER_SEQUENCE_HPP_