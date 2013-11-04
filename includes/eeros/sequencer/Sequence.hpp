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

		/**
		 *Usage: 
		 * The first allocation of a sequencer is automatically the Main-Sequencer!!
		 * MySequencer mainSequencer("MainSequencer");
		 * MySequence sequence("MySequence", mainSequencer);
		 */		
		class Sequence : public Runnable
		{
		public:
			/**
			 * Defining pointer to a methode as a type
			 */
			typedef void (Sequence::*method)();
			/**
			 * Constructor. name is the name of the sequence, this name have toi be uniqu, else the constructoor will throw an exception 
			 * and the caller is the sequencer, which will execute this sequence.
			 * The state of the sequence is initialised with kSequenceNotStarted. 
			 */
			Sequence(std::string name, Sequencer& caller);
			
			/**
			 * Destructor removes this sequence from the list allSequences. 
			 */
			~Sequence();
			
			/**
			 * returns the name of the sequence
			 */
			std::string getName();
			
			/** add a callback (own type method) to the list which will be called in teh run() method from the callerThread.
			  * Make sure, that the thread will be only as "one shot"
			  */
			void addCallBack(method callback);
			
			/**
			 * Finds a callback in the list callBacks and with setCurrent, you can set the iterator to it.
			 * This is useful in exceptions, if you want to recall a method.
			 */
			std::list<Sequence::method>::iterator findCallBack(method callback, bool setCurrent);
			
			/**
			 * Helper for findCallBack, it sets the current iterator in th callBacks list.
			 */
			void setCurrentCallBack(std::list<Sequence::method>::iterator iter);
			
			/**
			 * The run method will be called through the thread (sequencer, callerThread) and normally it will execute every method in the list callBacks.
			 */
			virtual void run();
			
			/**
			 * fillCallBacks() have to be implemented by the user, in this method you can fill the callBacks list with the desired method in the desired sequence to be executed.
			 * Referto the collaboration wiki or the documention for further details.
			 */
			virtual void fillCallBacks() = 0;
			
			/**
			 * Every Sequence is saved on cration in the list allSequences, so you can reuse it.
			 * Since the sequence is identified by name, it can be used to get it.
			 */
			static Sequence* getSequence(std::string name);
			
			/** 
			 * removes the sequence with name from the list allSequences. 
			 * this is used in the destructor
			 */
			static void removeSequence(std::string name);
			
			/**
			 * There are three states 
			 * kSequenceRunning: the sequence is running
			 * kSequenceFinished: the sequence has finished the execution
			 * kSequenceNotStarted: the sequence is not started
			 */
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