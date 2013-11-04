#ifndef ORG_EEROS_SEQUENCER_SEQUENCER_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCER_HPP_

#include <string>
#include <eeros/core/Executor.hpp>



namespace eeros{

	namespace control {
		class TimeDomain;
	};
	
	namespace sequencer{
		
		class SequenceException;

		/**
		 * Sequencer is a class, which runs as a thread and will invoke severals Sequencesto be run.
		 * To use this class you have to build your own seqencer. 
		 * Usage:
		 * class MySequencer : public eeros::sequencer::Sequencer{
		 * public:
		 *	MySequencer(std::string name);
		 * };
		 * 
		 * in the main function:
		 * The first allocation of a sequencer is automatically the Main-Sequencer!!
		 * MySequencer mainSequencer("MainSequencer");
		 * MySequence sequence("MySequence", mainSequencer);

		 * create the thread
		 * mainSequencer.start();
		 *
		 * Remark:
		 * The sequence should not be called periodically, the thread should only exisat until the sequence has terminated (one shot)
		 */
		class Sequencer : public Executor
		{
		public:
			/*
			 * In a system you have one main sequencer and many sub sequncers, the sub sequencers are filled in a List.
			 * getMainSequencer returns the main one.
			 */
			static Sequencer* getMainSequencer();
			
			/*
			 * Create an object by initialising the Executor with period null and the first call will generate the main sequencer.
			 */
			Sequencer(std::string name);
			
			/*
			 * does nothing
			 */
			virtual ~Sequencer();
			
			/*
			 * Adds a time domain to the time domain list timeDomains
			 */
			//void addTimeDomain(TimeDomain* tDomain);
			void addTimeDomain(control::TimeDomain* tDomain);
			
			/*
			 * deltes all time domains from the list timeDomains
			 */
			void deleteAllTimeDomains();
			
			/*
			 * Fills the sub sequencer in the subSequencers List
			 */
			void addSubSequencer(Sequencer* seq);
			
			/*
			 * Clears the list of sub sequencer
			 */
			void deleteAllSubSequencers();
			
			/**
			 * returns the name of the sequence
			 */
			std::string getName();
			
			/*
			 * Find a sub sequencer by name, every Sequencer is identified by name.
			 */
			Sequencer* findSequencer(std::string name);
			/*
			 * Deletes a sub sequencer by name.
			 */
			void deleteSequencer(std::string name);
		protected:
			
		private:
			static Sequencer* mainSequencer;
			//List for the mainSequencer, the only Object
			std::list<Sequencer*> subSequencers;
			std::list<control::TimeDomain*> timeDomains;
			std::string sequenceName;
		};//class Sequence

	};//namespace sequencer
};//namespace eeros

#endif //ORG_EEROS_SEQUENCER_SEQUENCE_HPP_