#ifndef ORG_EEROS_SEQUENCER_SEQUENCE_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCE_HPP_

#include <string>
#include <map>
#include <functional>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/SequenceResult.hpp>

namespace eeros {
	namespace sequencer {
	
		// Base class
		class SequenceBase {
			
			friend class eeros::sequencer::Sequencer;
			
		public:
			SequenceBase(std::string name, Sequencer* sequencer);
			
			virtual std::string getName() const;
			virtual bool checkPreCondition();
			virtual bool checkPostCondition();
			
		protected:
			virtual void yield();
			
			virtual void init();
			virtual void exit();
			
			eeros::logger::Logger log;
			std::string name;
			Sequencer* sequencer;
		};
		
		// Class template Sequence
		template<typename Treturn = void, typename ... Targs>
		class Sequence : public SequenceBase {
			
		public:
			Sequence(std::string name, Sequencer* sequencer) : SequenceBase(name, sequencer) { }
			
			SequenceResult<Treturn> operator()(Targs ... args) {
				init();
				yield();
				
				if(!checkPreCondition())
					return SequenceResult<Treturn>(result::preConditionFailure);
				
				yield();
				Treturn res = run(args...);
				yield();
				
				if(!checkPostCondition())
					return SequenceResult<Treturn>(result::postConditionFailure, res);
				
				yield();
				exit();
				
				return SequenceResult<Treturn>(result::success, res);
			}
			
		protected:
			virtual Treturn run(Targs... args) { }
		};
		
		// Specializations for class template Sequence
		template<typename ... Targs>
		class Sequence<void, Targs...> : public SequenceBase {
			
		public:
			Sequence(std::string name, Sequencer* sequencer) : SequenceBase(name, sequencer) { }
			
			SequenceResult<void> operator()(Targs ... args) {
				init();
				yield();
				
				if(!checkPreCondition())
					return SequenceResult<void>(result::preConditionFailure);
				
				yield();
				run(args...);
				yield();
				
				if(!checkPostCondition())
					return SequenceResult<void>(result::postConditionFailure);
				
				yield();
				exit();
				
				return SequenceResult<void>(result::success);
			}
			
		protected:
			virtual void run(Targs... args) { }
		};
		
		template<>
		class Sequence<void> : public SequenceBase {
			
		public:
			Sequence(std::string name, Sequencer* sequencer) : SequenceBase(name, sequencer) {
				sequencer->addCmdSequence(this);
			}
			
			SequenceResult<void> operator()() {
				init();
				yield();
				
				if(!checkPreCondition())
					return SequenceResult<void>(result::preConditionFailure);
				
				yield();
				run();
				yield();
				
				if(!checkPostCondition())
					return SequenceResult<void>(result::postConditionFailure);
				
				yield();
				exit();
				
				return SequenceResult<void>(result::success);
			}
			
		protected:
			virtual void run() { }
		};
	}; // namespace sequencer
}; // namespace eeros

#endif // ORG_EEROS_SEQUENCER_SEQUENCE_HPP_
