#ifndef ORG_EEROS_SEQUENCER_SEQUENCE_HPP_
#define ORG_EEROS_SEQUENCER_SEQUENCE_HPP_

#include <string>
#include <map>
#include <functional>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <iostream>

#include <eeros/core/Runnable.hpp>
#include "SequenceResult.hpp"
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/LogWriter.hpp>
#include <eeros/sequencer/Sequencer.hpp>

namespace eeros {
	namespace sequencer {

		// Forward declarations
		class Sequencer;
	
		template < typename Treturn = void, typename ... Targs >
		class Sequence {
			
			friend class eeros::sequencer::Sequencer;
			
		public:
			Sequence(std::string name, Sequencer* sequencer = nullptr) : name(name), sequencer(sequencer) { }
//			Sequence(std::function<void(void)> lambda, Sequencer* Sequencer = nullptr) { }
			
			virtual std::string getName() const { return name; }
			
			SequenceResult<Treturn> operator()(Targs ... args) {
				init();
				if(!checkPreCondition()) return SequenceResult<Treturn>(result::fail);
				Treturn res = run(args...);
				if(!checkPreCondition()) return SequenceResult<Treturn>(result::fail, res);
				exit();
				return SequenceResult<Treturn>(result::success, res);
			}
			
			virtual bool checkPreCondition() { return true; }
			virtual bool checkPostCondition() { return true; }
			
		protected:
			virtual void init() { }
			virtual Treturn run(Targs... args) { }
			virtual void exit() { }
			
			virtual void yield() {
				if(sequencer != nullptr) sequencer->yield();
			}
			
			eeros::logger::Logger<eeros::logger::LogWriter> log;
			
		private:
			std::string name;
			Sequencer* sequencer;
			
		}; // class Sequence
		
		template <typename ... Targs>
		class Sequence<void, Targs...> {
			
			friend class eeros::sequencer::Sequencer;
			
		public:
			Sequence(std::string name, Sequencer* sequencer = nullptr) : name(name), sequencer(sequencer) { }
			
			virtual std::string getName() const { return name; }
			
			SequenceResult<void> operator()(Targs ... args) {
				init();
				yield();
				if(!checkPreCondition()) return SequenceResult<void>(result::fail);
				yield();
				run(args...);
				yield();
				if(!checkPreCondition()) return SequenceResult<void>(result::fail);
				yield();
				exit();
				return SequenceResult<void>(result::success);
			}
			
			virtual bool checkPreCondition() { return true; }
			virtual bool checkPostCondition() { return true; }
			
		protected:
			virtual void init() { }
			virtual void run(Targs... args) { }
			virtual void exit() { }
			
			virtual void yield() {
				if(sequencer != nullptr) {
					sequencer->yield();
				}
			}
			
			eeros::logger::Logger<eeros::logger::LogWriter> log;
			
		private:
			std::string name;
			Sequencer* sequencer;
			
		};
	}; // namespace sequencer
}; // namespace eeros

#endif // ORG_EEROS_SEQUENCER_SEQUENCE_HPP_
