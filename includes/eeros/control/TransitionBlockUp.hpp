#ifndef ORG_EEROS_CONTROL_TRANSITIONBLOCKUP_HPP_
#define ORG_EEROS_CONTROL_TRANSITIONBLOCKUP_HPP_

#include <mutex>
#include <eeros/control/Block1i.hpp>
#include <eeros/control/Block1o.hpp>
#include <eeros/logger/Logger.hpp>

namespace eeros {
	namespace control {
		
		template < typename T > class TransitionBlockUp;
		
		template < typename T = double >
		class TransitionBlockIn : public Block1i<T> {
		public:
			TransitionBlockIn(TransitionBlockUp<T>* c) : container(c) { }
			virtual ~TransitionBlockIn() { }
			
			virtual void run() { 
				container->mtx.lock();
				container->prevIn = container->in;
				container->in = this->getIn().getSignal(); 
				container->refresh = true;
				container->mtx.unlock();
			}
			
		protected:
			TransitionBlockUp<T>* container;
		};
		
		template < typename T = double >
		class TransitionBlockOut : public Block1o<T> {
		public:
			TransitionBlockOut(TransitionBlockUp<T>* c) : container(c), count(0) {in.clear(); prevIn.clear();}
			virtual ~TransitionBlockOut() { }
			
			virtual void run() { 
				if (container->refresh) {
					container->mtx.lock();
					prevIn = container->prevIn;
					in = container->in;
					container->refresh = false;
					container->mtx.unlock();
					count = 0;
					dVal = (in.getValue() - prevIn.getValue()) / container->ratio;
					dTime= (in.getTimestamp() - prevIn.getTimestamp()) / container->ratio;
				}
				T val = prevIn.getValue() + count * dVal;
				this->getOut().getSignal().setValue(val);
				timestamp_t time = prevIn.getTimestamp() + count * dTime;
				this->getOut().getSignal().setTimestamp(time);
				count++;
			}
			
		protected:
			TransitionBlockUp<T>* container;
			Signal<T> prevIn, in;
			double dVal, dTime;
			uint32_t count;
		};

		template < typename T = double >
		class TransitionBlockUp {
		friend class TransitionBlockIn<T>;
		friend class TransitionBlockOut<T>;
		public:
			TransitionBlockUp(double ratio) : ratio(ratio), inBlock(this), outBlock(this), refresh(false) {in.clear(); prevIn.clear();}
			
			TransitionBlockIn<T> inBlock;
			TransitionBlockOut<T> outBlock;
			
		private: 
			Signal<T> in, prevIn;
			double ratio;
			bool refresh;
			std::mutex mtx;
			eeros::logger::Logger log;
		};

		/********** Print functions **********/
		template <typename T>
		std::ostream& operator<<(std::ostream& os, TransitionBlockUp<T>& t) {
			os << "Block transition: '" << t.getName() << "'"; 
		}

	};
};

#endif /* ORG_EEROS_CONTROL_TRANSITIONBLOCKUP_HPP_ */