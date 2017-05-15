#ifndef ORG_EEROS_CONTROL_TRANSITIONBLOCK_HPP_
#define ORG_EEROS_CONTROL_TRANSITIONBLOCK_HPP_

#include <mutex>
#include <eeros/control/Block1i.hpp>
#include <eeros/control/Block1o.hpp>

namespace eeros {
	namespace control {
		
		template < typename T > class TransitionBlock;
		
		template < typename T = double >
		class TransitionBlockIn : public Block1i<T> {
		public:
			TransitionBlockIn(TransitionBlock<T>* c) : container(c) { }
			virtual ~TransitionBlockIn() { }
			
			virtual void run() { 
				container->mtx.lock();
				container->val = this->getIn().getSignal().getValue(); 
				container->mtx.unlock();
			}
			
		protected:
			TransitionBlock<T>* container;
		};
		
		template < typename T = double >
		class TransitionBlockOut : public Block1o<T> {
		public:
			TransitionBlockOut(TransitionBlock<T>* c) : container(c) { }
			virtual ~TransitionBlockOut() { }
			
			virtual void run() { 
				container->mtx.lock();
				this->getOut().getSignal().setValue(container->val);
				container->mtx.unlock();
			}
			
		protected:
			TransitionBlock<T>* container;
		};

		template < typename T = double >
		class TransitionBlock {
		friend class TransitionBlockIn<T>;
		friend class TransitionBlockOut<T>;
		public:
			TransitionBlock() : inBlock(this), outBlock(this) { }
			
			TransitionBlockIn<T> inBlock;
			TransitionBlockOut<T> outBlock;
			
		private: 
			T val;
			std::mutex mtx;
		};
	};
};

#endif /* ORG_EEROS_CONTROL_TRANSITIONBLOCK_HPP_ */