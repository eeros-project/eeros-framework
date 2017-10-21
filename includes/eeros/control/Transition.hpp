#ifndef ORG_EEROS_CONTROL_TRANSITION_HPP_
#define ORG_EEROS_CONTROL_TRANSITION_HPP_

#include <mutex>
#include <eeros/control/Block1i.hpp>
#include <eeros/control/Block1o.hpp>
#include <eeros/control/Block1i1o.hpp>

namespace eeros {
	namespace control {
		
		template < typename T > class Transition;
		
		template < typename T = double >
		class TransitionInBlock : public Block1i<T> {
		public:
			TransitionInBlock(Transition<T>* c) : container(c) { }
			virtual ~TransitionInBlock() { }
			
			virtual void run() { 
				if (up) {	// up
					container->mtx.lock();
					container->prevIn = container->in;
					container->in = this->getIn().getSignal(); 
					container->refresh = true;
					container->mtx.unlock();
				} else {	//down
					container->mtx.lock();
					auto val = this->getIn().getSignal(); 
					container->buf.push_back(val);
					container->mtx.unlock();
				}
			}
			
			bool up;

		protected:
			Transition<T>* container;
		};
		
		template < typename T = double >
		class TransitionOutBlock : public Block1i1o<T> {
		public:
			TransitionOutBlock(Transition<T>* c) : container(c), count(0) {in.clear(); prevIn.clear();}
			virtual ~TransitionOutBlock() { }
			
			virtual void run() { 
				if (up) {	// up
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
					T val = prevIn.getValue() + dVal * count;
					this->getOut().getSignal().setValue(val);
					timestamp_t time = prevIn.getTimestamp() + count * dTime;
					this->getOut().getSignal().setTimestamp(time);
					count++;
				} else {	//down
					auto time = this->getIn().getSignal().getTimestamp();
					container->mtx.lock();
					int i = 0;
					while (i < container->buf.size() && time > container->buf[i].getTimestamp()) i++;
					if (i > 0) i--;
					Signal<T> sig = container->buf[i];
					container->buf.clear();
					container->mtx.unlock();
					this->getOut().getSignal().setValue(sig.getValue());
					this->getOut().getSignal().setTimestamp(sig.getTimestamp());
				}
			}
			
			bool up;

		protected:
			Transition<T>* container;
			Signal<T> prevIn, in;
			T dVal;
			double dTime;
			uint32_t count;
		};

		template < typename T = double >
		class Transition {
		friend class TransitionInBlock<T>;
		friend class TransitionOutBlock<T>;
		public:
			Transition(double ratio) : ratio(ratio), inBlock(this), outBlock(this) {
				if (ratio >= 1.0) {	// slow to fast time domain
					inBlock.up = true;
					outBlock.up = true;
					refresh = false;
					in.clear(); 
					prevIn.clear();
				} else {	// fast to slow time domain
					inBlock.up = false;
					outBlock.up = false;
					bufSize = 1 / ratio;
				}
			}
			virtual ~Transition() { }
			
			TransitionInBlock<T> inBlock;
			TransitionOutBlock<T> outBlock;
			
		private: 
			std::vector<Signal<T>> buf;
			Signal<T> in, prevIn;
			bool refresh;
			double ratio;
			uint32_t bufSize;
			std::mutex mtx;
		};

		/********** Print functions **********/
		template <typename T>
		std::ostream& operator<<(std::ostream& os, Transition<T>& t) {
			os << "Block transition: '" << t.getName() << "'"; 
		}

	};
};

#endif /* ORG_EEROS_CONTROL_TRANSITION_HPP_ */