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
				container->prevIn = container->in;
				container->in = this->getIn().getSignal().getValue(); 
				container->refresh = true;
				container->mtx.unlock();
			}
			
		protected:
			TransitionBlock<T>* container;
		};
		
		template < typename T = double >
		class TransitionBlockOut : public Block1o<T> {
		public:
			TransitionBlockOut(TransitionBlock<T>* c) : container(c), count(0) {
				prevIn = container->prevIn;
				in = container->in;
			}
			virtual ~TransitionBlockOut() { }
			
			virtual void run() { 
				if (container->refresh) {
					container->mtx.lock();
					prevIn = container->prevIn;
					in = container->in;
					container->refresh = false;
					container->mtx.unlock();
					count = 0;
				}
				T val = prevIn + count * (in - prevIn) / container->ratio;
				this->getOut().getSignal().setValue(val);
				count++;
			}
			
		protected:
			TransitionBlock<T>* container;
			T prevIn, in;
			uint32_t count;
		};

		template < typename T = double >
		class TransitionBlock {
		friend class TransitionBlockIn<T>;
		friend class TransitionBlockOut<T>;
		public:
			TransitionBlock(double ratio) : ratio(ratio), inBlock(this), outBlock(this), refresh(false) {_clear<T>(); prevIn = in;}
			
			TransitionBlockIn<T> inBlock;
			TransitionBlockOut<T> outBlock;
			
		private: 
			T in, prevIn;
			double ratio;
			bool refresh;
			std::mutex mtx;
		
			template <typename S> typename std::enable_if<std::is_integral<S>::value>::type _clear() {
				in = std::numeric_limits<int32_t>::min();
			}
			template <typename S> typename std::enable_if<std::is_floating_point<S>::value>::type _clear() {
				in = std::numeric_limits<double>::quiet_NaN();
			}
			template <typename S> typename std::enable_if<!std::is_arithmetic<S>::value && std::is_integral<typename S::value_type>::value>::type _clear() {
				in.fill(std::numeric_limits<int32_t>::min());
			}
			template <typename S> typename std::enable_if<   !std::is_arithmetic<S>::value && std::is_floating_point<typename S::value_type>::value>::type _clear() {
				in.fill(std::numeric_limits<double>::quiet_NaN());
			}
		};

		/********** Print functions **********/
		template <typename T>
		std::ostream& operator<<(std::ostream& os, TransitionBlock<T>& t) {
			os << "Block transition: '" << t.getName() << "'"; 
		}

	};
};

#endif /* ORG_EEROS_CONTROL_TRANSITIONBLOCK_HPP_ */