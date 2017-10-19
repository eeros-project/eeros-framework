#ifndef ORG_EEROS_CONTROL_TRANSITIONBLOCKDOWN_HPP_
#define ORG_EEROS_CONTROL_TRANSITIONBLOCKDOWN_HPP_

#include <mutex>
#include <eeros/control/Block1i.hpp>
#include <eeros/control/Block1o.hpp>
#include <eeros/control/Block1i1o.hpp>
#include <eeros/control/Signal.hpp>
#include <eeros/logger/Logger.hpp>

namespace eeros {
	namespace control {
		
		template < typename T > class TransitionBlockDown;
		
		template < typename T = double >
		class TransitionBlockDownIn : public Block1i<T> {
		public:
			TransitionBlockDownIn(TransitionBlockDown<T>* c) : container(c) { }
			virtual ~TransitionBlockDownIn() { }
			
			virtual void run() { 
				container->mtx.lock();
				auto val = this->getIn().getSignal(); 
				container->buf.push_back(val);
				container->mtx.unlock();
			}
			
		protected:
			TransitionBlockDown<T>* container;
		};
		
		template < typename T = double >
		class TransitionBlockDownOut : public Block1i1o<T> {
		public:
			TransitionBlockDownOut(TransitionBlockDown<T>* c) : container(c) { }
			virtual ~TransitionBlockDownOut() { }
			
			virtual void run() { 
				auto time = this->getIn().getSignal().getTimestamp();
// 				this->container->log.fatal() << time << "  capacity = " << container->buf.size();
				container->mtx.lock();
				Signal<T> sig;
				for (typename std::vector<Signal<T>>::iterator it = container->buf.begin(); it != container->buf.end(); ++it) {
// 					this->container->log.error() << time << "   " << (*it).getTimestamp();
					sig = *it;
					if (time <= sig.getTimestamp()) break;
// 						container->log.warn() << *it;
				}
				container->buf.clear();
				container->mtx.unlock();
				this->getOut().getSignal().setValue(sig.getValue());
				this->getOut().getSignal().setTimestamp(sig.getTimestamp());
			}
			
		protected:
			TransitionBlockDown<T>* container;
		};

		template < typename T = double >
		class TransitionBlockDown {
		friend class TransitionBlockDownIn<T>;
		friend class TransitionBlockDownOut<T>;
		public:
			TransitionBlockDown(double ratio) : ratio(ratio), inBlock(this), outBlock(this), bufSize(1/ratio), buf(bufSize) { }
			
			TransitionBlockDownIn<T> inBlock;
			TransitionBlockDownOut<T> outBlock;
			
		private: 
			std::vector<Signal<T>> buf;
			double ratio;
			uint32_t bufSize;
			std::mutex mtx;
			eeros::logger::Logger log;
		
// 			template <typename S> typename std::enable_if<std::is_integral<S>::value>::type _clear() {
// 				in = std::numeric_limits<int32_t>::min();
// 			}
// 			template <typename S> typename std::enable_if<std::is_floating_point<S>::value>::type _clear() {
// 				in = std::numeric_limits<double>::quiet_NaN();
// 			}
// 			template <typename S> typename std::enable_if<!std::is_arithmetic<S>::value && std::is_integral<typename S::value_type>::value>::type _clear() {
// 				in.fill(std::numeric_limits<int32_t>::min());
// 			}
// 			template <typename S> typename std::enable_if<   !std::is_arithmetic<S>::value && std::is_floating_point<typename S::value_type>::value>::type _clear() {
// 				in.fill(std::numeric_limits<double>::quiet_NaN());
// 			}
		};

		/********** Print functions **********/
		template <typename T>
		std::ostream& operator<<(std::ostream& os, TransitionBlockDown<T>& t) {
			os << "Block transition: '" << t.getName() << "'"; 
		}

	};
};

#endif /* ORG_EEROS_CONTROL_TRANSITIONBLOCKDOWN_HPP_ */