#ifndef ORG_EEROS_CONTROL_TRANSITIONBLOCK_HPP_
#define ORG_EEROS_CONTROL_TRANSITIONBLOCK_HPP_

#include <string>
#include <eeros/core/Runnable.hpp>

namespace eeros {
	namespace control {
		
		template < typename TObject >
		class TransitionAction : public Runnable {
		public:
			using TMethod = void (TObject::*)();
			
			TransitionAction(TObject *object, TMethod method) :
				object(object), method(method) { }

			virtual void run() { (object->*method)(); }
		private:
			TObject *object;
			TMethod method;
		};
		
		class TransitionBlock {
		public:
			TransitionBlock();
			
			virtual void setName(std::string name);
			virtual std::string getName();
			
			virtual Runnable* getRunnableA();
			virtual Runnable* getRunnableB();
			
			virtual void runA() = 0;
			virtual void runB() = 0;
			
		private:
			std::string name;
			TransitionAction<TransitionBlock> runnableA;
			TransitionAction<TransitionBlock> runnableB;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_TRANSITIONBLOCK_HPP_ */