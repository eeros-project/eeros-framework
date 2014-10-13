#ifndef ORG_EEROS_CONTROLTIMEDOMAIN_HPP
#define ORG_EEROS_CONTROLTIMEDOMAIN_HPP

#include <list>
#include <string>
#include <eeros/core/PeriodicThread.hpp>
#include <eeros/core/Runnable.hpp>

namespace eeros {
	namespace control {

		class TimeDomain : public PeriodicThread {
		public:
			TimeDomain(std::string name, double period, bool realtime);
			virtual void addBlock(Runnable* block);
//			virtual void sortBlocks();
			
		protected:
			virtual void run();
			
		private:
			std::string name;
			std::list<Runnable*> blocks;
		};

	};
};

#endif // ORG_EEROS_CONTROLTIMEDOMAIN_HPP
