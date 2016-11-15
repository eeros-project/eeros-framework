#ifndef ORG_EEROS_CONTROLTIMEDOMAIN_HPP
#define ORG_EEROS_CONTROLTIMEDOMAIN_HPP

#include <list>
#include <string>
#include <eeros/core/Runnable.hpp>

namespace eeros {
	namespace control {

		class TimeDomain : public virtual Runnable {
		public:
			TimeDomain(std::string name, double period, bool realtime);
			virtual void addBlock(Runnable* block);
			
			std::string getName();
			double getPeriod();
			bool getRealtime();

			virtual void run();
			
		private:
			std::string name;
			double period;
			bool realtime;
			std::list<Runnable*> blocks;
		};

	};
};

#endif // ORG_EEROS_CONTROLTIMEDOMAIN_HPP
