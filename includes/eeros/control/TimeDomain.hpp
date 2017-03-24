#ifndef ORG_EEROS_CONTROLTIMEDOMAIN_HPP
#define ORG_EEROS_CONTROLTIMEDOMAIN_HPP

#include <list>
#include <string>
#include <eeros/core/Runnable.hpp>
#include <eeros/control/NotConnectedFault.hpp>
#include <eeros/control/NaNOutputFault.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/safety/SafetyLevel.hpp>

namespace eeros {
	namespace control {

		using namespace safety;

		class TimeDomain : public virtual Runnable {
		public:
			TimeDomain(std::string name, double period, bool realtime);
			virtual void addBlock(Runnable* block);
			virtual void addBlock(Runnable& block);
			virtual void removeBlock(Runnable* block);
			virtual void removeBlock(Runnable& block);
			
			std::string getName();
			double getPeriod();
			bool getRealtime();
			void registerSafetyEvent(SafetySystem* ss, SafetyEvent* e);

			virtual void run();
			virtual void start();
			virtual void stop();
			
		private:
			std::string name;
			double period;
			bool realtime;
			bool running = true;
			std::list<Runnable*> blocks;
			SafetySystem* safetySystem;
			SafetyEvent* safetyEvent;
		};

	};
};

#endif // ORG_EEROS_CONTROLTIMEDOMAIN_HPP
