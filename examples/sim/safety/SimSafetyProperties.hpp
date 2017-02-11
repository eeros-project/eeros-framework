#ifndef CH_NTB_SIMSAFETYPROPERTIES_HPP_
#define CH_NTB_SIMSAFETYPROPERTIES_HPP_

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/ScalableInput.hpp>
#include <eeros/safety/SafetyLevel.hpp>
	
	class SimControlSystem;
	
	class SimSafetyProperties : public eeros::safety::SafetyProperties {

		public:
			SimSafetyProperties(SimControlSystem* cs);
			virtual ~SimSafetyProperties();
		
			// outputs
			// inputs
			
			eeros::safety::SafetyLevel slOff;
			eeros::safety::SafetyLevel slRunning;
		
		private:
			SimControlSystem* controlSys;
			
			eeros::safety::SafetyEvent seRun;
	};

	#endif // CH_NTB_SIMSAFETYPROPERTIES_HPP_