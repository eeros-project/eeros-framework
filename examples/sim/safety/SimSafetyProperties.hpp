#ifndef CH_NTB_SIMSAFETYPROPERTIES_HPP_
#define CH_NTB_SIMSAFETYPROPERTIES_HPP_

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/ScalableInput.hpp>
#include <eeros/safety/SafetyLevel.hpp>
	
	class SimControlSystem;
	
	// Define events
	enum {
	  doOff = 1,
	};
	
	// Define levels
	enum {
	  off = 1,
	};
	
	class SimSafetyProperties : public eeros::safety::SafetyProperties {

		public:
			SimSafetyProperties(SimControlSystem* cs);
			virtual ~SimSafetyProperties();
		
			// outputs
			// inputs
		
		private:
			SimControlSystem* controlSys;
			eeros::safety::SafetyLevel slOff;
	};

	#endif // CH_NTB_SIMSAFETYPROPERTIES_HPP_