#ifndef CH_NTB_PARSERTESTSAFETYPROPERTIES_HPP_
#define CH_NTB_PARSERTESTSAFETYPROPERTIES_HPP_

#include <eeros/safety/SafetyProperties.hpp>
#include <eeros/hal/HAL.hpp>
#include <eeros/hal/ScalablePeripheralInput.hpp>
#include <eeros/safety/SafetyLevel.hpp>
	
	class ParserTestControlSystem;
	
	// Define events
	enum {
	  doOff = 1,
	};
	
	// Define levels
	enum {
	  off = 1,
	};
	
	class ParserTestSafetyProperties : public eeros::safety::SafetyProperties {

		public:
			ParserTestSafetyProperties(ParserTestControlSystem* cs);
			virtual ~ParserTestSafetyProperties();
		
			// outputs
			// inputs
		
		private:
			ParserTestControlSystem* controlSys;
			eeros::safety::SafetyLevel slOff;
	};

	#endif // CH_NTB_PARSERTESTSAFETYPROPERTIES_HPP_