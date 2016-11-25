#ifndef ORG_EEROS_HALFEATURES_HAL_HPP_
#define ORG_EEROS_HALFEATURES_HAL_HPP_

#include <map>
#include <string>

namespace eeros{
	namespace hal{
		enum Direction {
			Input,
			Output
		};
		
		enum Type {
			Logic,
			Real
		};
		
		const std::map<std::string, Direction> directionOfChannel = {
			{ "DigIn", 	Input },
			{ "DigOut", 	Output },
			{ "DAC", 	Output },
			{ "ADC", 	Input },
			{ "PWM", 	Output },
			{ "FQD", 	Input }
		};
		
		const std::map<std::string, Type> typeOfChannel = {
			{ "DigIn", 	Logic },
			{ "DigOut", 	Logic },
			{ "DAC", 	Real },
			{ "ADC", 	Real },
			{ "PWM", 	Real },
			{ "FQD", 	Real }
		};
	};
};

#endif /* ORG_EEROS_HALFEATURES_HAL_HPP_ */