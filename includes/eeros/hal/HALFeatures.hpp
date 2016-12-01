#ifndef ORG_EEROS_HALFEATURES_HAL_HPP_
#define ORG_EEROS_HALFEATURES_HAL_HPP_

#include <map>
#include <string>

namespace eeros{
	namespace hal{
		enum Direction {
			In,
			Out
		};
		
		enum Type {
			Logic,
			Real
		};
		
		const std::map<std::string, Direction> directionOfChannel = {
			{ "DigIn", 	In },
			{ "DigOut", 	Out },
			{ "DAC", 	Out },
			{ "ADC", 	In },
			{ "PWM", 	Out },
			{ "FQD", 	In }
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