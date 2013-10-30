#ifndef ORG_EEROS_EXAMPLES_SEQUENCER_MYSEQUENCER_HPP_
#define ORG_EEROS_EXAMPLES_SEQUENCER_MYSEQUENCER_HPP_

#include <eeros/sequencer/Sequencer.hpp>

namespace eeros {
	namespace examples {
		namespace sequencer {

			class MySequencer : public eeros::sequencer::Sequencer {
			public:
				MySequencer(std::string name);
			};

		};
	};
};

#endif