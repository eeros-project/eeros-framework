#ifndef ORG_EEROS_EXAMPLES_SEQUENCER_MYERRORHANDLERB_HPP_
#define ORG_EEROS_EXAMPLES_SEQUENCER_MYERRORHANDLERB_HPP_

#include <string>

#include <eeros/sequencer/ErrorHandler.hpp>

namespace eeros {
	namespace examples {
		namespace sequencer {

			class MyErrorHandlerB : public eeros::sequencer::ErrorHandler {
			public:
				MyErrorHandlerB(std::string name);
				virtual ~MyErrorHandlerB(void);

				virtual void run();

				void Reset();
				void Referencing();
				void RestartSequence();

			};
		};
	};
};
#endif