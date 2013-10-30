#ifndef ORG_EEROS_EXAMPLES_SEQUENCER_MYERRORHANDLERA_HPP_
#define ORG_EEROS_EXAMPLES_SEQUENCER_MYERRORHANDLERA_HPP_

#include <string>

#include <eeros/sequencer/ErrorHandler.hpp>

namespace eeros {
	namespace examples {
		namespace sequencer {

			class MyErrorHandlerA : public eeros::sequencer::ErrorHandler {
			public:
				MyErrorHandlerA(std::string name);
				virtual ~MyErrorHandlerA(void);

				virtual void run();

				void Reset();
				void Referencing();
				void RestartSequence();

			};

		};
	};
};
#endif