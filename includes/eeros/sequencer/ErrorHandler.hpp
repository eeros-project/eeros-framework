#ifndef ORG_EEROS_SEQUENCER_ERRORHANDLER_HPP_
#define ORG_EEROS_SEQUENCER_ERRORHANDLER_HPP_

#include <string>
#include <list>

class list;

namespace eeros{
	namespace sequencer{
		class ErrorHandler{
		public:
			ErrorHandler(std::string name);
			virtual ~ErrorHandler();
			virtual void run()=0;
			static ErrorHandler* getErrorHandler(std::string name);
			std::string getName();
		private:
			std::string handlerName;
			static std::list<ErrorHandler*> allErrorHandlers;
		};
	};//namespace sequencer
}; //namsespace eeros

#endif