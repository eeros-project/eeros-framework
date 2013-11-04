#ifndef ORG_EEROS_SEQUENCER_ERRORHANDLER_HPP_
#define ORG_EEROS_SEQUENCER_ERRORHANDLER_HPP_

#include <string>
#include <list>

class list;

namespace eeros{
	namespace sequencer{
		/**
		 * This class can be used to define a sequence in exception, 
		 * this means, if an exception occurs then you can go to a defined Step.
		 * All ErrorHanlders are saved in the static list allErrorHandlers, so you can reuse the object
		 */
		class ErrorHandler{
		public:
			/**
			 * Constructor saves the ErrorHandler in the list allErrorHandlers
			 */
			ErrorHandler(std::string name);
			
			/**
			 * Removes the ErrorHandler from the list allErrorHandlers.
			 */
			virtual ~ErrorHandler();
			
			/**
			 * The sequence calls the run methode of an exception occurs duriogn the run method of the sequence
			 */
			virtual void run()=0;
			
			/**
			 * Returns the ErrorHandler by name
			 */
			static ErrorHandler* getErrorHandler(std::string name);
			
			/**
			 * Removes the ErrorHandler by name from the list allErrorHandlers.
			 */
			static void removeErrorHandler(std::string name);
			
			/**
			 * Returns the name
			 */
			std::string getName();
		private:
			std::string handlerName;
			static std::list<ErrorHandler*> allErrorHandlers;
		};
	};//namespace sequencer
}; //namsespace eeros

#endif