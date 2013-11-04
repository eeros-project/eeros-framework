#include <eeros/sequencer/ErrorHandler.hpp>

using namespace eeros::sequencer;

std::list<eeros::sequencer::ErrorHandler*> ErrorHandler::allErrorHandlers;

ErrorHandler::ErrorHandler(std::string name) : handlerName(name){
	ErrorHandler::allErrorHandlers.push_back(this);
}

ErrorHandler::~ErrorHandler(){
	ErrorHandler::removeErrorHandler(handlerName);
}

std::string ErrorHandler::getName(){
	return handlerName;
}

ErrorHandler* ErrorHandler::getErrorHandler(std::string name){
	std::list<ErrorHandler*>::iterator iter = eeros::sequencer::ErrorHandler::allErrorHandlers.begin();
		while(iter != eeros::sequencer::ErrorHandler::allErrorHandlers.end()){
			if((*iter)->getName().compare(name) == 0){
				return *iter;
			}
			iter++;
		}
		return 0;
}

void ErrorHandler::removeErrorHandler(std::string name){
	std::list<ErrorHandler*>::iterator iter = eeros::sequencer::ErrorHandler::allErrorHandlers.begin();
		while(iter != eeros::sequencer::ErrorHandler::allErrorHandlers.end()){
			if((*iter)->getName().compare(name) == 0){
				allErrorHandlers.erase(iter);
				return;
			}
			iter++;
		}
		return 0;
}