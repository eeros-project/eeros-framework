#ifndef ORG_EEROS_CONTROL_SIGNAL_HPP_
#define ORG_EEROS_CONTROL_SIGNAL_HPP_

#include <string>
#include <list>
#include <type_traits>
#include <eeros/types.hpp>
#include <eeros/control/SignalInterface.hpp>

namespace eeros {
	namespace control {
		
		template < typename T = double >
		class Signal : public SignalInterface {
		public:
			Signal() {
				id = signalCounter++;
// 				signalList.push_back(this);
			}
			
			virtual sigid_t getId() const {
				return static_cast<sigid_t>(id) << 16;
			}
			
			virtual std::string getName() const {
				return name;
			}
			
			virtual void setName(std::string n) {
				name = name;
			}
			
			virtual std::string getLabel() const {
// 				std::stringstream label;
// 				label << '#' << id << '.' << index << ": " << getName(index);
// 				if(getAffiliation(index) != "") {
// 					label << " (" << getAffiliation(index) << ')';
// 				}
// 				if(getUnit(index) != "") {
// 					label << " [" << getUnit(index) << ']';
// 				}
// 				return label.str();
				return name; // TODO
			}
			
			virtual T getValue() const { // TODO better return a reference?
				return value;
			}
			
			virtual void setValue(T newValue) {
				value = newValue;
			}
			
			virtual timestamp_t getTimestamp() const {
				return timestamp;
			}
			
			virtual void setTimestamp(timestamp_t newTimestamp) {
				timestamp = newTimestamp;
			}
			
			virtual void clear() {
				value = 0;
			}
			
			Signal<T>& operator= (Signal<T> right) {
				value = right.value;
				timestamp = right.timestamp;
				return *this;
			}
			
			Signal<T>& operator= (T right) {
				value = right;
				return *this;
			}
			
			static Signal<T>& getIllegalSignal() {
				return illegalSignal;
			}
			
			static std::list<SignalInterface*> getSignalList() {
				return signalList;
			}
			
			static SignalInterface* getSignalById(sigid_t id) {
				std::list<SignalInterface*>::iterator i = signalList.begin();
				while(i != signalList.end()) {
					if((*i)->getId() == id) {
						return (*i);
					}
					i++;
				}
				return NULL;
			}
			
		protected:
			T value;
			timestamp_t timestamp;
			sigid_t id;
			std::string name;
		
		private:
			static std::list<SignalInterface*> signalList;
			static uint16_t signalCounter;
			static Signal<T> illegalSignal;
		};
		
		template < typename T>
		std::list<SignalInterface*> Signal<T>::signalList;
		
		template < typename T>
		uint16_t Signal<T>::signalCounter = startSignalId;
		
		template < typename T>
		Signal<T> Signal<T>::illegalSignal;
		
	};
};

#endif /* ORG_EEROS_CONTROL_SIGNAL_HPP_ */
