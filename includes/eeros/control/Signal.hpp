#ifndef ORG_EEROS_CONTROL_SIGNAL_HPP_
#define ORG_EEROS_CONTROL_SIGNAL_HPP_

#include <string>
#include <list>
#include <type_traits>
#include <limits>
#include <eeros/types.hpp>
#include <eeros/control/SignalInterface.hpp>

namespace eeros {
	namespace control {
		
		extern uint16_t signalCounter;
			
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
			
			virtual void setName(std::string name) {
				this->name = name;
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
			
			virtual T getValue() const {
				return value;
			}
			
			virtual void setValue(T newValue) {
				value = newValue;
			}
			
			template < typename VT >
			void setValue(VT newValue) {
				value = newValue;
			}
			
			virtual timestamp_t getTimestamp() const {
				return timestamp;
			}
			
			virtual void setTimestamp(timestamp_t newTimestamp) {
				timestamp = newTimestamp;
			}
			
			virtual void clear() {
				_clear<T>();
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
			template <typename S> typename std::enable_if<std::is_arithmetic<S>::value>::type _clear() {
				value = std::numeric_limits<double>::quiet_NaN();
			}
			
			template <typename S> typename std::enable_if<!std::is_arithmetic<S>::value>::type _clear() {
				value.fill(std::numeric_limits<double>::quiet_NaN());
			}
			
			static std::list<SignalInterface*> signalList;
			static Signal<T> illegalSignal;
		};
		
		template < typename T>
		std::list<SignalInterface*> Signal<T>::signalList;
		
		template < typename T>
		Signal<T> Signal<T>::illegalSignal;
	
		/********** Print functions **********/
		template <typename T>
		std::ostream& operator<<(std::ostream& os, Signal<T>& signal) {
			os << "Signal: '" << signal.getName() << "' timestamp = " << signal.getTimestamp() << " value = " << signal.getValue(); 
		}
		template <typename T>
		std::ostream& operator<<(std::ostream& os, Signal<T>* signal) {
			os << "Signal: '" << signal.getName() << "' timestamp = " << signal.getTimestamp() << " value = " << signal.getValue(); 
		}

	};
};

#endif /* ORG_EEROS_CONTROL_SIGNAL_HPP_ */
