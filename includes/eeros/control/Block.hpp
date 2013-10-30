#ifndef ORG_EEROS_CONTROL_BLOCK_HPP_
#define ORG_EEROS_CONTROL_BLOCK_HPP_

#include <string>
#include <eeros/core/Runnable.hpp>

namespace eeros {
	namespace control {
		
		class Block : public Runnable {
		public:
			virtual void setName(std::string name);
			virtual std::string getName();
			
		private:
			std::string name;
		};

	};
};

#endif /* ORG_EEROS_CONTROL_BLOCK_HPP_ */