#ifndef ORG_EEROS_CONTROL_INVERSEKINEMATIC_HPP_
#define ORG_EEROS_CONTROL_INVERSEKINEMATIC_HPP_

#include <eeros/types.hpp>
#include <eeros/control/Block1i1o.hpp>
#include <vector>

namespace eeros {
	namespace control {
		
		class InverseKinematic: public Block1i1o {

		public:
			InverseKinematic(std::vector<double> cartesianCoords, sigdim_t dim);
			virtual ~InverseKinematic();
			virtual void run();
			
		protected:
			bool coordInWorkspace = false; 
				
		private:
			sigdim_t dim; 
			const double lArm1 = 0.25;
			const double lArm2 = 0.25;	
			double zMax = 0.40;
			double zMin = 0.98;
			double a = 0.1; 
			double b = 0.1;
			double r2 = lArm1+lArm2;
  
			std::vector<double> cartesianCoords;
			virtual void checkCoordinates(std::vector<double> cartesianCoords);
		};
	};
};

#endif /* ORG_EEROS_CONTROL_INVERSEKINEMATIC_HPP_ */