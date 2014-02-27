#include <eeros/control/InverseKinematic.hpp>
#include <math.h>

using namespace eeros::control;

InverseKinematic::InverseKinematic(std::vector<double> cartesianCoords, sigdim_t dim) {
	this->cartesianCoords = cartesianCoords;
}

InverseKinematic::~InverseKinematic() { 
	    // nothing to do...
}

void InverseKinematic::checkCoordinates(std::vector<double> cartesianCoords){
// 	if  (((cartesianCoords[0]*cartesianCoords[0]+cartesianCoords[1]*cartesianCoords[1])<(lArm1*lArm1)) || ((cartesianCoords[0]*cartesianCoords[0]+cartesianCoords[1]*cartesianCoords[1])>(r2*r2))) {
//     
//     if(cartesianCoords[1]<=0){
// 	  if((lArm2*lArm2<((cartesianCoords[0]+a)*(cartesianCoords[0]+a)+(cartesianCoords[1]-b)*(cartesianCoords[1]-b))&&(cartesianCoords[0]<=0)))
// 	      coordInWorkspace = false;
// 	  if((lArm2*lArm2<((cartesianCoords[0]-a)*(cartesianCoords[0]-a)+(cartesianCoords[1]+b)*(cartesianCoords[1]+b))&&(cartesianCoords[0]<=0)))
// 	      coordInWorkspace = false;
//       }
//       else
// 		coordInWorkspace = false;
//   }
//   else
//       coordInWorkspace = true;
// 
//   if((!coordInWorkspace)||(cartesianCoords[2] <= zMin) ||(cartesianCoords[2] >= zMax))
//       coordInWorkspace = false;
//   else
//     coordInWorkspace = true;
}

void InverseKinematic::run(){
	double jointCoords[4];
	
	jointCoords[1] = acos((-lArm1*lArm1-lArm2*lArm2+cartesianCoords[0]*cartesianCoords[0]+cartesianCoords[1]*cartesianCoords[1])/(2*lArm1*lArm2));

	// avoid singularity, phi2 doesn't have to be equals to zero
	if(jointCoords[1]>-0.02 && jointCoords[1] <0.02){
		if(jointCoords[1]<0)
			jointCoords[1] = -0.02;
		else
			jointCoords[1] = 0.02;
	}
  
	// calculate phi1
	if (jointCoords[1] >= 0)
		jointCoords[0] = atan(cartesianCoords[1]/cartesianCoords[0]) - acos((cartesianCoords[0]*cartesianCoords[0]+cartesianCoords[1]*cartesianCoords[1]+lArm1*lArm1-lArm2*lArm2)/(2*lArm1*sqrt(cartesianCoords[0]*cartesianCoords[0]+cartesianCoords[1]*cartesianCoords[1])));
	else
		jointCoords[0] = atan(cartesianCoords[1]/cartesianCoords[0]) + acos((cartesianCoords[0]*cartesianCoords[0]+cartesianCoords[1]*cartesianCoords[1]+lArm1*lArm1-lArm2*lArm2)/(2*lArm1*sqrt(cartesianCoords[0]*cartesianCoords[0]+cartesianCoords[1]*cartesianCoords[1])));
    
	jointCoords[2] = -cartesianCoords[2];
	jointCoords[3] = jointCoords[0]+jointCoords[1]-cartesianCoords[3];
	
// 	checkCoordinates(jointCoords);
	
// 	if(coordInWorkspace){
		for(sigdim_t i = 0; i<dim; i++)
			out.setValue(jointCoords[i], i);
// 	}
// 	else {
// 		throw EEROSException("Coordinates out of workspace");
// 	}
}

		

	