#ifndef ORG_EEROS_CONTROL_PATHPLANNER_HPP_
#define ORG_EEROS_CONTROL_PATHPLANNER_HPP_

#include <eeros/types.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/RealSignalInput.hpp>
#include <eeros/control/RealSignalOutput.hpp>
#include <vector>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/Mutex.hpp>

namespace eeros {
	namespace control {
		
		enum TrajectoryType { linearVelocity = 0, limitedJerk = 1, limitedJerkSquare = 2, limitedSnap = 3, trigonometric = 4 };
		
		class PathPlanner: public Block {
  
		public:
			PathPlanner(std::vector<double> velMax, std::vector<double> accMax, std::vector<double> decMax, double dt = 0.001, sigdim_t dim = 1, TrajectoryType trajType = linearVelocity);
			PathPlanner(std::vector<double> velMax, std::vector<double> accMax, double dt = 0.001, sigdim_t dim = 1, TrajectoryType trajType = linearVelocity);
			virtual ~PathPlanner();
			
			virtual void run();
			virtual void reset();
			virtual void enable();
			virtual void disable();
			virtual void setVelMax(std::vector<double> velMax);
			virtual void setAccMax(std::vector<double> accMax);
			virtual void setDecMax(std::vector<double> decMax);
			virtual void addPosition(std::vector<double> posFinal);
			virtual void addHelpPosition(std::vector<double> posFinal);
			virtual void setInitialPosition(std::vector<double> posInit);
			
			virtual RealSignalOutput& getPosOut();
			virtual RealSignalOutput& getVelOut();
			virtual RealSignalOutput& getAccOut();
			
			bool pathEnded = false;
				
		protected:
			bool enabled = false;
			bool first = true;
			bool trajParamSet = false;
			bool smoothTrajectory = false;
			bool helpParamError = false;
				
			RealSignalOutput acc;
			RealSignalOutput vel;
			RealSignalOutput pos;
			
		private:
			sigdim_t dim;
			int indexAddPos, indexReadPos;
			bool prevNewValue;
			TrajectoryType trajType;
			Mutex mutex;
	
			std::vector<bool> isNewValue, isHelpValue;
			std::vector<double> velMax, accMax, decMax; 
			std::vector<double> distance, distance1, distance2;
			std::vector<double> posFinalPrev, posPrev;
			
			static const int dimBuffer = 10; 
			eeros::math::Matrix<10,4> posFinal;
		
			std::vector<double> posHelp, posA, posB, cCircle;
	
			const double dCircle = 0.1;
			double rCircle, thetaCircleInit, thetaCircleEnd, thetaPrev;
			double lambda, velNorm, velNorm1, velNorm2, velNormC; 
			double dt, dT1, dT2, dT3, dT4, dTC; 
			double tOffset, time, timePrev, timeScaled;
	
			virtual void checkPathEnd();
			virtual void setLambda(TrajectoryType trajType);	
			virtual void setSmoothCurvesParameters();
			virtual void setTrajectoryParameters();
			virtual double setPosGain(double k, double dK);
			virtual double setVelGain(double k, double dK);
			virtual double setAccGain(double k, double dK);
		};
	};
};

#endif /* ORG_EEROS_CONTROL_PATHPLANNER_HPP_ */