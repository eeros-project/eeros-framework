#ifndef ORG_EEROS_CONTROL_PATHPLANNER_HPP_
#define ORG_EEROS_CONTROL_PATHPLANNER_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/math/Matrix.hpp>
#include <mutex>
#include <atomic>

typedef eeros::math::Matrix<4,1,double> AxisVector;

namespace eeros {
	namespace control {
			
			class PathPlanner: public eeros::control::Block {
			
			public:
				PathPlanner(AxisVector velMax, AxisVector accMax, AxisVector decMax, double dt);
				
				virtual eeros::control::Output<AxisVector>& getPosOut();
				virtual eeros::control::Output<AxisVector>& getVelOut();
				virtual eeros::control::Output<AxisVector>& getAccOut();
				virtual eeros::control::Output<AxisVector>& getJerkOut();
				
				virtual void setInitPos(AxisVector initPos);
				virtual bool move(AxisVector p, bool jerk_limit = false);
// 				virtual bool move(std::array<AxisVector, 100> array, bool limitOn = false);
				virtual bool move(std::vector<AxisVector> array, bool limitOn = false);
				virtual bool move(std::string filename, double time = 1.0, AxisVector end_position = 1.0);
				virtual bool posReached();
				
				virtual void setMaxSpeed(AxisVector speed);
				virtual void setMaxAcc(AxisVector acc);
				virtual void setMaxDec(AxisVector dec);
				
				virtual void reset();
				virtual void run();
			
	// 		protected:
				eeros::control::Output<AxisVector> posOut;
				eeros::control::Output<AxisVector> velOut;
				eeros::control::Output<AxisVector> accOut;
				eeros::control::Output<AxisVector> jerkOut;
				
				std::mutex mtx;
				
				virtual bool calculateCoefficients_fromPosition();
				virtual bool calculateCoefficients_fromJerk();
				
				double dt, t;
				double dT1, dT2, dT3;
				AxisVector cj1, cj2, cj3, ca1, ca2, ca3, cv1, cv2, cv3, cp1, cp2, cp3; 
				
				AxisVector /*double*/ a_prev, v_prev, p_prev, j_prev;
				double dT, dTold;
				
				int k = 0;
				int points_nr;
				int segments_nr;
			
				std::atomic<bool> finish; // = true
				bool finish_segment = true;
				bool segment_set = false;
				bool jerk_limit_on = false;
				
				double j0, s0; 
				
				AxisVector velMax, accMax, decMax; 
				std::vector<AxisVector> last = std::vector<AxisVector>(4);
				std::vector<AxisVector> positions = std::vector<AxisVector>(100);
// 				std::array<AxisVector, 4> last;
// 				std::array<AxisVector, 100> positions;
				
				eeros::math::Matrix<100,5,AxisVector> coefficients;
		};
	};
};

#endif /* ORG_EEROS_CONTROL_PATHPLANNER_HPP_ */
