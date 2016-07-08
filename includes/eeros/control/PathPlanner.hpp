#ifndef CH_NTB_PEEPINGPANEL_PATHPLANNER_HPP_
#define CH_NTB_PEEPINGPANEL_PATHPLANNER_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <atomic>
#include "../types.hpp"
#include <array>

namespace pathos {
	namespace peepingpanel {
	
		class PathPlanner: public eeros::control::Block {
			
		public:
			PathPlanner(AxisVector velMax, AxisVector accMax, AxisVector decMax, double dt);
			
			virtual eeros::control::Output<AxisVector>& getPosOut();
			virtual eeros::control::Output<AxisVector>& getVelOut();
			virtual eeros::control::Output<AxisVector>& getAccOut();
			virtual eeros::control::Output<AxisVector>& getJerkOut();
			
			virtual void setInitPos(AxisVector initPos);
			virtual bool move(AxisVector p, bool jerk_limit = false);
			virtual bool move(std::array<AxisVector, 100> array, bool limitOn = false);
			virtual bool move(std::string filename, double time_tot, AxisVector end_position);
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
			
			virtual bool calculateCoefficients_fromPosition();
			virtual bool calculateCoefficients_fromJerk();
			
			double dt, t;
			double dT1, dT2, dT3;
			AxisVector cj1, cj2, cj3, ca1, ca2, ca3, cv1, cv2, cv3, cp1, cp2, cp3; 
			
			double a_prev, v_prev, p_prev, j_prev, dT, dTold;
			
			int k = 0;
			int points_nr;
			int segments_nr;
		
			std::atomic<bool> finish; // = true;
			bool finish_segment = true;
			bool segment_set = false;
			bool jerk_limit_on = false;
			
			double j0, s0; 
			
			AxisVector velMax, accMax, decMax; 
			std::array<AxisVector, 4> last;
			std::array<AxisVector, 100> positions;
			
			eeros::math::Matrix<100,5,double> coefficients;
	};
	};
}

#endif /* CH_NTB_PEEPINGPANEL_PATHPLANNER_HPP_ */
