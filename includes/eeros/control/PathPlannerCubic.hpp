#ifndef ORG_EEROS_CONTROL_PATHPLANNER_CUBIC_HPP_
#define ORG_EEROS_CONTROL_PATHPLANNER_CUBIC_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/logger/Logger.hpp>
#include <mutex>
#include <atomic>

namespace eeros {
	namespace control {
			
		class PathPlannerCubic: public eeros::control::Block {
		public:
			PathPlannerCubic(double dt);
			
			virtual eeros::control::Output<double>& getPosOut();
			virtual eeros::control::Output<double>& getVelOut();
			virtual eeros::control::Output<double>& getAccOut();
			virtual eeros::control::Output<double>& getJerkOut();
			
			virtual void init(std::string filename);
			virtual void setInitPos(double initPos);
			virtual bool move(double init_pos);
			virtual bool move(double time, double init_pos, double end_pos); 
			virtual bool posReached();				
			virtual void reset();
			virtual void run();
				
		private:
			void clear();
			void scalePath(double time, double end_pos);
			eeros::control::Output<double> posOut;
			eeros::control::Output<double> velOut;
			eeros::control::Output<double> accOut;
			eeros::control::Output<double> jerkOut;
			
			bool finished;
			double dt;
			double prevJerk, prevAcc, prevVel, prevPos;
			std::vector<double> timeCoeffRaw, jerkCoeffRaw, accCoeffRaw, velCoeffRaw, posCoeffRaw;
			std::vector<double> timeCoeff, jerkCoeff, accCoeff, velCoeff, posCoeff, posCoeffShifted;
			std::mutex m;
			eeros::logger::Logger log;
		};
	};
};

#endif /* ORG_EEROS_CONTROL_PATHPLANNER_CUBIC_HPP_ */
