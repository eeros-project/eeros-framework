#ifndef ORG_EEROS_CONTROL_PATHPLANNER_TRAPEZOID_HPP_
#define ORG_EEROS_CONTROL_PATHPLANNER_TRAPEZOID_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/math/Matrix.hpp>
#include <eeros/core/System.hpp>
#include <eeros/logger/Logger.hpp>
#include <mutex>
#include <atomic>
#include <iostream>
#include <fstream>

//typedef eeros::math::Matrix<4,1,double> AxisVector;

namespace eeros {
	namespace control {
			
		template < typename T = eeros::math::Matrix<1,1,double> >
		class PathPlannerTrapezoid: public eeros::control::Block {
		public:
			PathPlannerTrapezoid(T velMax, T accMax, T decMax, double dt) : velMax(velMax), accMax(accMax), decMax(decMax), dt(dt), posOut(this), velOut(this), accOut(this), jerkOut(this), log('F') {
				coefficients.zero();
				coefficients.transpose();
				posOut.getSignal().clear();
				velOut.getSignal().clear();
				accOut.getSignal().clear();
				jerkOut.getSignal().clear();
				finished = true;
			}
			~PathPlannerTrapezoid() { }
			
			virtual eeros::control::Output<T>& getPosOut() {return posOut;}
			virtual eeros::control::Output<T>& getVelOut() {return velOut;}
			virtual eeros::control::Output<T>& getAccOut() {return accOut;}
			virtual eeros::control::Output<T>& getJerkOut() {return jerkOut;}
			
			virtual void setInitPos(T initPos) {
				T z; 
 				z.zero();
				std::vector<T> r = std::vector<T>(4);
				r[0] = initPos;
				r[1] = z;
				r[2] = z;
				r[3] = z;

				last = r;
				finished = true;
			}
			
			virtual bool move(T pos, bool jerkLimit = false) {
				if (!finished) return false;
				coefficients.zero();
				positions[0] = pos;
				this->jerkLimit = jerkLimit;
				nofPoints = 1;
				if (calculateCoefficients_fromPosition()) {
					finished = false;
					finish_segment = false;
					t = 0;
					dTold = 0;
					k = 0;
				} else {
					finished = true;
					finish_segment = true;
					t = 0;
					dTold = 0;
					k = 0;
				}
				return true;
			}
			
			virtual bool move(std::vector<T> pos, bool jerkLimit = false) {
				if (!finished) return false;
				coefficients.zero();
				positions = pos;
				this->jerkLimit = jerkLimit;
				
				// find end of array
				T zero; 
				zero.zero();
				int nofPoints = 0;

				for (int i = 0; i < pos.size(); i++) {
					if (pos[i] < zero) i = pos.size(); else nofPoints++;
				}
				
				// Safety check, if -1 at end of array missing, don't calculate trajectory
			// 	if(nofPoints == array.size()){
			// 		std::cout << "return false" << std::endl;
			// 		return false;
			// 	}
				this->nofPoints = nofPoints;
					
				if (calculateCoefficients_fromPosition()){
					finished = false;
					finish_segment = false;
					t = 0;
					dTold = 0;
					k = 0;
				} else {
					finished = true;
					finish_segment = true;
					t = 0;
					dTold = 0;
					k = 0;
				}	
				return true;
			}
			
			virtual bool move(std::string filename, double timeTot = 1.0, T end_position = 1.0) {
				if(!finished) return false;
				coefficients.zero();
				jerkLimit = false;
	
				std::fstream file;
				file.open(filename, std::fstream::in);
				if (!file.is_open()) throw Fault("Path planner cannot open path file");

				std::vector<double> input_time = std::vector<double>(100);
				std::vector<double> input_jerk = std::vector<double>(100);
				
				// set array dimension, time and jerk values
				int nofPoints = 0;
				for(int j = 0; j < input_time.size(); j++){
					file >> input_time[j]; 
					file >> input_jerk[j];
					if (input_time[j] < 0.0) j = input_time.size(); else nofPoints++;
				}
				file.close();
				this->nofPoints = nofPoints;
				
				// Norm and scale curve
				for(int j = 0; j < nofPoints; j++){
					input_time[j] = input_time[j] * timeTot; 
					input_jerk[j] = input_jerk[j] * end_position[0] / (timeTot*timeTot*timeTot); // TODO end position universal for axisVectors
				}
				
				// TEST
				bool test = true;
				if(test == true){
					double rounded_points_nr = 0;
			// 		std::array<double, 100> rounded_input_time; 
			// 		std::array<double, 100> rounded_input_jerk; 
			// 		std::array<double, 100> dT;
			// 		std::array<double, 100> rest;
			// 		std::array<int   , 100> rest2;
					std::vector<double> rounded_input_time = std::vector<double>(100);
					std::vector<double> rounded_input_jerk = std::vector<double>(100);
					std::vector<double> dT = std::vector<double>(100);
					std::vector<double> rest = std::vector<double>(100);
					std::vector<int> rest2 = std::vector<int>(100);
					
					double timePrev = 0.0;
					for(int j = 0; j < nofPoints; j++){
						// check if time intervals are multiple of sample time
						if(j == 0) {dT[j] = input_time[j];          }
						else       {dT[j] = dT[j-1] + input_time[j];}
							
						rest[j]  = dT[j] / dt;                              
						rest2[j] = static_cast<int>(std::floor(rest[j]));
						
						
						double roundDown;
						if(rest[j] != rest2[j]) {
							// calculate variation in "next init values" to be saved
							roundDown = rest2[j]*dt - timePrev;
							rounded_input_time[rounded_points_nr] = roundDown;
							rounded_input_jerk[rounded_points_nr] = input_jerk[j];
						
							double t1 = dT[j]-timePrev-roundDown; 
							double t2 = (dt-t1);
							if(j < nofPoints -1) {
								rounded_input_time[rounded_points_nr+1] = dt;
								rounded_input_jerk[rounded_points_nr+1] = (input_jerk[j] * t1 + input_jerk[j+1] * t2)/ dt;
							}
							
							// update indexes
							timePrev = dT[j] + t2; 
							if(j < nofPoints -1) rounded_points_nr = rounded_points_nr + 2;
							else rounded_points_nr = rounded_points_nr + 1;
						}
						else{
							rounded_input_time[rounded_points_nr] = dT[j] - timePrev;   //input_time[j];
							rounded_input_jerk[rounded_points_nr] = input_jerk[j];
						
							// update indexes
							timePrev = timePrev + rounded_input_time[rounded_points_nr];
							rounded_points_nr = rounded_points_nr + 1;
						}
					}
					nofPoints = rounded_points_nr;
					
					// final settings
					for(int j = 0; j < nofPoints; j++){
						input_time[j] = rounded_input_time[j];
						input_jerk[j] = rounded_input_jerk[j];
					}
				}
				// END TEST
				
				// save time and jerk into coefficients
				int j = 0;
				while (j < nofPoints) {
					coefficients(j,0) = input_time[j]; // dT
					coefficients(j,1) = input_jerk[j]; // cj1
					j++;
				}
				
				// Calculate coefficients like matlab program
				calculateCoefficients_fromJerk();
				finished = false;
				finish_segment = false;
				t = 0;
				dTold = 0;
				k = 0;
				return true;
			}
			
			virtual bool endReached() {return finished;}
			
			virtual void setMaxSpeed(T speed) {velMax = speed;}
			virtual void setMaxAcc(T acc) {accMax = acc;}
			virtual void setMaxDec(T dec) {decMax = dec;}
			
			virtual void reset() {
				finished = true;
				coefficients.zero();
				segment_set = false;
			}
			
			virtual void run() {
				std::vector<T> y = std::vector<T>(4);
				y = this->last;
				timestamp_t time = System::getTimeNs();	
				t += dt;
				
				if (!finished) {
					T a_prev, v_prev, p_prev, j_prev;
					double dT;
					double j0, s0; 
					if (k < segments_nr) {
						// define coefficients
						if (segment_set == false) {
							dT     = coefficients(k,0)(0);
							j_prev = coefficients(k,1);
							a_prev = coefficients(k,2);
							v_prev = coefficients(k,3);
							p_prev = coefficients(k,4);
							for (int i=0; i<j_prev.size() ;i++) {
								j0 =   6.0 * coefficients(k,2)(i) / coefficients(k,0)(i);
								s0 = -12.0 * coefficients(k,2)(i) / (coefficients(k,0)(i)*coefficients(k,0)(i));
							}
							segment_set = true;
						}
						
						// define trajectory
						if (t >= dTold && t < dT + dTold) { }
						else {
							finish_segment = true;
							segment_set = false;
							dTold = dTold + dT;
							k++;
						}
						
						y[0] = p_prev + v_prev * dt + (a_prev/2.0) * pow(dt,2) + (j_prev/6.0) * pow(dt,3);
						y[1] = v_prev + a_prev * dt + (j_prev/2.0) * pow(dt,2);
						y[2] = a_prev + j_prev * dt;
						
						if (jerkLimit) 
							y[2] = (s0 / 2.0) * (t-dTold) * (t-dTold) + j0 * (t-dTold);
						else 
							y[2] = a_prev + j_prev * dt;
						
						y[3] = j_prev;
						
						p_prev = y[0];
						v_prev = y[1];
						a_prev = y[2];
						j_prev = y[3];
					}
					if (finish_segment == true && k == segments_nr) finished = true;
				}
				// set last value
				this->last[0] = y[0];
				this->last[1] = y[1];
				this->last[2] = y[2];
				this->last[3] = y[3];

				posOut.getSignal().setValue( y[0]);
				velOut.getSignal().setValue( y[1]);
				accOut.getSignal().setValue( y[2]);
				jerkOut.getSignal().setValue(y[3]);

				posOut.getSignal().setTimestamp( time);
				velOut.getSignal().setTimestamp( time);
				accOut.getSignal().setTimestamp( time);
				jerkOut.getSignal().setTimestamp(time);
			}
		
		private:
			virtual bool calculateCoefficients_fromPosition() {
							
				T zero; zero = 0;
				for (int k = 0; k < nofPoints; k++) {
					std::vector<T> start = std::vector<T>(4);
					if (k == 0) start = last; 
					else start = {positions[k-1], zero, zero, zero};
					
					std::vector<T> end = std::vector<T>{positions[k], zero, zero, zero};
				
					T calcVelNorm, calcAccNorm, calcDecNorm;
					double velNorm, accNorm, decNorm, squareNormVel;
					T distance = end[0] - start[0];
					
					if (distance == zero) return false;
					
					// Define speeds and accelerations
					for(unsigned int i = 0; i < calcVelNorm.size(); i++) {
						calcVelNorm[i] = fabs(velMax[i] / distance[i]);
						calcAccNorm[i] = fabs(accMax[i] / distance[i]);
						calcDecNorm[i] = fabs(decMax[i] / distance[i]);
					}
					
					// Init velNorm, accNorm, decNorm and find minimum
					velNorm = calcVelNorm[0]; accNorm = calcAccNorm[0]; decNorm = calcDecNorm[0]; 
					for(unsigned int i = 0; i < calcVelNorm.size(); i++) {
						if(calcVelNorm[i] < velNorm) velNorm = calcVelNorm[i];
						if(calcAccNorm[i] < accNorm) accNorm = calcAccNorm[i];
						if(calcDecNorm[i] < decNorm) decNorm = calcDecNorm[i];
					}
					
					// Minimize velocity
					squareNormVel = sqrt(2 * (accNorm * decNorm) / (accNorm + decNorm));
					if(velNorm > squareNormVel) velNorm = squareNormVel; 
					
					// Calculate time intervals    
					double dT1 = velNorm / accNorm;
					double dT3 = velNorm / decNorm;
					double dT2 = 1 / velNorm - (dT1 + dT3) * 0.5;
					if (dT2 < 0) dT2 = 0;
					
					// Adaptation to timestamps
					dT1 = ceil(dT1 / dt) * dt;
					dT2 = ceil(dT2 / dt) * dt;
					dT3 = ceil(dT3 / dt) * dt; 
					// Adaptation of speed to new timestamps
					velNorm = 1/((dT2 + (dT1 + dT3)*0.5)*dt);
					
					T cj1 = 0.0;  
					T cj2 = 0.0;  
					T cj3 = 0.0;
					
					T ca1 = velNorm * dt / dT1 * distance; 
					T ca2 = 0.0; 
					T ca3 = velNorm * dt / dT3 * distance * (-1); 
					
					T cv1 = 0.0;  
					T cv2 = ca1 * dT1;  
					T cv3 = ca1 * dT1 + ca2 * dT2;  
					
					T cp1;
					if (k == 0) cp1 = last[0]; else	cp1 = positions[k-1];
					T cp2 = cp1 + cv1 * dT1 + ca1 * dT1 * dT1 / 2.0; 
					T cp3 = cp2 + cv2 * dT2 + ca2 * dT2 * dT2 / 2.0;

					coefficients(0+k*3,0) = dT1; coefficients(0+k*3,1) = cj1; coefficients(0+k*3,2) = ca1;
					coefficients(0+k*3,3) = cv1; coefficients(0+k*3,4) = cp1; 
					
					coefficients(1+k*3,0) = dT2; coefficients(1+k*3,1) = cj2; coefficients(1+k*3,2) = ca2;
					coefficients(1+k*3,3) = cv2; coefficients(1+k*3,4) = cp2;
					
					coefficients(2+k*3,0) = dT3; coefficients(2+k*3,1) = cj3; coefficients(2+k*3,2) = ca3;
					coefficients(2+k*3,3) = cv3; coefficients(2+k*3,4) = cp3;
					
				}
				segments_nr = nofPoints * 3.0;
				return true;
			}
			
			virtual bool calculateCoefficients_fromJerk() {
				for (int k = 0; k < nofPoints; k++) {
					T cj1, ca1, cv1, cp1;
					if(k == 0){
						cj1 = coefficients(k,1);
						ca1 = last[2]; //[0];
						cv1 = last[1]; //[0];
						cp1 = last[0]; //[0];
					}
					else{
						T t_old = coefficients(k-1,0);
						T j_old = coefficients(k-1,1);
						T a_old = coefficients(k-1,2);
						T v_old = coefficients(k-1,3);
						T p_old = coefficients(k-1,4);
						
						cj1 = coefficients(k,1);
						
						for(int i=0; i<ca1.size(); i++){
							ca1(i) = a_old(i) + j_old(i) * t_old(i);
							cv1(i) = v_old(i) + a_old(i) * t_old(i) + j_old(i) / 2.0 * t_old(i) * t_old(i);
							cp1(i) = p_old(i) + v_old(i) * t_old(i) + a_old(i) / 2.0 * t_old(i) * t_old(i) + j_old(i) / 6.0 * t_old(i) * t_old(i) * t_old(i);
						}
					}
					coefficients(k,2) = ca1;
					coefficients(k,3) = cv1; 
					coefficients(k,4) = cp1; 
				}
				segments_nr = nofPoints; 
				return true;
			}
			
			eeros::control::Output<T> posOut;
			eeros::control::Output<T> velOut;
			eeros::control::Output<T> accOut;
			eeros::control::Output<T> jerkOut;
			
			std::mutex mtx;
			double dt;	// period of the block running
			double t;	// time integrating up with the path planner
			double dTold;
			int k = 0;
			int nofPoints;	// nof of points of a given path
			int segments_nr;
		
			std::atomic<bool> finished;
			bool finish_segment = true;
			bool segment_set = false;
			bool jerkLimit = false;
			
			T velMax, accMax, decMax; 
			std::vector<T> last = std::vector<T>(4);
			std::vector<T> positions = std::vector<T>(100);
			eeros::math::Matrix<100,5,T> coefficients;
			eeros::logger::Logger log;
		};
	};
};

#endif /* ORG_EEROS_CONTROL_PATHPLANNER_TRAPEZOID_HPP_ */
