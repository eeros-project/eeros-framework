#ifndef ORG_EEROS_CONTROL_CONSTANTACCTRAJECTORYGENERATOR_HPP_
#define ORG_EEROS_CONTROL_CONSTANTACCTRAJECTORYGENERATOR_HPP_

#include <cmath>
#include <mutex>
#include <eeros/core/Fault.hpp>
#include <eeros/control/TrajectoryGenerator.hpp>


namespace eeros {
	namespace control {
		
		template<typename T>
		class ConstantAccTrajectoryGenerator : public TrajectoryGenerator<T, 3> {
			
		public:
			
			using E = typename T::value_type;
			
			ConstantAccTrajectoryGenerator(T velMax, T accMax, T decMax, double dt) : finish(true), velMax(velMax), accMax(accMax), decMax(decMax), dt(dt) { }
			
			virtual bool finished() {
				std::lock_guard<std::mutex> lck(mtx);
				return finish;
			}
			
			virtual std::array<T, 3> get(double dt) {
				std::lock_guard<std::mutex> lck(mtx);
				std::array<T, 3> y = this->last;
				t += dt;
				
				for(unsigned int i = 0; i < a1p.size(); i++) {
					if(!finish) {
						if(t >= 0 && t < dT1) {
							y[0][i] = a1p[i] * pow(t, 2) + c1p[i];
							y[1][i] = b1v[i] * t;
							y[2][i] = c1a[i];
						}
						else if(t >= dT1 && t < dT1 + dT2) {
							y[0][i] = b2p[i] * t + c2p[i];
							y[1][i] = c2v[i];
							y[2][i] = 0;
						}
						else if(t >= dT1 + dT2 && t < dT1 + dT2 + dT3) {
							y[0][i] = a3p[i] * pow(t, 2) + b3p[i] * t + c3p[i];
							y[1][i] = b3v[i] * t + c3v[i];
							y[2][i] = c3a[i];
						}
						else if(t >= dT1 + dT2 + dT3) {
							finish = true;
							y[0][i] = this->last[0][i];
							y[1][i] = 0.0;
							y[2][i] = 0.0;
						}
						else { // t < 0
							throw Fault("get() failed, t < 0");
						}
					}
					// set last value
					this->last[0][i] = y[0][i];
					this->last[1][i] = y[1][i];
					this->last[2][i] = y[2][i];
				}
				
				return y;
			}
			
			virtual bool push(std::array<T, 3> start, std::array<T, 3> end) {
				if(!finish) return false;
				T calcVelNorm, calcAccNorm, calcDecNorm;
				E velNorm, accNorm, decNorm, squareNormVel;
				T distance = end[0] - start[0];
				
				T zero;
				zero = 0;
				if (distance == zero)
					return false;
				
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
				dT1 = velNorm / accNorm;
				dT3 = velNorm / decNorm;
				dT2 = 1 / velNorm - (dT1 + dT3) * 0.5;
				if (dT2 < 0) dT2 = 0;
				
				// Adaptation to timestamps
				dT1 = ceil(dT1 / dt) * dt;
				dT2 = ceil(dT2 / dt) * dt;
				dT3 = ceil(dT3 / dt) * dt; 
				// Adaptation of speed to new timestamps
				velNorm = 1/((dT2 + (dT1 + dT3)*0.5)*dt);
				
				a1p = 0.5 * velNorm / dT1 * distance * dt;
				c1p = start[0];
				b1v = velNorm / dT1 * dt * distance * dt;
				c1a = velNorm * dt / dT1 * distance * pow(dt, 2);
				
				b2p = velNorm * dt * distance;
				c2p = start[0] - 0.5 * velNorm * dT1 * dt * distance;
				c2v = velNorm * distance * dt * dt;
				c2a = 0; 
				
				a3p = (-1) * velNorm * 0.5 * dt / dT3 * distance;
				b3p = velNorm * dt / dT3 * (dT1 + dT2 + dT3) * distance;
				c3p = start[0] + (1 - velNorm * 0.5 * dt / dT3 * pow(dT1 + dT2 + dT3, 2)) * distance;
				b3v = (-1) *velNorm * dt * distance / dT3 * dt;  
				c3v = velNorm * (dT1 + dT2 + dT3) * distance * dt / dT3 * dt; 
				c3a = (-1) * velNorm * dt / dT3 * distance * dt * dt;
				
				std::lock_guard<std::mutex> lck(mtx);
				finish = false;
				t = 0;
				
				return true;
			}
			
			virtual void reset(std::array<T, 3> last) {
				std::lock_guard<std::mutex> lck(mtx);
				this->last = last;
				this->finish = true;
			}
			
			virtual void setMaxSpeed(T speed) {
				velMax = speed;
			}
			
			virtual void setMaxAcc(T acc) {
				accMax = acc;
			}
			virtual void setMaxDec(T dec) {
				decMax = dec;
			}
			
		protected:
			std::mutex mtx;
			double dt;
			T velMax, accMax, decMax;
			
			double dT1, dT2, dT3;
			T a1p, c1p, b1v, c1a, b2p, c2p, c2v, c2a, a3p, b3p, c3p, b3v, c3v, c3a; 
			
			double t;
			bool finish;
		};
	};
};

#endif /* ORG_EEROS_CONTROL_CONSTANTACCTRAJECTORYGENERATOR_HPP_ */

