#include "PathPlanner.hpp"
#include <eeros/core/System.hpp>
#include <iostream>
#include <fstream>

using namespace pathos::peepingpanel;
using namespace eeros;
using namespace eeros::control;

PathPlanner::PathPlanner(AxisVector velMax, AxisVector accMax, AxisVector decMax, double dt) : 
                         velMax(velMax), accMax(accMax), decMax(decMax), dt(dt) {
	coefficients.zero();
	coefficients.transpose();
	finish = true;
}

Output<AxisVector>& PathPlanner::getPosOut() {
	return posOut;
}

Output<AxisVector>& PathPlanner::getVelOut() {
	return velOut;
}

Output<AxisVector>& PathPlanner::getAccOut() {
	return accOut;
}

Output<AxisVector>& PathPlanner::getJerkOut() {
	return jerkOut;
}

void PathPlanner::reset() {
	this->finish = true;
	
	coefficients.zero();
	segment_set = false;
}

bool PathPlanner::move(AxisVector p, bool limitOn) {
	if(!finish)
		return false;
	coefficients.zero();
	
	positions[0] = p;
	jerk_limit_on = limitOn;
	
	this->points_nr = 1.0;
	
	if(calculateCoefficients_fromPosition()){
		finish = false;
		finish_segment = false;
		t = 0;
		dTold = 0;
		k = 0;
	}
	else {
		finish = true;
		finish_segment = true;
		t = 0;
		dTold = 0;
		k = 0;
	}	
	return true;
}

bool PathPlanner::move(std::array<AxisVector, 100> array, bool limitOn) {
	if(!finish)
		return false;
	coefficients.zero();

	positions = array;
	jerk_limit_on = limitOn;
	
	// find end of array
	AxisVector zero; zero.zero();
	int nr_in_points = 0;
	for(int i = 0; i < array.size(); i++){
		if(array[i] < zero)
			i = array.size();
		else
			nr_in_points++;
	}
	
	// Safety check, if -1 at end of array missing, don't calculate trajectory
	if(nr_in_points == array.size())
		return false;
	
	this->points_nr = nr_in_points;
		
	if(calculateCoefficients_fromPosition()){
		finish = false;
		finish_segment = false;
		t = 0;
		dTold = 0;
		k = 0;
	}
	else {
		finish = true;
		finish_segment = true;
		t = 0;
		dTold = 0;
		k = 0;
	}	
	return true;
}

bool PathPlanner::move(std::string filename, double time_tot, AxisVector end_position){
	if(!finish)
		return false;
	coefficients.zero();
	
	jerk_limit_on = false;
	
	std::fstream file;
	file.open(filename, std::fstream::in);
	if(!file.is_open()) throw EEROSException("File for loading trajectory!");

	std::array<double, 100> input_time; 
	std::array<double, 100> input_jerk; 
	
	// set array dimension, time and jerk values
	int nr_in_points = 0;
	for(int j = 0; j < input_time.size(); j++){
		file >> input_time[j]; 
		file >> input_jerk[j];
		if(input_time[j] < 0.0)
			j = input_time.size();
		else
			nr_in_points++;
	}
	file.close();
	this->points_nr = nr_in_points;
	
	// Norm and scale curve
	for(int j = 0; j < points_nr; j++){
		input_time[j] = input_time[j] * time_tot;
		input_jerk[j] = input_jerk[j] * end_position / (time_tot*time_tot*time_tot);
	}
	
	double rounded_points_nr = 0;
	std::array<double, 100> rounded_input_time; 
	std::array<double, 100> rounded_input_jerk; 
	std::array<double, 100> dT;
	std::array<double, 100> rest;
	std::array<int   , 100> rest2;
	
	double timePrev = 0.0;
	for(int j = 0; j < points_nr; j++){
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
			if(j < points_nr -1) {
				rounded_input_time[rounded_points_nr+1] = dt;
				rounded_input_jerk[rounded_points_nr+1] = (input_jerk[j] * t1 + input_jerk[j+1] * t2)/ dt;
			}
			
			// update indexes
			timePrev = dT[j] + t2; 
			if(j < points_nr -1) rounded_points_nr = rounded_points_nr + 2;
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
	points_nr = rounded_points_nr;
	
	// final settings
	for(int j = 0; j < points_nr; j++){
		input_time[j] = rounded_input_time[j];
		input_jerk[j] = rounded_input_jerk[j];
	}
	
	// save time and jerk into coefficients
	int j = 0;
	while (j < points_nr) {
		coefficients(j,0) = input_time[j]; // dT
		coefficients(j,1) = input_jerk[j]; // cj1
		j++;
	}
	
	// Calculate coefficients like matlab program
	calculateCoefficients_fromJerk();
	
	finish = false;
	finish_segment = false;
	t = 0;
	dTold = 0;
	k = 0;
	
	return true;
}

bool PathPlanner::calculateCoefficients_fromPosition() {
	for (int k = 0; k < points_nr; k++) {
		std::array<AxisVector, 4> start;
		if(k == 0) {
			start = last; 
		}
		else{
			start = {positions[k-1](0), 0, 0, 0};
		}
		std::array<AxisVector, 4> end = {positions[k](0), 0, 0, 0};

		AxisVector calcVelNorm, calcAccNorm, calcDecNorm;
		AxisVector velNorm, accNorm, decNorm, squareNormVel;
		AxisVector distance = end[0] - start[0];
				
		AxisVector zero;
		zero = 0;
		if (distance == zero){
			return false;
		}
		
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
		
		cj1 = 0.0;  
		cj2 = 0.0;  
		cj3 = 0.0;
		
		ca1 = velNorm * dt / dT1 * distance; 
		ca2 = 0.0; 
		ca3 = velNorm * dt / dT3 * distance * (-1); 
		
		cv1 = 0.0;  
		cv2 = ca1 * dT1;  
		cv3 = ca1 * dT1 + ca2 * dT2;  
		
		if(k == 0)
			cp1 = last[0][0];
		else
			cp1 = positions[k-1](0);
		cp2 = cp1 + cv1 * dT1 + ca1 * dT1 * dT1 / 2.0; 
		cp3 = cp2 + cv2 * dT2 + ca2 * dT2 * dT2 / 2.0;

		coefficients(0+k*3,0) = dT1; coefficients(0+k*3,1) = cj1; coefficients(0+k*3,2) = ca1;
		coefficients(0+k*3,3) = cv1; coefficients(0+k*3,4) = cp1; 
		
		coefficients(1+k*3,0) = dT2; coefficients(1+k*3,1) = cj2; coefficients(1+k*3,2) = ca2;
		coefficients(1+k*3,3) = cv2; coefficients(1+k*3,4) = cp2;
		
		coefficients(2+k*3,0) = dT3; coefficients(2+k*3,1) = cj3; coefficients(2+k*3,2) = ca3;
		coefficients(2+k*3,3) = cv3; coefficients(2+k*3,4) = cp3;
	}
	segments_nr = points_nr * 3.0;

	return true;
}

bool PathPlanner::calculateCoefficients_fromJerk() {
	for (int k = 0; k < points_nr; k++) {
		if(k == 0){
			cj1 = coefficients(k,1);
			ca1 = last[2][0];
			cv1 = last[1][0];
			cp1 = last[0][0];
		}
		else{
			AxisVector t_old = coefficients(k-1,0);
			AxisVector j_old = coefficients(k-1,1);
			AxisVector a_old = coefficients(k-1,2);
			AxisVector v_old = coefficients(k-1,3);
			AxisVector p_old = coefficients(k-1,4);
			
			cj1 = coefficients(k,1);
			ca1 = a_old + j_old * t_old;
			cv1 = v_old + a_old * t_old + j_old / 2.0 * t_old * t_old;
			cp1 = p_old + v_old * t_old + a_old / 2.0 * t_old * t_old + j_old / 6.0 * t_old * t_old * t_old;
		}
		coefficients(k,2) = ca1;
		coefficients(k,3) = cv1; 
		coefficients(k,4) = cp1; 
	}
	segments_nr = points_nr; 
	
	return true;
}

bool PathPlanner::posReached() {
	return finish;
}

void PathPlanner::setInitPos(AxisVector initPos) {
	AxisVector z; z.zero();
	std::array<AxisVector, 4> r;
	r[0] = initPos;
	r[1] = z;
	r[2] = z;
	r[3] = z;

	this->last = r;
	this->finish = true;
}

void PathPlanner::run() {
	// get()
	std::array<AxisVector, 4> y = this->last;
	timestamp_t time = System::getTimeNs();	
	t += dt;
	
	for(unsigned int i = 0; i < velMax.size() ; i++) {
		if(!finish) {
			if (k < segments_nr) {
				// define coefficients
				if(segment_set == false) {
					dT     = coefficients(k,0);
					j_prev = coefficients(k,1);
					a_prev = coefficients(k,2);
					v_prev = coefficients(k,3);
					p_prev = coefficients(k,4);
					j0 =   6.0 * coefficients(k,2) / coefficients(k,0);
					s0 = -12.0 * coefficients(k,2) / (coefficients(k,0)*coefficients(k,0));
					segment_set = true;
				}
				
				// define trajectory
				if(t >= dTold && t < dT + dTold) { }
				else {
					finish_segment = true;
					segment_set = false;
					dTold = dTold + dT;
					k++;
				}
				
				y[0][i] = p_prev + v_prev * dt + (a_prev/2.0) * pow(dt,2) + (j_prev/6.0) * pow(dt,3);
				y[1][i] = v_prev + a_prev * dt + (j_prev/2.0) * pow(dt,2);
// 				y[2][i] = a_prev + j_prev * dt;
				
				if(jerk_limit_on) {
					y[2][i] = (s0 / 2.0) * (t-dTold) * (t-dTold) + j0 * (t-dTold);
				}
				else {
					y[2][i] = a_prev + j_prev * dt;
				}
				
				y[3][i] = j_prev;

				p_prev = y[0][i];
				v_prev = y[1][i];
				a_prev = y[2][i];
				j_prev = y[3][i];
			}
			if(finish_segment == true && k == segments_nr){
				finish = true;
			}
		}
		// set last value
		this->last[0][i] = y[0][i];
		this->last[1][i] = y[1][i];
		this->last[2][i] = y[2][i];
		this->last[3][i] = y[3][i];
	}	

	posOut.getSignal().setValue( y[0]);
	velOut.getSignal().setValue( y[1]);
	accOut.getSignal().setValue( y[2]);
	jerkOut.getSignal().setValue(y[3]);

	posOut.getSignal().setTimestamp(time);
	velOut.getSignal().setTimestamp(time);
	accOut.getSignal().setTimestamp(time);
	jerkOut.getSignal().setTimestamp(time);
}


void PathPlanner::setMaxSpeed(AxisVector speed) {
	this->velMax = speed;
}

void PathPlanner::setMaxAcc(AxisVector acc) {
	this->accMax = acc;
}

void PathPlanner::setMaxDec(AxisVector dec) {
	this->decMax = dec;
}