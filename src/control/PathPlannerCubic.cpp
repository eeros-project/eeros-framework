#include <eeros/control/PathPlannerCubic.hpp>
#include <eeros/core/System.hpp>
#include <iostream>
#include <fstream>
#include <unistd.h>

using namespace eeros;
using namespace eeros::control;

PathPlannerCubic::PathPlannerCubic(double dt) : dt(dt), timeInterval(-dt), log('P'), posOut(this), velOut(this), accOut(this), jerkOut(this) {
	prevPos = 0.0;
	prevVel = 0.0;
	prevAcc = 0.0;
	posOut.getSignal().clear();
	velOut.getSignal().clear();
	accOut.getSignal().clear();
	jerkOut.getSignal().clear();
}

Output<double>& PathPlannerCubic::getPosOut() {
	return posOut;
}

Output<double>& PathPlannerCubic::getVelOut() {
	return velOut;
}

Output<double>& PathPlannerCubic::getAccOut() {
	return accOut;
}

Output<double>& PathPlannerCubic::getJerkOut() {
	return jerkOut;
}

void PathPlannerCubic::clear() {
	timeCoeffRaw.clear();
	jerkCoeffRaw.clear();
	accCoeffRaw.clear();
	velCoeffRaw.clear();
	posCoeffRaw.clear();
	timeCoeff.clear();
	jerkCoeff.clear();
	accCoeff.clear();
	velCoeff.clear();
	posCoeff.clear();
	posCoeffShifted.clear();
}

void PathPlannerCubic::reset() {
	this->finished = true;
	clear();
}

void PathPlannerCubic::init(std::string filename) {
	std::ifstream file;       
	
	log.info() << "filename " << filename; 
	file.open(filename.c_str());   
	if (!file.is_open()) throw Fault("File for loading trajectory is not open!");
	clear();
	while(!file.eof()) {
		std::string tmp_string;
		double tmp_double;
		
		// time
		file >> tmp_string;
		tmp_double = ::atof(tmp_string.c_str());
		if (tmp_string.empty()) break;
		timeCoeffRaw.push_back(tmp_double);
		
		// jerk
		file >> tmp_string;
		tmp_double = ::atof(tmp_string.c_str());
		jerkCoeffRaw.push_back(tmp_double);
		
		// acc
		file >> tmp_string;
		tmp_double = ::atof(tmp_string.c_str());
		accCoeffRaw.push_back(tmp_double);
		
		// vel
		file >> tmp_string;
		tmp_double = ::atof(tmp_string.c_str());
		velCoeffRaw.push_back(tmp_double);
		
		// pos
		file >> tmp_string;
		tmp_double = ::atof(tmp_string.c_str());
		posCoeffRaw.push_back(tmp_double);
	}
// 	std::cout << "raw values from file" << std::endl;
// 	std::cout << "index; time diff; time accumulated; jerk; acc; vel; pos" << std::endl;
// 	double sum = 0;
// 	for (int i = 0; i < timeCoeffRaw.size(); i++) {
// 		sum += timeCoeffRaw[i];
// 		std::cout << i << "\t" << timeCoeffRaw[i] << "\t" << sum << "\t" << jerkCoeffRaw[i] << "\t" << accCoeffRaw[i] << "\t" << velCoeffRaw[i] << "\t" << posCoeffRaw[i] << std::endl;
// 	}
}

void PathPlannerCubic::scalePath(double time, double deltaPos) {
	std::vector<double> time_rounded, jerk_rounded;
	
	// Get total time of curve
	double tabSize = timeCoeffRaw.size();
	double timeTotal = 0.0;
	for (int i = 0; i < tabSize; i++) timeTotal = timeTotal + timeCoeffRaw[i];
	
	// Scale time vector and round to multiple of sampling time
	double timeScale = time / timeTotal;
	for (int i = 0; i < tabSize; i++) 
		time_rounded.push_back(round(timeCoeffRaw[i] * timeScale / dt) * dt);
	
	// Get total time of rounded time spans
	double timeTotalRounded = 0.0;
	for (int i = 0; i < tabSize; i++) timeTotalRounded = timeTotalRounded + time_rounded[i];
	timeScale = timeTotalRounded / timeTotal;
	log.info() << "time scale = " << timeScale;

	// Scale jerk according to scaled time and scaled position
	double posScale  = deltaPos / posCoeffRaw[tabSize-1];
	log.fatal() << deltaPos;
	log.fatal() << posScale;
	log.fatal() << timeScale;
	for(int i = 0; i < tabSize; i++) 
		jerk_rounded.push_back(jerkCoeffRaw[i] * posScale / (timeScale*timeScale*timeScale));
		
	// Scale acc, vel and pos coefficients
	std::vector<double> acc_rounded, vel_rounded, pos_rounded;
	acc_rounded.resize(jerk_rounded.size());
	vel_rounded.resize(jerk_rounded.size());
	pos_rounded.resize(jerk_rounded.size());
	double acc_prev = 0.0; acc_rounded[0] = acc_prev;
	double vel_prev = 0.0; vel_rounded[0] = vel_prev;
	double pos_prev = 0.0; pos_rounded[0] = pos_prev;
	for (int i = 1; i < tabSize; i++) {
		double t = time_rounded[i-1];
		double j = jerk_rounded[i-1];
		acc_rounded[i] = acc_prev + j * t;
		vel_rounded[i] = vel_prev + acc_prev * t + j * t * t / 2;
		pos_rounded[i] = pos_prev + vel_prev * t + acc_prev * t * t / 2 + j * t * t * t / 6;
		acc_prev = acc_rounded[i];
		vel_prev = vel_rounded[i];
		pos_prev = pos_rounded[i];
	}
	
// 	std::cout << "rounded values" << std::endl;
// 	std::cout << "index; time diff; time accumulated; jerk; acc; vel; pos" << std::endl;
// 	double sum = 0;
// 	for (int i = 0; i < tabSize; i++) {
// 		sum += time_rounded[i];
// 		std::cout << i << "\t" << time_rounded[i] << "\t" << sum << "\t" << jerk_rounded[i] << "\t" << acc_rounded[i] << "\t" << vel_rounded[i] << "\t" << pos_rounded[i] << std::endl;
// 	}
	std::lock_guard<std::mutex> guard(m);
	timeCoeff = time_rounded;
	jerkCoeff = jerk_rounded;
	accCoeff  = acc_rounded;
	velCoeff  = vel_rounded;
	posCoeff  = pos_rounded;
}

bool PathPlannerCubic::move(double initPos){
	if (!finished) return false;
	if (timeCoeffRaw.size() <= 0) throw Fault("Time coeff array empty!"); 
	timeCoeff.resize(timeCoeffRaw.size());
	jerkCoeff.resize(timeCoeffRaw.size());
	accCoeff.resize(timeCoeffRaw.size());
	velCoeff.resize(timeCoeffRaw.size());
	posCoeff.resize(timeCoeffRaw.size());
	posCoeffShifted.resize(timeCoeffRaw.size());
	for (int i = 0; i < timeCoeffRaw.size(); i++) {
		timeCoeff[i] = timeCoeffRaw[i];
		jerkCoeff[i] = jerkCoeffRaw[i];
		accCoeff[i] = accCoeffRaw[i];
		velCoeff[i] = velCoeffRaw[i];
		posCoeff[i] = posCoeffRaw[i];
		posCoeffShifted[i] = posCoeffRaw[i] + initPos;	// set start position
	}
	finished = false;
	return true;
}

bool PathPlannerCubic::move(double time, double startPos, double deltaPos){
	if (!finished) return false;
	if (timeCoeffRaw.size() <= 0) throw Fault("Time coeff array empty!"); 
	
	scalePath(time, deltaPos); 
	
	posCoeffShifted.resize(posCoeff.size());
	for (int i = 0; i < posCoeff.size(); i++) posCoeffShifted[i] = posCoeff[i] + startPos;	// set start position
	finished = false;
	return true;
}

bool PathPlannerCubic::posReached() {
	return finished;
}

void PathPlannerCubic::setInitPos(double initPos) {
	prevJerk = 0;
	prevAcc  = 0; 
	prevVel  = 0;
	prevPos  = initPos;
	finished = true;
}

void PathPlannerCubic::run() {
	double pos, vel, acc, jerk;
// 	static int index = 0;
// 	static bool first = true;
// 	static double timeInterval = -dt;
// 	static double t = 0;
	
	std::lock_guard<std::mutex> guard(m);
	
	if(!finished && timeCoeff.size() > 0 && index < timeCoeff.size()) {
		if(t <= timeInterval + (dt / 2)) {
			jerk = jerkCoeff[index];
			acc = prevAcc + jerk * dt;
			vel = prevVel + prevAcc * dt + jerk / 2 * dt * dt;
			pos = prevPos + prevVel * dt + prevAcc / 2 * dt * dt + jerk / 6 * dt * dt * dt;
		} else {
			if (first) {index == 0; timeInterval = timeCoeff[index]; first = false;}
			else {
				index++;
				if (index == timeCoeff.size() - 1) finished = true;
				timeInterval += timeCoeff[index];
			}
			log.info() << "new interval: index = " << index;
			jerk = jerkCoeff[index];
			acc  = accCoeff[index];
			vel  = velCoeff[index];
			pos  = posCoeffShifted[index];
		}
	} else {
		t = 0;
		index = 0;
		timeInterval = -dt;
		first = true;
		pos = prevPos;
		vel = prevVel;
		acc = prevAcc;
		jerk = prevJerk;
	}
	t += dt;
	prevPos = pos;
	prevVel = vel;
	prevAcc = acc;
	prevJerk = jerk;
	
	posOut.getSignal().setValue(pos);
	velOut.getSignal().setValue(vel);
	accOut.getSignal().setValue(acc);
	jerkOut.getSignal().setValue(jerk);
	
	timestamp_t time = System::getTimeNs();
	posOut.getSignal().setTimestamp( time);
	velOut.getSignal().setTimestamp( time);
	accOut.getSignal().setTimestamp( time);
	jerkOut.getSignal().setTimestamp(time);
}
