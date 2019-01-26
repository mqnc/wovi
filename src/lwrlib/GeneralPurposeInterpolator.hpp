
// GeneralPurposeInterpolator by Mirko Kunze and Mirko Comparetti
// designed to work under realtime conditions
// (dynamic memory allocation is only performed during construction)

// EDIT 18.10.2013 by Mirko Kunze: forgot return true in copyElements
// EDIT 12.12.2013 by Mirko Kunze: removed TOLERANCE
//                                 replaced X (like in xNow) by p for position


// usage:
/*
// on a realtime system the initialization of the interpolator (also of vectors)
// MUST be done during initialization:
GeneralPurposeInterpolator GPI(2); // argument = number of parameters to interpolate

std::vector<double> buffer_0(2,0); // vector with two elements, filled with zeros
std::vector<double> buffer_1(2,1); // vector with two elements, filled with ones
std::vector<double> buffer_10(2,10); // vector with two elements, filled with tens

// the followings things can also be done during runtime:
GPI.setPTarget(buffer_1); // target
GPI.setPLast(buffer_0); // start value
GPI.setVLast(buffer_0); // start velocity
GPI.setPMin(buffer_0); // minimum value to return (e.g. joint limit)
GPI.setPMax(buffer_10); // maximum value to return
GPI.setVMax(buffer_1); // velocity limit
GPI.setAMax(buffer_1); // acceleration limit
GPI.setDt(0.01); // sampling time
GPI.setMode(1); // interpolation mode:
// 0 = static target, each parameter is interpolated individually
// 1 = static target, all parameters end on their target synchronously
// 2 = trajectory following; target has to update every cycle, all parameters are braked if limits are exceeded

// this has to be executed every cycle:
GPI.interpolate(); // perform one step
GPI.getPNow(buffer_0); // read current values into buffer
GPI.getVNow(buffer_0); // read current velocities into buffer

// the functions return true if successful and false if there is a problem
// (most likely some vector dimensions do not match)
*/

#ifndef GPI_HPP
#define GPI_HPP

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

using namespace std;

class GeneralPurposeInterpolator{

public:
	// constructor:
	GeneralPurposeInterpolator(int numParameters){
		this->numVars = numParameters;
		dt=1;
		mode=0;

		// initialize vectors with 0
		this->pTarget =
		this->pLast =
		this->vLast =
		this->pNow =
		this->vNow =
		this->pMin =
		this->pMax =

		this->pTargetLim =
		this->pToGo =
		this->a1 =
		this->a2 =
		this->T1 =
		this->T2 =
		this->T3 =
		this->TSum =
		this->vInt =
			std::vector<double>(this->numVars,0.0);

		// initialize with low limits, otherwise divisions by zero can happen
		this->vMax =
		this->aMax =
			std::vector<double>(this->numVars,1e-6);
	}

	// setters, getters:
	// (they do nothing and return false if the parameters are of wrong size)
	bool setPTarget(std::vector<double> (&newPTarget) ){
		return this->copyElements(this->pTarget, newPTarget);
	}
	bool setPLast(std::vector<double> (&newPLast) ){
		return this->copyElements(this->pLast, newPLast);
	}
	bool setVLast(std::vector<double> (&newVLast) ){
		return this->copyElements(this->vLast, newVLast);
	}
	bool setPMin(std::vector<double> (&newPMin) ){
		return this->copyElements(this->pMin, newPMin);
	}
	bool setPMax(std::vector<double> (&newPMax) ){
		return this->copyElements(this->pMax, newPMax);
	}
	bool setVMax(std::vector<double> (&newVMax) ){
		return this->copyElements(this->vMax, newVMax);
	}
	bool setAMax(std::vector<double> (&newAMax) ){
		return this->copyElements(this->aMax, newAMax);
	}
	bool setDt(double newDt){
		this->dt = newDt;
	}
	bool setMode(int newMode){
		this->mode = newMode;
	}

	bool getPNow(std::vector<double> (&outputBuffer) ){
		return this->copyElements(outputBuffer, this->pNow);
	}
	bool getVNow(std::vector<double> (&outputBuffer) ){
		return this->copyElements(outputBuffer, this->vNow);
	}

	// core function:
	bool interpolate();

private:

	// interpolation parameters:
	int numVars;

	std::vector<double> pTarget;
	std::vector<double> pLast;
	std::vector<double> vLast;
	std::vector<double> pNow;
	std::vector<double> vNow;
	std::vector<double> pMin;
	std::vector<double> pMax;
	std::vector<double> vMax;
	std::vector<double> aMax;
	double dt;
	int mode;

	// auxiliary variables:
	std::vector<double> pTargetLim;
	std::vector<double> pToGo;
	std::vector<double> a1;
	std::vector<double> a2;
	std::vector<double> T1;
	std::vector<double> T2;
	std::vector<double> T3;
	std::vector<double> TSum;
	std::vector<double> vInt;

	// copy without memory reallocation:
	static bool copyElements(std::vector<double> (&dst), std::vector<double> (&src)){
		if(dst.size() != src.size()){return false;}
		for(unsigned int i=0; i<dst.size(); i++){
			dst[i] = src[i];
		}
		return true;
	}
};

#endif
