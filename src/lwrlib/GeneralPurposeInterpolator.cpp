
#include "GeneralPurposeInterpolator.hpp"

#define SIGN(x) (((x) < 0) ? -1 : ((x) > 0))
#define ABS(x) (((x) < 0) ? (-(x)) : (x))
#define SQUARE(x) ((x)*(x))

bool GeneralPurposeInterpolator::interpolate(){

	int n=this->numVars;

	double pJolt = 0.0;
	double tToVMax = 0.0;
	double radic = 0.0;
	double tBrake = 0.0;
	bool flip = false;
	double pAfterT1 = 0.0;
	double pAfterT2 = 0.0;
	double pBrakeEnd = 0.0;
	double TMax = 0.0;
	bool brakeAll = false;
	double a = 0.0;
	double v = 0.0;
	double vNowMinNoOS = 0.0;
	double vNowMaxNoOS = 0.0;

	for (unsigned int i = 0; i < n; i++) {
		pTargetLim[i] = pTarget[i];
		if(pTargetLim[i] > pMax[i]){pTargetLim[i] = pMax[i];}
		if(pTargetLim[i] < pMin[i]){pTargetLim[i] = pMin[i];}
		pToGo[i] = pTargetLim[i]-pLast[i];
	}

	switch (mode) {
		case 0:
			// (no break here!)
		case 1: // or 0
			for (unsigned int i = 0; i < n; i++) {
				// where would I end up if I decelerated to v = 0?
				tBrake = ABS(vLast[i]) / aMax[i];
				pBrakeEnd = vLast[i] / 2.0 * tBrake;	// seen from current position

				// find maximum of needed velocity (positive or negative)
				if (pBrakeEnd <= pToGo[i]) {	// more positive velocity (vInt>vLast, vInt>0)

					// where would I end up when I accelerated up to + vmax and then braked? ( = jolt)
					tToVMax = ABS(vMax[i] - vLast[i]) / aMax[i];
					tBrake = vMax[i] / aMax[i];
					pJolt = (vLast[i] + vMax[i]) / 2.0 * tToVMax + vMax[i] * tBrake / 2.0;

					a1[i] = aMax[i];
					a2[i] = -aMax[i];

					if (pJolt <= pToGo[i]) {	// not enough, stay at vMax for some time

						vInt[i] = vMax[i];
						T1[i] = tToVMax;
						T2[i] = (pToGo[i] - pJolt) / vMax[i];
						T3[i] = tBrake;

					}
					else {	// that would be too far, find the appropriate speed maximum

						radic = pToGo[i] * aMax[i] + SQUARE(vLast[i]) / 2.0;
						if (radic < 0.0) radic = 0.0;	// prevent numerical errors

						vInt[i] = sqrt(radic);
						T1[i] = (vInt[i] - vLast[i]) / aMax[i];
						T2[i] = 0.0;
						T3[i] = vInt[i] / aMax[i];

					}
				}
				else {	// more negative velocity (vInt<vLast, vInt<0)
						// wh-ere would I end up when I accelerated up to - vmax and then braked? ( = jolt)
					tToVMax = ABS( -vMax[i] - vLast[i]) / aMax[i];
					tBrake = vMax[i] / aMax[i];
					pJolt = (vLast[i] - vMax[i]) / 2.0 * tToVMax - vMax[i] * tBrake / 2.0;

					a1[i] = -aMax[i];
					a2[i] = aMax[i];
					if (pJolt >= pToGo[i]) {	// not enough, stay at - vMax for some time

						vInt[i] = -vMax[i];
						T1[i] = tToVMax;
						T2[i] = (pToGo[i] - pJolt) / - vMax[i];
						T3[i] = tBrake;

					}
					else {	// that would be too far, find the appropriate speed maximum

						radic = -pToGo[i] * aMax[i] + SQUARE(vLast[i]) / 2.0;
						if (radic < 0.0) radic = 0.0;	// prevent numerical errors

						vInt[i] = -sqrt(radic);
						T1[i] = (-vInt[i] + vLast[i]) / aMax[i];
						T2[i] = 0.0;
						T3[i] = -vInt[i] / aMax[i];

					}
				}
				// the case that vInt lies between 0 and vLast can not happen, nothing would happen there
				// (e.g. a = -amax until v = vInt and then further a = -amax until v = 0)

				TSum[i] = T1[i] + T2[i] + T3[i];
			}

			if (mode == 1) {
				// now find the joint that will take the longest
				TMax = *(std::max_element(TSum.begin(), TSum.end()));
				//int iTMax = std::distance(TSum.begin(), std::find(TSum.begin(), TSum.end(), TMax));

				// now adjust all the other trajectories
				for (unsigned int i = 0; i < n; i++) {
					if (TSum[i] >= (TMax - dt / 100.0)) continue;	// no need to replan, will end almost synchronously

					flip = false;
					// now to find the new vInt, solve:
					// pToGo[i] = Tmax * vInt - ( (vInt - vLast)|vInt - vLast| + vInt|vInt| ) / 2aMax
					pToGo[i] = pTargetLim[i] - pLast[i];

					if (vInt[i] < 0.0) {	// vInt will not change its sign, always having positive vInt will simplify further calculations by reducing branches

						flip = true;
						vInt[i] = -vInt[i];
						pToGo[i] = -pToGo[i];
						vLast[i] = -vLast[i];

					}

					if ((vLast[i] < 0.0) || ((vLast[i] * TMax - SQUARE(vLast[i]) / 2.0 / aMax[i]) < pToGo[i])) {	// vInt will be higher than vLast and 0
						radic = SQUARE(TMax) * SQUARE(aMax[i]) + 2.0 * TMax * aMax[i] * vLast[i] - SQUARE(vLast[i]) - 4.0 * pToGo[i] * aMax[i];
						if (radic < 0.0) radic = 0.0;
						//vInt[i] = 1 / 2 * TMax * aMax[i] + 1 / 2 * vLast[i] - 1 / 2 * sqrt(radic);
						vInt[i] = 0.5 * (TMax * aMax[i] + vLast[i] - sqrt(radic));
					}
					else {	// vInt will lie between 0 and vLast
						double nom = (pToGo[i] * aMax[i] - SQUARE(vLast[i]) / 2.0);
						double den = (TMax * aMax[i] - vLast[i]);
						//if ((nom < numeric_limits<double>::min()) && (den < numeric_limits<double>::min()))	// we will stay 0 seconds on vInt, so this is undefined
						if ((nom < dt / 100.0) && (den < dt / 100.0))	// we will stay 0 seconds on vInt, so this is undefined
							vInt[i] = vLast[i];
						else
							vInt[i] = nom / den;
					}

					T1[i] = ABS(vInt[i] - vLast[i]) / aMax[i];
					T3[i] = vInt[i] / aMax[i];
					T2[i] = TMax - T1[i] - T3[i];

					a1[i] = SIGN(vInt[i] - vLast[i]) * aMax[i];
					a2[i] = -aMax[i];

					if (flip) {
						vInt[i] = -vInt[i];
						vLast[i] = -vLast[i];
						a1[i] = -a1[i];
						a2[i] = -a2[i];
					}
				}
			}

			// now drive
			pAfterT1 = 0.0;
			pAfterT2 = 0.0;
			for (unsigned int i = 0; i < n; i++) {
				// now let's see where I am on the desired trajectory...
				if (dt <= T1[i]) {
					vNow[i] = vLast[i] + a1[i] * dt;
					pNow[i] = pLast[i] + (vLast[i] + vNow[i]) / 2.0 * dt;
				}
				else {
					pAfterT1 = pLast[i] + (vInt[i] + vLast[i]) / 2.0 * T1[i];

					if (dt <= T1[i] + T2[i]) {
						vNow[i] = vInt[i];
						pNow[i] = pAfterT1 + (dt - T1[i]) * vInt[i];
					}
					else {
						pAfterT2 = pAfterT1 + vInt[i] * T2[i];

						if (dt <= T1[i] + T2[i] + T3[i]) {
							vNow[i] = vInt[i] + a2[i] * (dt - T1[i] - T2[i]);
							pNow[i] = pAfterT2 + (vInt[i] + vNow[i]) / 2.0 * (dt - T1[i] - T2[i]);
						}
						else {
							vNow[i] = 0.0;
							pNow[i] = pTargetLim[i];
						}
					}
				}
			}

			break;
		case 2:

			//vNoOS=sqrt(2*aMax*pToGo);
			brakeAll = false;
			for (unsigned int i = 0; i < n; i++) {

				// acceleration necessary to get from the last status (p and v) to the desired p,
				// assumed that the acceleration would be constant during the whole dt
				// (which could lead to premature braking during the next checks):
				a = (pTargetLim[i] - pLast[i] - vLast[i] * dt) * 2.0 / SQUARE(dt);

				if ((a > aMax[i]) || (a < (-aMax[i]))) {
					brakeAll = true;
					break;
				}

				// velocity that would be reached:
				v = vLast[i] + a * dt;

				if (v > vMax[i] || v < -vMax[i]) {
					brakeAll = true;
					break;
				}

				// maximum negative and positive velocity here so that no overshoot happens at pMin or pMax
				vNowMinNoOS = -sqrt(2.0 * aMax[i] * ABS(pMin[i] - pTargetLim[i]));
				vNowMaxNoOS = sqrt(2.0 * aMax[i] * ABS(pMax[i] - pTargetLim[i]));

				if (v > vNowMaxNoOS || v < vNowMinNoOS) {
					brakeAll = true;
					break;
				}

				// looks like everything is ok
				pNow[i] = pTargetLim[i];
				vNow[i] = v;
			}

			if (brakeAll) {
				for (unsigned int i = 0; i < n; i++) {
					// can we brake down to v=0 within dt?
					if (ABS(vLast[i]) <= aMax[i] * dt) {
						vNow[i] = 0.0;
						tBrake = vLast[i] / aMax[i];
						pNow[i] = pLast[i] + vLast[i] / 2.0 * tBrake;
					}
					else {
						vNow[i] = vLast[i] -SIGN(vLast[i]) * aMax[i] * dt;
						pNow[i] = pLast[i] + (vLast[i] + vNow[i]) / 2.0 * dt;
					}
				}
			}

			break;
		default:
			return false;
	}

	this->copyElements(pLast,pNow);
	this->copyElements(vLast,vNow);

	return true;
}

