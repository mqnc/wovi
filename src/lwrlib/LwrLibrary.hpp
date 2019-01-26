#ifndef LWR_LIBRARY_HPP
#define LWR_LIBRARY_HPP

// this should be removed from here and put into an external #define switches file
//#define FRI_CONNECTION

#include "LwrLibraryClasses.hpp"
#include "GeneralPurposeInterpolator.hpp"

#ifdef FRI_CONNECTION
#include "fri/friremote.h"
#endif

class Lwr{

protected:

	#ifdef FRI_CONNECTION
		friRemote* hfri;
	#endif

	double robotTimeMsr; // timestamp from robot
	double dtMsr; // duration between the last two measurements
	double dtSet; // sample time setting

	// joint space:

    LwrJoints jointPosMsr; // measured position
    LwrJoints jointPosSnt; // last sent position
    LwrJoints jointPosNxt; // position for next step
    LwrJoints jointPosTrg; // current target position

    LwrJoints jointVelMsr; // measured velocity (measured position difference/dt)
    LwrJoints jointVelLst; // jointVelNxt from last step
    LwrJoints jointVelNxt; // joint velocity for next step
	LwrJoints jointVelMax; // maximum joint velocity for interpolator

	LwrJoints jointAccMax; // maximum joint acceleration for interpolator

    LwrJoints jointTrqMsr; // measured joint torque
    LwrJoints jointTrqExt; // estimated external joint torque (gravity and robot dynamics compensated)

    // extended cartesian space:

    LwrXCart xCartPosMsr; // measured position
    LwrXCart xCartPosSnt; // last sent position
    LwrXCart xCartPosNxt; // position for next step
    LwrXCart xCartPosTrg; // current target position

    LwrXTwist xCartVelMsr; // measured velocity (measured position difference/dt)
    LwrXTwist xCartVelLst; // xCartVelVelNxt from last step
    LwrXTwist xCartVelNxt; // (xCartVelPosNxt-xCartVelPosSnt)/dt

    LwrXTwist xCartFTMsr;  // measured joint torque
    LwrXTwist xCartFTExt;  // estimated external joint torque (gravity and robot dynamics compensated)

    // jacobians

    LwrXJacobian jacobianMsr; // jacobian in base coordinate system based on measured joints
    LwrXJacobian jacobianSnt; // jacobian in base coordinate system based on sent joints

	// interpolator

	GeneralPurposeInterpolator jointInterpolator;

	// helper functions

	static LwrFrame denavitHartenberg(double d, double theta, double r, double alpha); // compute a DH matrix from DH parameters
	static void quicksortIntervals(LwrElbowInterval arr[], int left, int right);
	static void mergeIntervals(LwrElbowInterval dst[], const LwrElbowInterval src[], const int n);

	static LwrErrorMsg elbowIntervalsPart( // compute blocked elbow intervals for half of the robot
		LwrElbowInterval blockedPerJoint[3][4],
		Vector3d target,
		double l1, double l2,
		double jointLimitsPart[3],
		bool inv1, bool inv3);

public:

	// limb lengths in meters (base <-0-> shoulder <-1-> elbow <-2-> wrist <-3-> flange)
	static const double limbs[4];
	static const LwrJoints jointLimits;

	// constructor

	Lwr();
	bool friConnect(int port=-1);
	bool friReceive(bool updateCartesian=false);
	bool friCheckCmdMode();
	bool friSend(bool useCartesian=false);
	bool friClose();

    // getters and setters:

	//! \brief Set sampling time
	void setDt(double dt) {dtSet=dt; jointInterpolator.setDt(dt);}

	//! \brief Get joint position that was measured by the robot in last step
	const LwrJoints& getJointPosMsr() const {return jointPosMsr;}
	//! \brief Get joint position that was sent to the robot in last step
	const LwrJoints& getJointPosSnt() const {return jointPosSnt;}
	//! \brief Get joint position that will be sent to the robot in next step
	const LwrJoints& getJointPosNxt() const {return jointPosNxt;}
	//! \brief Set joint position that will be sent to the robot in next step
	LwrErrorMsg setJointPosNxt(LwrJoints jointPosNxt_) {jointPosNxt = jointPosNxt_;} // TODO error treatment
	//! \brief Get joint position for current target
	const LwrJoints& getJointPosTrg() const {return jointPosTrg;}
	//! \brief Set joint position for current target
	LwrErrorMsg setJointPosTrg(LwrJoints jointPosTrg_) { // TODO error treatment
		jointPosTrg = jointPosTrg_;
		std::vector <double> buffer(7);
		jointPosTrg.getJoints(buffer);
		jointInterpolator.setPTarget(buffer);
	}

	//! \brief Get measured joint velocities (difference between last two measured positions divided by time difference)
	const LwrJoints& getJointVelMsr() const {return jointVelMsr;}
	//! \brief Get last joint velocities (generated joint velocities from the last step)
	const LwrJoints& getJointVelLst() const {return jointVelLst;}
	//! \brief Get next joint velocities (generated joint velocities for the next step)
	const LwrJoints& getJointVelNxt() const {return jointVelNxt;}
	//! \brief Set next joint velocities (generated joint velocities for the next step)
	LwrErrorMsg setJointVelNxt(LwrJoints jointVelNxt_) {jointVelNxt = jointVelNxt_;} // TODO error treatment
	//! \brief Set max joint velocities
	LwrErrorMsg setJointVelMax(LwrJoints jointVelMax_) { // TODO error treatment
		jointVelMax = jointVelMax_;
		std::vector <double> buffer(7);
		jointVelMax.getJoints(buffer);
		jointInterpolator.setVMax(buffer);
	}

	//! \brief Set max joint accelerations
	LwrErrorMsg setJointAccMax(LwrJoints jointAccMax_) { // TODO error treatment
		jointAccMax = jointAccMax_;
		std::vector <double> buffer(7);
		jointAccMax.getJoints(buffer);
		jointInterpolator.setAMax(buffer);
	}

	//! \brief Get measured joint torque from last step
	const LwrJoints& getJointTrqMsr() const {return jointTrqMsr;}
	//! \brief Get estimated external joint torque from last step
	const LwrJoints& getJointTrqExt() const {return jointTrqExt;}


	//! \brief Get extended Cartesian position, computed from measured joint angles
	const LwrXCart& getCartPosMsr() const {return xCartPosMsr;}
	//! \brief Get extended Cartesian position, computed from sent joint angles
	const LwrXCart& getCartPosSnt() const {return xCartPosSnt;}
	//! \brief Get next extended Cartesian position
	const LwrXCart& getCartPosNxt() const {return xCartPosNxt;}
	//! \brief Set next extended Cartesian position
	LwrErrorMsg setCartPosNxt(LwrXCart cartPosNxt_) {xCartPosNxt = cartPosNxt_;}
	//! \brief Get extended Cartesian target position
	const LwrXCart& getCartPosTrg() const {return xCartPosTrg;}
	//! \brief Set extended Cartesian target position
	LwrErrorMsg setCartPosTrg(LwrXCart cartPosTrg_, bool updateJointPosTrg=true){ // TODO error treatment
		xCartPosTrg = cartPosTrg_;
		if(updateJointPosTrg){
			inverseKinematics(jointPosTrg,xCartPosTrg);
		}
	}

	//! \brief Get extended Cartesian velocities (difference between last two measured positions divided by time difference)
	const LwrXTwist& getCartVelMsr() const {return xCartVelMsr;}
	//! \brief Get last extended Cartesian velocities (generated joint velocities from the last step)
	const LwrXTwist& getCartVelLst() const {return xCartVelLst;}
	//! \brief Get next extended Cartesian velocities (generated joint velocities for the next step)
	const LwrXTwist& getCartVelNxt() const {return xCartVelNxt;}
	//! \brief Set next extended Cartesian velocities
	LwrErrorMsg setCartVelNxt(LwrXTwist cartVelNxt_) {xCartVelNxt = cartVelNxt_;}

	//! \brief Get measured Cartesian force/torque from last step
	const LwrXTwist& getCartFtMsr() const {return xCartFTMsr;}
	//! \brief Get estimated external Cartesian force/torque from last step
	const LwrXTwist& getCartFtExt() const {return xCartFTExt;}

	// methods:

	//! \brief Compute Cartesian position, null-space parameter and configuration from joint angles
    static LwrErrorMsg forwardKinematics(LwrXCart& xCartPose, LwrJoints& jointAngles);
    //! \brief Compute joint angles from Cartesian position, null-space parameter and configuration
    static LwrErrorMsg inverseKinematics(LwrJoints& jointAngles, LwrXCart& xCartPose);

	//! \brief Compute 7x7 extended Jacobian in base coordinate system (7th row applies to null-space parameter)
	static LwrErrorMsg computeXJacobian(LwrXJacobian& jacobian, LwrJoints& jointAngles);
	//! \brief Transform a Jacobian into another coordinate system (7th row untouched)
	static LwrXJacobian transformXJacobian(LwrXJacobian& jacobian, Matrix3d ori);

	//! \brief Compute Cartesian twist from Jacobian and joint velocities
    static LwrErrorMsg forwardJacobian(LwrXTwist& xCartTwist, LwrXJacobian& jacobian, LwrJoints& jointVelocities);
    //! \brief Compute joint velocities from Jacobian and Cartesian twist
    static LwrErrorMsg inverseJacobian(LwrJoints& jointVelocities, LwrXJacobian& jacobian, LwrXTwist& xCartTwist);

	//! \brief Estimate Cartesian force/torque on flange from joint torque
	static LwrErrorMsg xCartFTFromJointTorque(LwrXTwist& xCartFT, LwrXJacobian& jacobian, LwrJoints& jointTrq);
	//! \brief Compute joint torque required to generate force/torque on flange
	static LwrErrorMsg jointTorqueFromXCartFT(LwrJoints& jointTrq, LwrXJacobian& jacobian, LwrXTwist& xCartFT);

	//! \brief Compute valid elbow poses for a given Cartesian target
	static LwrErrorMsg elbowIntervals(
			LwrElbowInterval currentMargin, // margin for the elbow within its current interval
			LwrElbowInterval reachable[11], // intervals that are reachable
			LwrElbowInterval blocked[11], // intervals that are blocked
			LwrElbowInterval blockedPerJoint[7][3], // which interval is blocked by which joint
			LwrXCart& xCartPose); // target pose

	//! \brief Perform one interpolation step
	void interpolate();
};

#endif // LWR_LIBRARY_HPP
