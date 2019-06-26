/* Allegro, Copyright 2012 SimLab Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF SimLab Co., LTD.
 */

/**
 * @file BHand.h
 * @author SimLab
 * @brief Class definition of BHand.
 *
 * @mainpage BHand
 * @section INTRODUCTION
 * Implementation of Allegro Hand grasping algorithm.<br>
 * The algorithm is originally published at ICRA 2012.<br>
 * For the detail of the algorithm please refer to the paper, 
 * "A Grasp Strategy with the Geometric Centroid of a Groped Object
 * Shape Derived from Contact Spots" by Ji-Hun Bae, Sung-Woo Park,
 * Doik-Kim, Moon-Hong Baeg, and Sang-Rok Oh.<br>
 * 
 * @section CREATEINFO
 * - Author: Sangyup (Sean) Yi
 * - Data: 2012/08/31
 * @section MODIFYINFO
 * - 2012/08/31: Commit this document. (by Sean)
 */

#ifndef __BHAND_H__
#define __BHAND_H__

#include "BHandDef.h"

#define NOF 4 // number of fingers
#define NOJ 4 // number of joints in each finger
#define SIZEOF_VARRAY (NOF*NOJ*8)

/**
 * Type of motion.
 * @brief Motion type set by user.
 */
enum eMotionType
{
	eMotionType_NONE,				///< power off
	eMotionType_HOME,				///< go to home position
	eMotionType_READY,				///< finger position move motion (ready)
	eMotionType_GRAVITY_COMP,		///< finger position move motion (gravity compensation)
	eMotionType_PRE_SHAPE,			///<
	eMotionType_GRASP_3,			///< grasping using 3 fingers
	eMotionType_GRASP_4,			///< grasping using 4 fingers
	eMotionType_PINCH_IT,			///< pinching using index finger and thumb
	eMotionType_PINCH_MT,			///< pinching using middle finger and thumb
	eMotionType_OBJECT_MOVING,		///<
	eMotionType_ENVELOP,			///< enveloping
	eMotionType_JOINT_PD,			///< joint pd control
	eMotionType_MOVE_OBJ,
	eMotionType_FINGERTIP_MOVING,
	NUMBER_OF_MOTION_TYPE
};

/**
 * Type of hand. It is input argument to instanciate BHand class.
 * @brief It specifies whether it is for left hand or right hand.
 */
enum eHandType
{
	eHandType_Left = 0,				///< left hand
	eHandType_Right					///< right hand
};

/**
 * BHand class.
 * @brief Allegro Hand grasping algorithm.
 * @author Jihoon Bae, SimLab
 */
class BHANDEXPORT BHand
{
public:
	BHand(eHandType ht);
	~BHand(void);

	/**
	 * Get the type of hand.
	 * @return It returns hand type, whether it is left or right hand.
	 * @see eHandType
	 */
	eHandType GetType();

	/**
	 * Set time interval.
	 * Algorithm assumes UpdateControl() is called once in this time interval.
	 * @param dT Time interval in seconds.
	 */
	void SetTimeInterval(double dT);

	/**
	 * Get time interval.
	 * Algorithm assumes UpdateControl() is called once in this time interval.
	 * @param dT Time interval in seconds.
	 */
	double GetTimeInterval();

	/**
	 * Set motion type.
	 * @param motionType Type of motion to set.
	 * @see eMotionType
	 */
	void SetMotionType(int motionType);

	/**
	 * Set joint position of each joint. 
	 * The hand has sixteen joints totally. 
	 * The dimension of 'q' should be sixteen.
	 * This function should be called once in a control loop before UpdateControl() is called.
	 * @param q Array of joint positions in radian.
	 */
	void SetJointPosition(double* q);

	/**
	 * Update control algorithm callback. This function should be called in each control time step manually.
	 * Desired joint torques are computed once in every control loop.
	 * @param time Current time in second.
	 */
	void UpdateControl(double time);

	/**
	 * Get desired(computed) joint torques.
	 * This function should be called once in a control loop after UpdateControl() is called.
	 * @param tau [out] Array of desired joint torques.
	 */
	void GetJointTorque(double* tau);

	/**
	 * Get forward kinematic solution for each finger.
	 * @param x [out] x coordinate of each fingertip.
	 * @param y [out] y coordinate of each fingertip.
	 * @param z [out] z coordinate of each fingertip.
	 */
	void GetFKResult(double x[4], double y[4], double z[4]);

	/**
	 * Set desired joint position.
	 * @param q Array of desired joint positions in radian.
	 */
	void SetJointDesiredPosition(double* q);

	/**
	 * Set control gains explicitly.
	 * @param kp proportional gains
	 * @param kd derivative gains
	 */
	void SetGainsEx(double* kp, double* kd);

	/**
	 * Set desired grasping forces.
	 * @param f desired grasping force for each finger
	 */
	void SetGraspingForce(double f[4]);

	/**
	 * Set scalar for the preset envelop grasp torque.
	 * @param set_scalar will be multiplied by the pre-computed grasping torque
	 * at each joint. Default is one to rest the torque to its original value
	 */
	void SetEnvelopTorqueScalar(double set_scalar=1.0);

	/**
	 * Get desired grasping forces.
	 * @param fx [out] desired grasping force in x-direction for each finger
	 * @param fy [out] desired grasping force in y-direction for each finger
	 * @param fz [out] desired grasping force in z-direction for each finger
	 */
	void GetGraspingForce(double fx[4], double fy[4], double fz[4]);

	/**
	 * An attempt on object moving
	 */
	void SetObjectDisp(double x_d_o[3]);

	/**
	 * An attempt on object moving
	 */
	void MoveFingerTip(double set_xyz_0[3], double set_xyz_1[3], double set_xyz_2[3], double set_xyz_3[3]);

	/**
	 * Set orientation
	 */
	void SetOrientation(double roll, double pitch, double yaw);
	void SetOrientation(double R[9]);


private:
	BHand();
	void SetGains(int motionType);
	void SolveFK();
	void SolveFKLeft();
	void SolveFKRight();
	void CalculateJacobian();
	//void CalculateJacobianLeft();
	//void CalculateJacobianRight();
	void CalculateGravity();
	void CalculateGravityEx();
	//void CalculateGravityLeft();
	//void CalculateGravityRight();

	void Motion_HomePosition();
	void Motion_Ready();
	void Motion_GravityComp();
	void Motion_ReadyToMove();
	void Motion_PreShape();
	void Motion_Grasp3();
	void Motion_Grasp4();
	void Motion_PinchIT();
	void Motion_PinchMT();
	void Motion_ObjectMoving();
	void Motion_FingertipMoving();
	void Motion_Envelop();
	void Motion_JointPD();

private:
	double _dT;							///< control time step (second)
	double _curT;
	
	eHandType _handType;				///> whether it is for left hand or right
	eMotionType _motionType;			///< type of motion currently set
	
	double _q[NOF][NOJ];				///< current joint angle (radian)
	double _q_filtered[NOF][NOJ];		///< current joint angle (radian, low pass filtered)
	double _q_pre[NOF][NOJ];			///< previous(last) joint angle (radian)
	double _q_filtered_pre[NOF][NOJ];	///< previous(last) joint angle (radian, low pass filtered)
	double _qdot[NOF][NOJ];				///< joint velocity (radian/sec)
	double _qdot_filtered[NOF][NOJ];	///< joint velocity (radian/sec, low pass filtered)
	double _qdot_pre[NOF][NOJ];			///< previous(last) joint velocity (radian/sec)
	double _tau_des[NOF][NOJ];			///< desired joint torque
	double _q_des[NOF][NOJ];			///< desired joint position used in joint pd control motion

	double _envelop_torque_scalar;		///< used to control (scale) the torque applied during enveloping grasp

	double _kp[NOF][NOJ];				///< proportional control gain for each joint
	double _kd[NOF][NOJ];				///< derivative control gain for each joint
	double _kp_task[NOF][NOJ];			///<
	double _kd_task[NOF][NOJ];			///<

	double _x[NOF];						///< x position of finger tip along with cartesian space
	double _y[NOF];						///< y position of finger tip along with cartesian space
	double _z[NOF];						///< z position of finger tip along with cartesian space
	double _x_filtered[NOF];			///< x position of finger tip along with cartesian space (low pass filtered)
	double _y_filtered[NOF];			///< y position of finger tip along with cartesian space (low pass filtered)
	double _z_filtered[NOF];			///< z position of finger tip along with cartesian space (low pass filtered)
	double _x_pre[NOF];					///< previous x position of finger tip along with cartesian space
	double _y_pre[NOF];					///< previous y position of finger tip along with cartesian space
	double _z_pre[NOF];					///< previous z position of finger tip along with cartesian space
	double _x_filtered_pre[NOF];		///< previous x position of finger tip along with cartesian space (low pass filtered)
	double _y_filtered_pre[NOF];		///< previous y position of finger tip along with cartesian space (low pass filtered)
	double _z_filtered_pre[NOF];		///< previous z position of finger tip along with cartesian space (low pass filtered)
	
	double _xdot[NOF];					///< velocity of finger tip along with x-axis
	double _ydot[NOF];					///< velocity of finger tip along with y-axis
	double _zdot[NOF];					///< velocity of finger tip along with z-axis
	double _xdot_filtered[NOF];			///< velocity of finger tip along with x-axis (low pass filtered)
	double _ydot_filtered[NOF];			///< velocity of finger tip along with y-axis (low pass filtered)
	double _zdot_filtered[NOF];			///< velocity of finger tip along with z-axis (low pass filtered)
	double _xdot_pre[NOF];				///< previous velocity of finger tip along with x-axis
	double _ydot_pre[NOF];				///< previous velocity of finger tip along with y-axis
	double _zdot_pre[NOF];				///< previous velocity of finger tip along with z-axis

	double _J[NOF][3][NOJ];				///< Jacobian

	double _G[NOF][NOJ];				///< gravitational vector

	double _f_des[NOF];					///< desired force

	double _R[9];						///< body(palm) orientation

	double x_d_object;
	double y_d_object;
	double z_d_object;

	double x_move;
	double y_move;
	double z_move;

	double _x_des[NOF];					///< desired x position in cartesian coordinate for the center of each finger tip
	double _y_des[NOF];					///< desired y position in cartesian coordinate for the center of each finger tip
	double _z_des[NOF];					///< desired z position in cartesian coordinate for the center of each finger tip

	double _set_x_des[NOF];				// Set by user in MoveFingerTip 
	double _set_y_des[NOF];				// Set by user in MoveFingerTip
	double _set_z_des[NOF];				// Set by user in MoveFingerTip

	double S1_C[NOJ], S2_C[NOJ], S3_C[NOJ], S4_C[NOJ], S23_C[NOJ], S234_C[NOJ], S34_C[NOJ];
	double C1_C[NOJ], C2_C[NOJ], C3_C[NOJ], C4_C[NOJ], C23_C[NOJ], C234_C[NOJ], C34_C[NOJ];

	double _mass[NOF][NOJ];				///< link mass
};

BHAND_EXTERN_C_BEGIN

/**
 * @brief Creates an instance of Allegro Hand control algorithm. It is for left hand.
 */
BHANDEXPORT BHand* bhCreateLeftHand();

/**
 * @brief Creates an instance of Allegro Hand control algorithm. It is for right hand.
 */
BHANDEXPORT BHand* bhCreateRightHand();

BHAND_EXTERN_C_END

#endif // __BHAND_H__