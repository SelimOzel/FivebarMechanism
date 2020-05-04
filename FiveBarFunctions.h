/*
  FiveBarFunctions.h - Library for kinematic functions
  of five bar mechanisms. I use this inside Arduino/mbed
  to control robots.
*/

#ifndef FiveBarFunctions_h
#define FiveBarFunctions_h
#include "math.h"

class FiveBarFunctions
{
  public:
	// Constructor and overloaded operations.
    FiveBarFunctions(double a, double b, double c, double d, double f, double SL);
	FiveBarFunctions(double a, double b, double c, double d, double f, double SL, double theta2_Init, double theta5_Init);
	
	// Fivebar functions
	double 	FBar_Jacobian(double theta2_IN, double theta5_IN, double theta3_IN, double theta4_IN, double *J);	
    void 	FBar_OutputAngles(int theta2_IN, int theta5_IN, double *result);
	void 	FBar_ForwardKinematics(double theta2_IN, double theta5_IN, double theta3_IN, double theta4_IN, int theta2INT_IN, int theta3INT_IN, double *positionList);
	void 	FBar_InverseKinematics(double posX_IN, double posY_IN, double theta2_IN, double theta5_IN, int theta2INT_IN, int theta5INT_IN, double *targetTheta);
	double 	FBar_UpdateMechanism(int theta2INT_IN, int theta5INT_IN);
	void 	FBar_GetNeutralAngles(double *res);
	void 	FBar_GetCurrentTipPos(double *res);
	void 	FBar_GetCurrentJacobian(double *res);
	void 	FBar_GetCurrentJacobianInv(double *res);
	
	// Math functions
	void 	Math_Inv2x2Mat(double *Mat, double *res);
	void 	Math_22x21MatrixMult(double *Mat, double *vec, double *res);
	void 	Math_2DRotMat(double rot, double *vec, double *res);
	double  Math_fastAtan(double x);  
	double  Math_fastAtan2(double y, double x);
	double  Math_abs(double x);
	double 	Math_SinLUT(int x);
	double  Math_CosLUT(int x);
  private:
	// Optimized functions are private. They are used in real time. 
	double  FBar_JacobianOptimized(double theta3_IN, double theta4_IN, double *J);
	void 	FBar_ForwardKinematicsOptimized(double *positionList, double t3_IN);	  
	void 	FBar_Init(double a, double b, double c, double d, double f, double SL); // Initialization for overloaded constructors.
  
	// Link lengths
    double _a;
	double _b;
	double _c;
	double _d;
	double _f;
	
	double _SL; // Tip length
	
	// Current tip pos
	double _posX;
	double _posY;
	
	// Jacobian at current real time cycle
	double _J11;
	double _J12;
	double _J21;
	double _J22;
	
	// Inverse Jacobian at current real time cycle
	double _Jinv11;
	double _Jinv12;
	double _Jinv21;
	double _Jinv22;
	
	// Computation optimization
    double _a_sqrd;
	double _b_sqrd;
	double _c_sqrd;
	double _d_sqrd;
	double _f_sqrd;
	double _b_twoTimes;
	double _c_twoTimes;
	double _d_twoTimes;
	double _ad_twoTimes;
	double _af_twoTimes;
	double _K1;
	double _K2;
	
	// Computation optimization on fivebar angles
	double _c_t2;
	double _c_t3;
	double _c_t4;
	double _c_t5;
	double _s_t2;
	double _s_t3;
	double _s_t4;
	double _s_t5;
	
	// Neutral angles
	double _t2_Neutral;
	double _t5_Neutral;

	// Geometric constants for faster computations.
	double _pi;	
	double _twopi;
	double _threeOverTwopi; 
	double _piOverTwo;
	double _piOverFour;
	
	// Sin lookuptable
	//double* sinLookupTable;
};

#endif
