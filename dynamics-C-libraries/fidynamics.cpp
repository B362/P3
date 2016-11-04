/*
This library performs calculations for the forward dynamics,
inverse dynamics and jacobian conversions (hence the name FIJ)
*/

#include "WProgram.h"
#include "fidynamics.h"
#include <math.h>

// Constructor /////////////////////////////////////////////////////////////////

fidynamics::fidynamics(void)
{
}

// Public Methods //////////////////////////////////////////////////////////////

void fidynamics::getForwardDynamics()
{

}

void fidynamics::getInverseDynamics(double length1, double length2, double com1, double com2, double m1, double m2, double inertia1[3][3], double inertia2[3][3], double *brot, double *btheta, double *mtheta, double *dbrot, double *dbtheta, double *dmtheta, double *ddbrot, double *ddbtheta, double *ddmtheta)
{
	double temp1 = 2 * btheta + 2 * mtheta;
	double temp2 = 2 * btheta + mtheta;

	H11 = 0.5 * (pow(com2, 2) * m2 * cos(temp1) + pow(com2, 2) * m2 + 2 * com2 * length1 * m2 * cos(temp2) + 2 * com2 * length1 * m2 * cos(mtheta) + 2 * inertia1[2][2] + 2 * inertia2[2][2] + pow(length1, 2) * m2 * cos(2 * btheta) + pow(length1, 2) * m2);
}

void fidynamics::getXYZpos(double length1, double length2, double *Xpos, double *Ypos, double *Zpos, double *brot, double *btheta, double *mtheta)
{
	*Xpos = length1 * cos(btheta) * cos(brot) + length2 * cos(btheta + mtheta) * cos(brot);

	*Ypos = length1 * cos(btheta) * sin(brot) + length2 * cos(btheta + mtheta) * sin(brot);

	*Zpos = length1 * sin(btheta) + length2 * sin(btheta + mtheta);
}

void fidynamics::getAngles(double length1, double length2, double *Xpos, double *Ypos, double *Zpos, double *brot, double *btheta, double *mtheta)
{
	// Waiting for inverse kinematics
}

// Private Methods /////////////////////////////////////////////////////////////

