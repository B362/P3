/* 
This library performs calculations for the forward and inverse dynamics.
*/

#ifndef fidynamics_h
#define fidynamics_h

#include "WConstants.h"
#include "math.h"

// library interface description
class fidynamics
{
  public:

	  fidynamics(void);	// This does nothing

	  void getForwardDynamics();
	  void getInverseDynamics();
	  void getXYZpos(double, double, double, double, double, double, double, double);
	  void getAngles(double, double, double, double, double, double, double, double);

  private:
};

#endif

