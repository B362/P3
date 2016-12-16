// CONTROLLER FOR THE CRUSTCRAWLER ROBOT
//
// TODO Run controller in iterations of									...OK!
// TODO Change the timestep to adapt to the real delay (use millis())

#include <math.h>
#include <ax12.h>
#include <dynamixel_mx.h>

// DEFINITIONS

#define TIMESTEP 0.3	// In seconds
#define TESTSPEED 0.05
#define TORQUECONVERSION1 1000
#define TORQUECONVERSION2 1000
#define TORQUECONVERSION3 1000

// Proportional gains
double pospgain[3] = { 3.015, 3.015, 3.015 };
double spdpgain[3] = { 30.15, 30.15, 30.15 };
// Integral gains
double postigain[3] = { 0, 0, 0 };
double posglobalsum[3] = { 0, 0, 0 };

double ctrltorque[3] = { 0, 0, 0 };
// Global register of previous values of theta and dtheta
// for derivation purposes
double prevreftheta[3] = { 0, 0, 0 };
double prevrefdtheta[3] = { 0, 0, 0 };
double prevfbtheta[3] = { 0, 0, 0 };
double prevfbdtheta[3] = { 0, 0, 0 };
// Has torque been enabled?
bool tenabled[3];

//-----------------------------------------------SETUP-ROUTINE--------------------------------------------//

int nominal_positions[5];
int dir[5];
long int e, d;

float Kp = 10;//32.861;
float wr = 10.0;
float w = 0.0;
float torque = 0;
float ntorque = 0;
float zero = 0.0;
float spid, curr, ppos;

void setup()
{
	// Initialize the communication with the servos.
	// The default baud rate for the robot arms is
	// 1 Mbps.
	ax12Init(1000000);

	// Set torque on (stiffen the joints) on all servos.
	// (ID 254 is the braoadcasting address).
	TorqueControlDisable(1);
	TorqueControlDisable(2);
	TorqueControlDisable(3);
	TorqueControlDisable(4);
	TorqueControlDisable(5);
	for (int i = 0; i < 3; i++) tenabled[i] = false;
	TorqueOn(254);
	// Begin the serial communication
	Serial.begin(115200);


	// Delay a short while to get correct readings.
	delay(200);
    Serial.println('RS');

	// Raise the arm to nominal
	// - Set nominal positions
	nominal_positions[0] = 2048;
	nominal_positions[1] = 2048;
	nominal_positions[2] = 2048;
	nominal_positions[3] = 2048;
	nominal_positions[4] = 2048;

	// - check how far we are off
	for (int i = 0; i < 5; ++i){
		dir[i] = GetPosition(i + 1) - nominal_positions[i];
	}

	// - Send trajectory
	bool at_home = false;
	while (!at_home){
		for (int i = 0; i < 5; ++i){
			int set_point = nominal_positions[i] + dir[i];
			if (dir[i] > 0){
				dir[i] -= 1;
			}
			else if (dir[i] < 0){
				dir[i] += 1;
			}
			SetPosition(i + 1, set_point);
		}

		// - Check if we are done
		at_home = true;
		for (int i = 0; i < 5; ++i){
			if (dir[i] != 0){
				at_home = false;
				break;
			}
		}

		// - Delay a while. Increase to move slower.
		delay(5);
	}
        Serial.println('OK!');
	//SetPidGains(1, 0, 0, 0);
	//SetPidGains(2, 0, 0, 0);
	//SetPidGains(3, 0, 0, 0);
	delay(200);

//	TorqueControlEnable(1);
//	TorqueControlEnable(2);
//	TorqueControlEnable(3);
//	TorqueControlDisable(5);
//	TorqueControlDisable(6);
}

//------------------------------------------------VOID-LOOP-----------------------------------------------//

void loop() {
	
	// Simulating a hello arm

	// Servo 1
	while (GetPosition(1) < 3072) {
		control(TESTSPEED, 0, 0);
		delay(1000 * TIMESTEP);
	}
	while (GetPosition(1) > 1024) {
		control(-TESTSPEED, 0, 0);
		delay(1000 * TIMESTEP);
	}
	while (GetPosition(1) < 2048) {
		control(TESTSPEED, 0, 0);
		delay(1000 * TIMESTEP);
	}

	// Servo 2
	while (GetPosition(2) < 3072) {
		control(0, TESTSPEED, 0);
		delay(1000 * TIMESTEP);
	}
	while (GetPosition(2) > 1024) {
		control(0, -TESTSPEED, 0);
		delay(1000 * TIMESTEP);
	}
	while (GetPosition(2) < 2048) {
		control(0, TESTSPEED, 0);
		delay(1000 * TIMESTEP);
	}

	// Servo 3
	while (GetPosition(3) < 3072) {
		control(0, 0, TESTSPEED);
		delay(1000 * TIMESTEP);
	}
	while (GetPosition(3) > 1024) {
		control(0, 0, -TESTSPEED);
		delay(1000 * TIMESTEP);
	}
	while (GetPosition(3) < 2048) {
		control(0, 0, TESTSPEED);
		delay(1000 * TIMESTEP);
	}
	delay(100);
}

//------------------------------------------------CONTROLLER-----------------------------------------------//

void control(double speed1, double speed2, double speed3) {

	// ARBOTIX RECEIVES DATA FROM TRAJECTORY PLANNING AND FEEDBACK FROM SERVOS

	// Receive data from serial and arm

	double reftheta[3];
	double refdtheta[3];
	double refddtheta[3];

	refdtheta[0] = speed1;
	refdtheta[1] = speed2;
	refdtheta[2] = speed3;


	double fbtheta[3];
	double fbdtheta[3];
	double fbddtheta[3];

        fbtheta[0] = 0.001534 * (GetPosition(1) - 2048);
        fbtheta[1] = 0.001534 * (GetPosition(2) - 2048);
        fbtheta[2] = 0.001534 * (GetPosition(3) - 2048);

	for (int x = 0; x < 3; x++) {
		reftheta[x] += TIMESTEP * prevrefdtheta[x];
//		refdtheta[x] = (reftheta[x] - prevreftheta[x]) / TIMESTEP;
		refddtheta[x] = (refdtheta[x] - prevrefdtheta[x]) / TIMESTEP;
		fbdtheta[x] = (fbtheta[x] - prevfbtheta[x]) / TIMESTEP;
		fbddtheta[x] = (fbdtheta[x] - prevfbdtheta[x]) / TIMESTEP;
	}

	// SET BOUNDARIES FOR THE ARM POSITION

	if (reftheta[0] >= 6.25 || reftheta[0] <= 0) refdtheta[0] = prevreftheta[0];
	if (reftheta[1] >= 4.7 || reftheta[1] <= 1.55) refdtheta[1] = prevreftheta[1];
	if (reftheta[2] >= 4.7 || reftheta[2] <= 1.55) refdtheta[2] = prevreftheta[2];

	// THE MAGIC HAPPENS HERE

	double poserr[3];
	double spderr[3];

		for (int x = 0; x < 3; x++)
		{
			poserr[x] = pospgain[x] * (reftheta[x] - fbtheta[x]) + postigain[x] * posglobalsum[x];
			spderr[x] = spdpgain[x] * (refdtheta[x] - fbdtheta[x]);
			posglobalsum[x] += TIMESTEP * poserr[x];

			refddtheta[x] += poserr[x] + spderr[x];
		}

	// CONVERT ACCELERATION TO TORQUE

	getTorque(fbtheta[0], fbtheta[1], fbtheta[2], fbdtheta[0], fbdtheta[1], fbdtheta[2], refddtheta[0], refddtheta[1], refddtheta[2], &ctrltorque[0], &ctrltorque[1], &ctrltorque[2]);

	// We will have the updated values for the control torques,
	// so we can add a linear value to convert from Nm to Amp�re

	ctrltorque[0] *= TORQUECONVERSION1;
	ctrltorque[1] *= TORQUECONVERSION2;
	ctrltorque[2] *= TORQUECONVERSION3;

	// SEND SIGNAL TO ROBOT AND LOOP

	// Send torque

	//SetTorque(1, ctrltorque[0]);
	//SetTorque(2, ctrltorque[1]);
	//SetTorque(3, ctrltorque[2]);

	if (refdtheta[0] == 0) {
		if (tenabled[0]) {
			TorqueControlDisable(1);
			tenabled[0] = false;
		}
		SetPosition(1, fbtheta[0]);
	}
	else {
		if (!tenabled[0]) {
			TorqueControlEnable(1);
			tenabled[0] = true;
		}
		SetTorque(1, ctrltorque[0]);
	}

	if (refdtheta[1] == 0) {
		if (tenabled[1]) {
			TorqueControlDisable(2);
			tenabled[1] = false;
		}
		SetPosition(2, fbtheta[1]);
	}
	else {
		if (!tenabled[1]) {
			TorqueControlEnable(2);
			tenabled[1] = true;
		}
		SetTorque(2, ctrltorque[1]);
	}

	if (refdtheta[2] == 0) {
		if (tenabled[2]) {
			TorqueControlDisable(3);
			tenabled[2] = false;
		}
		SetPosition(3, fbtheta[2]);
	}
	else {
		if (!tenabled[2]) {
			TorqueControlEnable(3);
			tenabled[2] = true;
		}
		SetTorque(3, ctrltorque[2]);
	}

	// Update variables
	for (int x = 0; x < 3; x++) {
		prevreftheta[x] = reftheta[x];
		prevrefdtheta[x] = refdtheta[x];
		prevfbtheta[x] = fbtheta[x];
		prevfbdtheta[x] = fbdtheta[x];
	}

	Serial.print(fbtheta[0]);
	Serial.print(",");
	Serial.print(fbtheta[1]);
	Serial.print(",");
	Serial.print(fbtheta[2]);
	Serial.print(",");
	Serial.print(0.001 * ctrltorque[0]);
	Serial.print(",");
	Serial.print(0.001 * ctrltorque[1]);
	Serial.print(",");
	Serial.println(0.001 * ctrltorque[2]);

	return;
}

void getTorque(double theta1, double theta2, double theta3, double dtheta1, double dtheta2, double dtheta3, double ddtheta1, double ddtheta2, double ddtheta3, double *torque1, double *torque2, double *torque3)
{
	//= == == == == == == == == == == == == == == == == == == = link 1
	double l1 = 0.235; // length[m]
	double d1 = 0.160; // mass center
	double m1 = 0.228; // mass[kg]
	// moment of inertia
	double I1xx = 0.00006781; double I1xy = 0.00002608; double I1xz = 0;
	double I1yx = 0.00002608; double I1yy = 0.00631914; double I1yz = 0;
	double I1zx = 0; double I1zy = 0; double I1zz = 0.00630459;
	//= == == == == == == == == == == == == == == == == == == = link 2
	double l2 = 0.270; // length[m]
	double d2 = 0.135; // mass center
	double m2 = 0.275; // mass[kg]
	// moment of inertia
	double I2xx = 0.00012618; double I2xy = 0.00001090; double I2xz = 0;
	double I2yx = 0.00001090; double I2yy = 0.00062333; double I2yz = 0;
	double I2zx = 0; double I2zy = 0; double I2zz = 0.00055076;

	double g = 9.801;
	theta2 += 1.571;

	double H11 = (2 * I1zz + 2 * I2zz + pow(d2, 2)*m2 + pow(l1, 2)*m2 + pow(l1, 2)*m2*cos(2 * theta2) + 2 * d2*l1*m2*cos(theta3) + pow(d2, 2)*m2*cos(2 * (theta2 + theta3)) + 2 * d2*l1*m2*cos(2 * theta2 + theta3)) / 2;
	double H12 = (0 - (I1yz*cos(theta1)) - I1zy*cos(theta1) - I2yz*cos(theta1) - I2zy*cos(theta1) + I1xz*sin(theta1) + I1zx*sin(theta1) + I2xz*sin(theta1) + I2zx*sin(theta1)) / 2;
	double H13 = (0 - (I2yz*cos(theta1)) - I2zy*cos(theta1) + I2xz*sin(theta1) + I2zx*sin(theta1)) / 2;
	double H21 = (0 - (I1yz*cos(theta1)) - I1zy*cos(theta1) - I2yz*cos(theta1) - I2zy*cos(theta1) + I1xz*sin(theta1) + I1zx*sin(theta1) + I2xz*sin(theta1) + I2zx*sin(theta1)) / 2;
	double H22 = (I1xx + I1yy + I2xx + I2yy + 2 * pow(d2, 2)*m2 + 2 * pow(l1, 2)*m2 - I1xx*cos(2 * theta1) + I1yy*cos(2 * theta1) - I2xx*cos(2 * theta1) + I2yy*cos(2 * theta1) + 4 * d2*l1*m2*cos(theta3) - I1xy*sin(2 * theta1) - I1yx*sin(2 * theta1) - I2xy*sin(2 * theta1) - I2yx*sin(2 * theta1)) / 2;
	double H23 = (I2xx + I2yy + 2 * pow(d2, 2)*m2 - I2xx*cos(2 * theta1) + I2yy*cos(2 * theta1) + 2 * d2*l1*m2*cos(theta3) - I2xy*sin(2 * theta1) - I2yx*sin(2 * theta1)) / 2;
	double H31 = (0 - (I2yz*cos(theta1)) - I2zy*cos(theta1) + I2xz*sin(theta1) + I2zx*sin(theta1)) / 2;
	double H32 = (I2xx + I2yy + 2 * pow(d2, 2)*m2 - I2xx*cos(2 * theta1) + I2yy*cos(2 * theta1) + 2 * d2*l1*m2*cos(theta3) - I2xy*sin(2 * theta1) - I2yx*sin(2 * theta1)) / 2;
	double H33 = (I2xx + I2yy + 2 * pow(d2, 2)*m2 - I2xx*cos(2 * theta1) + I2yy*cos(2 * theta1) - I2xy*sin(2 * theta1) - I2yx*sin(2 * theta1)) / 2;

	double C11 = 0-2 * m2*(l1*cos(theta2) + d2*cos(theta2+theta3))*(l1*sin(theta2) + d2*sin(theta2+theta3))*dtheta2-2 * d2*m2*(l1*cos(theta2) + d2*cos(theta2+theta3))*sin(theta2+theta3)*dtheta3;
	double C12 = ((I1xy*cos(2 * theta1) + I1yx*cos(2 * theta1) + I2xy*cos(2 * theta1) + I2yx*cos(2 * theta1) - I1xx*sin(2 * theta1) + I1yy*sin(2 * theta1) - I2xx*sin(2 * theta1) + I2yy*sin(2 * theta1))*dtheta2) / 2.;
	double C13 = (I2xy*cos(2 * theta1) + I2yx*cos(2 * theta1) - I2xx*sin(2 * theta1) + I2yy*sin(2 * theta1))*dtheta2+((I2xy*cos(2 * theta1) + I2yx*cos(2 * theta1) - I2xx*sin(2 * theta1) + I2yy*sin(2 * theta1))*dtheta3) / 2.;
	double C21 = ((I1xz*cos(theta1) + I1zx*cos(theta1) + I2xz*cos(theta1) + I2zx*cos(theta1) + I1yz*sin(theta1) + I1zy*sin(theta1) + I2yz*sin(theta1) + I2zy*sin(theta1) + pow(l1, 2)*m2*sin(2 * theta2) + pow(d2, 2)*m2*sin(2 * (theta2+theta3)) + 2 * d2*l1*m2*sin(2 * theta2+theta3))*dtheta1) / 2. + (-(I1xy*cos(2 * theta1)) - I1yx*cos(2 * theta1) - I2xy*cos(2 * theta1) - I2yx*cos(2 * theta1) + I1xx*sin(2 * theta1) - I1yy*sin(2 * theta1) + I2xx*sin(2 * theta1) - I2yy*sin(2 * theta1))*dtheta2+(-(I2xy*cos(2 * theta1)) - I2yx*cos(2 * theta1) + I2xx*sin(2 * theta1) - I2yy*sin(2 * theta1))*dtheta3;
	double C22 = 0-2 * d2*l1*m2*sin(theta3)*dtheta3;
	double C23 = 0-(d2*l1*m2*sin(theta3)*dtheta3);
	double C31 = ((I2xz*cos(theta1) + I2zx*cos(theta1) + I2yz*sin(theta1) + I2zy*sin(theta1) + 2 * d2*l1*m2*cos(theta2)*sin(theta2+theta3) + 2 * pow(d2, 2)*m2*cos(theta2+theta3)*sin(theta2+theta3))*dtheta1) / 2. + (-(I2xy*cos(2 * theta1)) - I2yx*cos(2 * theta1) + I2xx*sin(2 * theta1) - I2yy*sin(2 * theta1))*dtheta2+(-(I2xy*cos(2 * theta1)) - I2yx*cos(2 * theta1) + I2xx*sin(2 * theta1) - I2yy*sin(2 * theta1))*dtheta3;
	double C32 = d2*l1*m2*sin(theta3)*dtheta2;
	double C33 = 0;

	double G1 = 0;
	double G2 = g*(d1*m1*cos(theta2) + l1*m2*cos(theta2) + d2*m2*cos(theta2+theta3));
	double G3 = d2*g*m2*cos(theta2+theta3);

	*torque1 = H11*ddtheta1 + H12*ddtheta2 + H13*ddtheta3 + C11*dtheta1 + C12*dtheta2 + C13*dtheta3 + G1 + ctsign(0.2);
	*torque2 = H21*ddtheta1 + H22*ddtheta2 + H23*ddtheta3 + C21*dtheta1 + C22*dtheta2 + C23*dtheta3 + G2 + ctsign(0.75);
	*torque3 = H31*ddtheta1 + H32*ddtheta2 + H33*ddtheta3 + C31*dtheta1 + C32*dtheta2 + C33*dtheta3 + G3 + ctsign(0.4);

	return;
}

double ctsign(double value) {
	if (value <= 0.001 || value >= -0.001) return 0;
	else return value / (abs(value));
}
