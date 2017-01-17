// Import the ax12 library that provides
// commands to control the Dynamixel servos.
#include <ax12.h>
// Additional dynamixel functionality for MX servos.
#include <dynamixel_mx.h>

#define JOINT 1
#define TIMESTEP 0.020

int nominal_positions[5];
int dir[5];
long int e,d; 

float Kv = 30;//32.861;
float Kp = 3.015;
float wr = 10.0;
float wr2 = -10.0;
float w = 0.0;
float torquep = 0;
float torque = 0;
float ntorque = 0;
float zero = 0.0;
float spid , curr, ppos;
float prevwr = 0;
float prevwr2 = 0;
//float prevw = 0;
float omegar;

float reftheta = 0;
float fbtheta = 0;
float refddtheta = 0;

//------------------------------SETUP-ROUTINE------------------------------------------------//

void setup()
{
  // Initialize the communication with the servos.
  // The default baud rate for the robot arms is
  // 1 Mbps.
  ax12Init(1000000);

  // Set torque on (stiffen the joints) on all servos.
  // (ID 254 is the braoadcasting address).
  TorqueControlDisable(1);
  delay(2);
  TorqueControlDisable(2);
  delay(2);
  TorqueControlDisable(3);
  delay(2);
  TorqueOn(254);
  // Begin the serial communication
  Serial.begin(115200);


  // Delay a short while to get correct readings.
  delay(100);

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
        //Serial.println('OK!');
	//SetPidGains(1, 0, 0, 0);
	//SetPidGains(2, 0, 0, 0);
	//SetPidGains(3, 0, 0, 0);
	delay(200);
        TorqueOn(254);
        delay(2);
        to_serial(zero, GetPosition(JOINT), zero, GetSpeed(JOINT), zero, GetCurrent(JOINT));
        delay(20);
        to_serial(zero, GetPosition(JOINT), zero, GetSpeed(JOINT), zero, GetCurrent(JOINT));
        delay(20);
        to_serial(zero, GetPosition(JOINT), zero, GetSpeed(JOINT), zero, GetCurrent(JOINT));
        delay(20);
}

//------------------------------------------------LOOP------------------------------//

void loop()
{
  //---------------------------------------Testing joint 1
 /* 
  d = millis();
  e = millis();
  TorqueControlEnable(1);
  while(millis() - e < 500) //does the kp control for 7 seconds 
  {
    control(1);
  }
  delay(2);
  
  e = millis();
  while(millis() - e < 500) //does the kp control for 7 seconds 
  {
    control2(1);
  }
  delay(2);

  TorqueControlDisable(1);
  delay(2000);

  //------------------------------------------Testing joint 3
  TorqueControlEnable(3);
  e = millis();
  while(millis() - e < 1000) //does the kp control for 7 seconds 
  {
    control(3);
  }
  delay(2);
  
  e = millis();
  while(millis() - e < 1000) //does the kp control for 7 seconds 
  {
    control2(3);
  }
  delay(2);
  e = millis();
  while(millis() - e < 1000) //does the kp control for 7 seconds 
  {
    control(3);
  }
  delay(2);
  TorqueControlDisable(3);
  delay(2000);
  */
  
  //------------------------------------------------------Testing joint 2
  e = millis();
  TorqueControlEnable(1);
  d = millis();
  while(millis() - e < 1000) //does the kp control for 7 seconds 
  {
    control(1);
  }
  delay(2);
  
  e = millis();
  while(millis() - e < 1000) //does the kp control for 7 seconds 
  {
    control2(1);
  }
  delay(2);
  
  TorqueControlDisable(1);
  delay(2000);
  /*
  e = millis();
  while(millis() -e < 200) //rest 200 ms without doing control 
  {
    spid = GetSpeed(JOINT);
    delay(2);
    curr = GetCurrent(JOINT);
    delay(2);
    ppos =  GetPosition(JOINT);
    if( millis() - d > 20) 
    {
      to_serial(zero, ppos, zero, spid, zero, curr);
      d = millis();
    }
  }
  */
  //while(true){}; //wait infinitely 
//  delay(2000);
}

//---------------------------------------------Function control positive direction

void control(int jointnr, float wr)
{
  reftheta += TIMESTEP * prevwr;
  fbtheta = 0.001534 * (GetPosition(jointnr) - 2048);
  refddtheta = (wr - prevwr)/ TIMESTEP;
  w = GetSpeed(jointnr); //reads the speed
  torquep = refddtheta + (wr - w)*Kv + (reftheta - fbtheta)*Kp; // calculates de torque based on a proportional gain.

  torque = getTorque(jointnr, torquep);
  if(abs(torque) > 4600.0) // do not send overload torque 
  {
    ntorque = 4600.0;
    if(torque< 0)
    {
      ntorque *= -1;
    }
    torque = ntorque;
  } 
//  Serial.print("the real torque:");
//  Serial.println(torque);
  SetTorque(jointnr, torque);
  //delay(2);
  //curr = GetCurrent(jointnr);
  //delay(2);
  //ppos =  GetPosition(jointnr);
  prevwr = wr;
  //long int oneloop = millis() - d;
  //Serial.print("one loop:");
  //Serial.println(oneloop);
  delay(2);
  curr = GetCurrent(jointnr);
  delay(2);
  ppos =  GetPosition(jointnr);
  omegar = wr;
  if( millis() - d > 20) //this executes every 20 miliseconds 
  {
    to_serial(zero, ppos, omegar, w, torque, curr); //send data 
    d = millis();
  }
}

//---------------------------------------------Function control negative direction

void control2(int jointnr)
{
  reftheta += TIMESTEP * prevwr;
  fbtheta = 0.001534 * (GetPosition(jointnr) - 2048);
  refddtheta = (wr2 - prevwr2)/ TIMESTEP;
//  Serial.print("refddtheta:");
//  Serial.println(refddtheta);
  w = GetSpeed(jointnr); //reads the speed
  torquep = refddtheta + (wr2 - w)*Kv + (reftheta - fbtheta)*Kp; // calculates de torque based on a proportional gain.
//  Serial.print("Control 2: torquep:");
//  Serial.println(torquep);
  torque = getTorque(jointnr, torquep);
  if(abs(torque) > 4600.0) // do not send overload torque 
  {
    ntorque = 4600.0;
    if(torque< 0)
    {
      ntorque *= -1;
    }
    torque = ntorque;
  } 
//  Serial.print("the real torque:");
//  Serial.println(torque);
  SetTorque(jointnr, torque);
//  delay(2);
//  curr = GetCurrent(jointnr);
//  delay(2);
//  ppos =  GetPosition(jointnr);
  prevwr2 = wr2;
    delay(2);
  curr = GetCurrent(jointnr);
  delay(2);
  ppos =  GetPosition(jointnr);
  omegar = wr2;
  if( millis() - d > 20) //this executes every 20 miliseconds 
  {
    to_serial(zero, ppos, omegar, w, torque, curr); //send data 
    d = millis();
  }
}

//----------------------------------------------------------Function get torque with compensator (alpha and beta)

float getTorque(int jointnr, float ddtheta) //, double *torque2, double *torque3)
{
        float ddtheta1;
        float ddtheta2;
        float ddtheta3;
        if(jointnr == 1)
        {
          ddtheta1 = ddtheta;
          ddtheta2 = 0;
          ddtheta3 = 0;
        }
        if(jointnr == 2)
        {
          ddtheta1 = 0;
          ddtheta2 = ddtheta;
          ddtheta3 = 0;
        }        
        if(jointnr == 3)
        {
          ddtheta1 = 0;
          ddtheta2 = 0;
          ddtheta3 = ddtheta;
        }

        double theta1 = 0.001534 * (GetPosition(1) - 2048);
        double theta2 = 0.001534 * (GetPosition(2) - 2048);
        double theta3 = 0.001534 * (GetPosition(3) - 2048);
        double dtheta1 = GetSpeed(1);
        double dtheta2 = GetSpeed(2);
        double dtheta3 = GetSpeed(3);
        
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

        float torque1 = 100*( H11*ddtheta1 + H12*ddtheta2 + H13*ddtheta3 + C11*dtheta1 + C12*dtheta2 + C13*dtheta3 + G1) + 0.2 * ctsign(dtheta1);
        float torque2 = 100*( H21*ddtheta1 + H22*ddtheta2 + H23*ddtheta3 + C21*dtheta1 + C22*dtheta2 + C23*dtheta3 + G2) + 0.75 * ctsign(dtheta2);
        float torque3 = 100*( H31*ddtheta1 + H32*ddtheta2 + H33*ddtheta3 + C31*dtheta1 + C32*dtheta2 + C33*dtheta3 + G3) + 0.4 * ctsign(dtheta3);
//        Serial.print("torques:");
//        Serial.println(torque1);
//        Serial.println(torque2);
//        Serial.println(torque3);

        if(jointnr == 1)
        {
          return torque1;
        }
        if(jointnr == 2)
        {
          return torque2;
        }        
        else
        {
	  return torque3;
        }
        
//	*torque1 = H11*ddtheta1 + H12*ddtheta2 + H13*ddtheta3 + C11*dtheta1 + C12*dtheta2 + C13*dtheta3 + G1;// + 0.2 * ctsign(dtheta1);
//	*torque2 = H21*ddtheta1 + H22*ddtheta2 + H23*ddtheta3 + C21*dtheta1 + C22*dtheta2 + C23*dtheta3 + G2;// + 0.75 * ctsign(dtheta2);
//	*torque3 = H31*ddtheta1 + H32*ddtheta2 + H33*ddtheta3 + C31*dtheta1 + C32*dtheta2 + C33*dtheta3 + G3;// + 0.4 * ctsign(dtheta3);

}

//---------------------------------------------------------------Function for Coulombs friction

double ctsign(double value) {
	if (value <= 0.001 || value >= -0.001) return 0;
	else return value / (abs(value));
}

//---------------------------------------------------------------Function for printing serial data to MATLAB

void to_serial(float posr, float pos, float spedr, float sped, float torr, float tor) //sends the data through serial 
{
  Serial.print(posr);
  Serial.print(",");
  Serial.print(pos);
  Serial.print(",");
  Serial.print(spedr);
  Serial.print(",");
  Serial.print(sped);
  Serial.print(",");
  Serial.print(torr);
  Serial.print(",");
  Serial.println(tor);

  //delay(15);
}
