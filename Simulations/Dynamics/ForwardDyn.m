function y = ForwardDyn(u)

%%   motor torques
tau1 = u(1); tau2 = u(2); tau3 = u(3);

%% joint velocity and displacement
dtheta1 = u(4); theta1 = u(7); 
dtheta2 = u(5); theta2 = u(8); 
dtheta3 = u(6); theta3 = u(9); 

g = 9.801; % gravity constant

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%  link properties
%======================================  link 1
l1 = 0.235; % length [m]
d1 = 0.16; % mass center
m1 = 0.228; % mass [kg]
% moment of inertia
I1xx = 0.00006781; I1xy = 0.00002608; I1xz = 0;
I1yx = 0.00002608; I1yy = 0.00631914; I1yz = 0;
I1zx = 0; I1zy = 0; I1zz = 0.00630459;
%======================================  link 2
l2 = 0.305; % length [m]
d2 = 0.135; % mass center
m2 = 0.275; % mass [kg]
% moment of inertia
I2xx = 0.00012618; I2xy = 0.00001090; I2xz = 0;
I2yx = 0.00001090; I2yy = 0.00062333; I2yz = 0;
I2zx = 0; I2zy = 0; I2zz = 0.00055076;

%% %%%%%%%%%%%%%%%%%% dynamic simulation

    %%%%%%%%%%%%%%%%%%%%%%%%%%%  coefficients of dynamic equation
     H11 = (1/2)*(2*I1zz+2*I2zz+d2.^2*m2+l1.^2*m2+l1.^2*m2*cos(2*theta2 ...
      )+2*d2*l1*m2*cos(theta3)+d2.^2*m2*cos(2*(theta2+theta3))+2* ...
      d2*l1*m2*cos(2*theta2+theta3));
     H12 = (1/2)*((-1)*(I1yz+I1zy+I2yz+ ...
      I2zy)*cos(theta1)+(I1xz+I1zx+I2xz+I2zx)*sin(theta1));
     H13 = (1/2)*((-1)*(I2yz+I2zy)*cos(theta1)+(I2xz+I2zx)*sin(theta1));

     H21 = (1/2)*((-1)*(I1yz+I1zy+I2yz+I2zy)*cos(theta1) ...
         +(I1xz+I1zx+I2xz+I2zx)*sin(theta1));
     H22 = (1/2)*(I1xx+I1yy+I2xx+I2yy+2*d2.^2*m2+2*l1.^2*m2+((-1)* ...
      I1xx+I1yy+(-1)*I2xx+I2yy)*cos(2*theta1)+4*d2*l1*m2*cos(theta3 ...
      )+(-1)*I1xy*sin(2*theta1)+(-1)*I1yx*sin(2*theta1)+(-1)*I2xy* ...
      sin(2*theta1)+(-1)*I2yx*sin(2*theta1));
     H23 = (1/2)*(I2xx+I2yy+2* ...
      d2.^2*m2+((-1)*I2xx+I2yy)*cos(2*theta1)+2*d2*l1*m2*cos(theta3) ...
      +(-1)*I2xy*sin(2*theta1)+(-1)*I2yx*sin(2*theta1));

     H31 = (1/2)*((-1)*(I2yz+I2zy)*cos(theta1)+(I2xz+I2zx)*sin(theta1));
     H32 = (1/2)*(I2xx+I2yy+2*d2.^2*m2+((-1)*I2xx+I2yy)*cos(2*theta1)+2*d2* ...
      l1*m2*cos(theta3)+(-1)*I2xy*sin(2*theta1)+(-1)*I2yx*sin(2*theta1));
     H33 = (1/2)*(I2xx+I2yy+2*d2.^2*m2+((-1)*I2xx+I2yy)*cos(2* ...
      theta1)+(-1)*(I2xy+I2yx)*sin(2*theta1));

     C11 = (-2)*m2*(l1*cos(theta2)+d2*cos(theta2+theta3))*((l1*sin(theta2) ...
      +d2*sin(theta2+theta3))*dtheta2+d2*sin(theta2+theta3) ...
      *dtheta3);
     C12 = (1/2)*((I1xy+I1yx+I2xy+I2yx)*cos(2*theta1 ...
      )+((-1)*I1xx+I1yy+(-1)*I2xx+I2yy)*sin(2*theta1))*dtheta2;
     C13 = (1/2)*((I2xy+I2yx)*cos(2*theta1)+((-1)*I2xx+I2yy)* ...
      sin(2*theta1))*(2*dtheta2+dtheta3);

     C21 = (1/2)*((I1xz+I1zx+I2xz+I2zx)*cos(theta1)+(I1yz+I1zy+I2yz+I2zy)* ...
      sin(theta1)+m2*(l1.^2*sin(2*theta2)+d2.^2*sin(2*(theta2+theta3))+ ...
      2*d2*l1*sin(2*theta2+theta3)))*dtheta1+(-1)*(( ...
      I1xy+I1yx+I2xy+I2yx)*cos(2*theta1)+((-1)*I1xx+I1yy+(-1)*I2xx+ ...
      I2yy)*sin(2*theta1))*dtheta2+(-1)*((I2xy+I2yx)* ...
      cos(2*theta1)+((-1)*I2xx+I2yy)*sin(2*theta1))*dtheta3;
     C22 = (-2)*d2*l1*m2*sin(theta3)*dtheta3;
     C23 = (-1)*d2*l1*m2*sin(theta3)*dtheta3;

     C31 = (1/2)*((I2xz+I2zx)*cos( ...
      theta1)+(I2yz+I2zy)*sin(theta1)+2*d2*m2*(l1*cos(theta2)+d2*cos( ...
      theta2+theta3))*sin(theta2+theta3))*dtheta1+(-1)*(( ...
      I2xy+I2yx)*cos(2*theta1)+((-1)*I2xx+I2yy)*sin(2*theta1))*( ...
      dtheta2+dtheta3);
     C32 = d2*l1*m2*sin(theta3)*dtheta2;
     C33 = 0;

     G1 = 0;
     G2 = g*((d1*m1+l1*m2)*cos(theta2)+d2*m2*cos(theta2+theta3));
     G3 = d2*g*m2*cos(theta2+theta3);    
     
     F1 = 4.65357*dtheta1;
     F2 = 2.09525*dtheta2;
     F3 = 4.65357*dtheta3;
    
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  actuator torques
%     tau_1 = H11*ddtheta_1 + H12*ddtheta_2 - h*(dtheta_2)^2 ...
%         -2*h*dtheta_1*dtheta_2 + G1;
%     tau_2 = H22*ddtheta_2 + H12*ddtheta_1 - h*(dtheta_1)^2 + G2;
    
    M = [H11 H12 H13; H21 H22 H23; H31 H32 H33]; 
    
    torque1 = tau1 - (C11*dtheta1 + C12*dtheta2 + C13*dtheta3) - G1 - F1;
    torque2 = tau2 - (C21*dtheta1 + C22*dtheta2 + C23*dtheta3) - G2 - F2;
    torque3 = tau3 - (C31*dtheta1 + C32*dtheta2 + C33*dtheta3) - G3 - F3;
    
    tvector = [torque1; torque2; torque3];
    
    y = M\tvector; 
