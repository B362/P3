function y = InertiaFunc(u)

%% joint velocity and displacement
ddtheta1 = u(1); theta1 = u(4); 
ddtheta2 = u(2); theta2 = u(5); 
ddtheta3 = u(3); theta3 = u(6);

g = 9.801; % gravity constant

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%  link properties
%======================================  link 1
l1 = 0.235; % length [m]
d1 = 0.065; % mass center
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

%% %%%%%%%%%%%%%%%%%%%%%%%%%  coefficients of dynamic equation
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

  %% Tau line calculation
  
    taul1 = H11*ddtheta1 + H12*ddtheta2 + H13*ddtheta3;
    taul2 = H21*ddtheta1 + H22*ddtheta2 + H23*ddtheta3;
    taul3 = H31*ddtheta1 + H32*ddtheta2 + H33*ddtheta3;
    
y = [taul1; taul2; taul3];