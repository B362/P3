%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%    The inverse dynamics of the 3-dof planar robotic arm (Crust Crawler)
%    
%    Made by Oskar and Guilherme
%    contact: shaoping bai, shb@m-tech.aau.dk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc

%%  %%%%%%%%%%%%%%%%%%%%%%%  motion profiles
%=======================  joint 1
A1 = 3; % magnitude
f1 = pi; % frequency

%=======================  joint 2
A2 = 0.5; % magnitude
f2 = 3*pi; % frequency

%=======================  joint 2
A3 = 0.2; % magnitude
f3 = 4*pi; % frequency

g = 9.801; % gravity constant

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%  link properties
%======================================  link 1
l1 = 0.235; % length [m]
d1 = 0.160; % mass center
m1 = 0.228; % mass [kg]
% moment of inertia
I1xx = 0; I1xy = 0; I1xz = 0;
I1yx = 0; I1yy = 0; I1yz = 0;
I1zx = 0; I1zy = 0; I1zz = 0;
%======================================  link 2
l2 = 0.305; % length [m]
d2 = 0.135; % mass center
m2 = 0.275; % mass [kg]
% moment of inertia
I2xx = 0; I2xy = 0; I2xz = 0;
I2yx = 0; I2yy = 0; I2yz = 0;
I2zx = 0; I2zy = 0; I2zz = 0;

%% %%%%%%%%%%%%%%%%%% dynamic simulation
%%%%%%%%%%%%%%%%%%% discrete time
T = 1; % second
N = 100; % resolution
i = 0; 
for t = linspace(0, T, N)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% instantaneous time
    i = i + 1; time(i) = t; 
    %%%%%%%%%%%%%%% Joint 1: angular displacement, velocity, acceleration
    theta1(i) = A1*sin(f1*t); dtheta1(i) = A1*f1*cos(f1*t); 
    ddtheta1(i) = -A1*f1^2*sin(f1*t); 
    
    %%%%%%%%%%%%%%% Joint 2: angular displacement, velocity, acceleration
    theta2(i) = A2*sin(f2*t+pi/4); dtheta2(i) = A2*f2*cos(f2*t+pi/4); %plus 45degrees
    ddtheta2(i) = -A2*f2^2*sin(f2*t+pi/4); %plus 45degrees
    
    %%%%%%%%%%%%%%% Joint 2: angular displacement, velocity, acceleration
    theta3(i) = A3*sin(f3*t+pi/4); dtheta3(i) = A3*f3*cos(f3*t+pi/4); %plus 45degrees
    ddtheta3(i) = -A3*f3^2*sin(f3*t+pi/4); %plus 45degrees
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%  coefficients of dynamic equation
     H11 = (1/2)*(2*I1zz+2*I2zz+d2.^2*m2+l1.^2*m2+l1.^2*m2*cos(2*theta2(i) ...
      )+2*d2*l1*m2*cos(theta3(i))+d2.^2*m2*cos(2*(theta2(i)+theta3(i)))+2* ...
      d2*l1*m2*cos(2*theta2(i)+theta3(i)));
     H12 = (1/2)*((-1)*(I1yz+I1zy+I2yz+ ...
      I2zy)*cos(theta1(i))+(I1xz+I1zx+I2xz+I2zx)*sin(theta1(i)));
     H13 = (1/2)*((-1)*(I2yz+I2zy)*cos(theta1(i))+(I2xz+I2zx)*sin(theta1(i)));

     H21 = (1/2)*((-1)*(I1yz+I1zy+I2yz+I2zy)*cos(theta1(i)) ...
         +(I1xz+I1zx+I2xz+I2zx)*sin(theta1(i)));
     H22 = (1/2)*(I1xx+I1yy+I2xx+I2yy+2*d2.^2*m2+2*l1.^2*m2+((-1)* ...
      I1xx+I1yy+(-1)*I2xx+I2yy)*cos(2*theta1(i))+4*d2*l1*m2*cos(theta3(i) ...
      )+(-1)*I1xy*sin(2*theta1(i))+(-1)*I1yx*sin(2*theta1(i))+(-1)*I2xy* ...
      sin(2*theta1(i))+(-1)*I2yx*sin(2*theta1(i)));
     H23 = (1/2)*(I2xx+I2yy+2* ...
      d2.^2*m2+((-1)*I2xx+I2yy)*cos(2*theta1(i))+2*d2*l1*m2*cos(theta3(i)) ...
      +(-1)*I2xy*sin(2*theta1(i))+(-1)*I2yx*sin(2*theta1(i)));

     H31 = (1/2)*((-1)*(I2yz+I2zy)*cos(theta1(i))+(I2xz+I2zx)*sin(theta1(i)));
     H32 = (1/2)*(I2xx+I2yy+2*d2.^2*m2+((-1)*I2xx+I2yy)*cos(2*theta1(i))+2*d2* ...
      l1*m2*cos(theta3(i))+(-1)*I2xy*sin(2*theta1(i))+(-1)*I2yx*sin(2*theta1(i)));
     H33 = (1/2)*(I2xx+I2yy+2*d2.^2*m2+((-1)*I2xx+I2yy)*cos(2* ...
      theta1(i))+(-1)*(I2xy+I2yx)*sin(2*theta1(i)));

     C11 = (-2)*m2*(l1*cos(theta2(i))+d2*cos(theta2(i)+theta3(i)))*((l1*sin(theta2(i)) ...
      +d2*sin(theta2(i)+theta3(i)))*dtheta2(i)+d2*sin(theta2(i)+theta3(i)) ...
      *dtheta3(i));
     C12 = (1/2)*((I1xy+I1yx+I2xy+I2yx)*cos(2*theta1(i) ...
      )+((-1)*I1xx+I1yy+(-1)*I2xx+I2yy)*sin(2*theta1(i)))*dtheta2(i);
     C13 = (1/2)*((I2xy+I2yx)*cos(2*theta1(i))+((-1)*I2xx+I2yy)* ...
      sin(2*theta1(i)))*(2*dtheta2(i)+dtheta3(i));

     C21 = (1/2)*((I1xz+I1zx+I2xz+I2zx)*cos(theta1(i))+(I1yz+I1zy+I2yz+I2zy)* ...
      sin(theta1(i))+m2*(l1.^2*sin(2*theta2(i))+d2.^2*sin(2*(theta2(i)+theta3(i)))+ ...
      2*d2*l1*sin(2*theta2(i)+theta3(i))))*dtheta1(i)+(-1)*(( ...
      I1xy+I1yx+I2xy+I2yx)*cos(2*theta1(i))+((-1)*I1xx+I1yy+(-1)*I2xx+ ...
      I2yy)*sin(2*theta1(i)))*dtheta2(i)+(-1)*((I2xy+I2yx)* ...
      cos(2*theta1(i))+((-1)*I2xx+I2yy)*sin(2*theta1(i)))*dtheta3(i);
     C22 = (-2)*d2*l1*m2*sin(theta3(i))*dtheta3(i);
     C23 = (-1)*d2*l1*m2*sin(theta3(i))*dtheta3(i);

     C31 = (1/2)*((I2xz+I2zx)*cos( ...
      theta1(i))+(I2yz+I2zy)*sin(theta1(i))+2*d2*m2*(l1*cos(theta2(i))+d2*cos( ...
      theta2(i)+theta3(i)))*sin(theta2(i)+theta3(i)))*dtheta1(i)+(-1)*(( ...
      I2xy+I2yx)*cos(2*theta1(i))+((-1)*I2xx+I2yy)*sin(2*theta1(i)))*( ...
      dtheta2(i)+dtheta3(i));
     C32 = d2*l1*m2*sin(theta3(i))*dtheta2(i);
     C33 = 0;

     G1 = 0;
     G2 = g*((d1*m1+l1*m2)*cos(theta2(i))+d2*m2*cos(theta2(i)+theta3(i)));
     G3 = d2*g*m2*cos(theta2(i)+theta3(i));
         
     F1 = 4.65357*dtheta1;
     F2 = 2.09525*dtheta2;
     F3 = 4.65357*dtheta3;
     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  actuator torques
    tau_1(i) = H11*ddtheta1(i) + H12*ddtheta2(i) + H13*ddtheta3(i) + ...
        C11*dtheta1(i) + C12*dtheta2(i) + C13*dtheta3(i) + G1 + F1;
    tau_2(i) = H21*ddtheta1(i) + H22*ddtheta2(i) + H23*ddtheta3(i) + ...
        C21*dtheta1(i) + C22*dtheta2(i) + C23*dtheta3(i) + G2 + F2;
    tau_3(i) = H31*ddtheta1(i) + H32*ddtheta2(i) + H33*ddtheta3(i) + ...
        C31*dtheta1(i) + C32*dtheta2(i) + C33*dtheta3(i) + G3 + F3;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  link positions
    %%%%%%%%%%%%%%%%%% link 1
    X1(i) = l1*cos(theta1(i))*cos(theta2(i));
    Y1(i) = l1*sin(theta1(i))*cos(theta2(i));
    Z1(i) = l1*sin(theta2(i));
   % x1(i) = 0.5*X1(i); y1(i) = 0.5*Y1(i); 
    %%%%%%%%%%%%%%%%%% link 2
    X2(i) = X1(i) + l2*cos(theta2(i) + theta3(i))*cos(theta1(i)); 
    Y2(i) = Y1(i) + l2*cos(theta2(i) + theta3(i))*sin(theta1(i));
    Z2(i) = Z1(i) + l2*sin(theta2(i) + theta3(i));
    %x2(i) = X1(i) + 0.5*X2(i); %Y2(i) = Y1(i) + 0.5*Y2(i); 
    
end

torque_1 = [time; tau_1]'; 
torque_2 = [time; tau_2]'; 
torque_3 = [time; tau_3]';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot the motion profiles
figure(2)
clf
figure(2)
subplot(3, 1, 1)
hold on
plot(time, theta1, 'b')
plot(time, theta2, 'r')
plot(time, theta3, 'g')
hold off
legend('theta 1', 'theta 2', 'theta 3')
grid on; 
xlabel('time [sec]'); ylabel('angular displacement [rad]'); 
subplot(3, 1, 2)
hold on
plot(time, dtheta1, 'b')
plot(time, dtheta2, 'r')
plot(time, dtheta3, 'g')
hold off
grid on; 
xlabel('time [sec]'); ylabel('angular velocity [rad/s]'); 
subplot(3, 1, 3)
hold on
plot(time, ddtheta1, 'b')
plot(time, ddtheta2, 'r')
plot(time, ddtheta3, 'g')
hold off
grid on; 
xlabel('time [sec]'); ylabel('angular acceleration [rad/s^2]'); 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot the motor torques
figure(1)
clf
figure(1)
hold on
plot(time, tau_1, 'b')
plot(time, tau_2, 'r')
plot(time, tau_3, 'g')
hold off
legend(' joint 1', 'joint 2', 'joint 3')
grid on; 
xlabel('time [sec]'); ylabel('torques [Nm/rad]'); 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  plot the link
figure(3)
clf
figure(3)
hold on 
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
set(gca,'NextPlot','replaceChildren');
for j = 1 : N
    plot3(0, 0, 0, 'ko', 'MarkerSize', 6, 'Linewidth', 2);
    axis equal; grid on; axis([-1 1 -1 1 -1 1]);
    plot3(X1(j), Y1(j), Z1(j), 'bo', 'MarkerSize', 6, 'Linewidth', 2);
    plot3(X2(j), Y2(j), Z2(j), 'ro', 'MarkerSize', 6, 'Linewidth', 2);
    line([0 X1(j)], [0 Y1(j)], [0 Z1(j)], 'Color', 'b', 'Linewidth', 2); 
    line([X1(j) X2(j)], [Y1(j) Y2(j)], [Z1(j) Z2(j)], 'Color', 'r', 'Linewidth', 2);  
    F(j) = getframe; 
end
hold off
movie(F); 
