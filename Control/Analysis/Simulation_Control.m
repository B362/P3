% simulation of two link robot
clear

%%%%%%%%%%%%%%%%%%%%%%%%%%%%  link properties
%======================================  link 1
l1 = 0.235; % length [m]
lc1 = 0.160; % mass center
m1 = 0.228; % mass [kg]
% moment of inertia
I1xx = 0.00006781; I1xy = 0.00002608; I1xz = 0;
I1yx = 0.00002608; I1yy = 0.00631914; I1yz = 0;
I1zx = 0; I1zy = 0; I1zz = 0.00630459;
%======================================  link 2
l2 = 0.305; % length [m]
lc2 = 0.135; % mass center
m2 = 0.275; % mass [kg]
% moment of inertia
I2xx = 0.00012618; I2xy = 0.00001090; I2xz = 0;
I2yx = 0.00001090; I2yy = 0.00062333; I2yz = 0;
I2zx = 0; I2zy = 0; I2zz = 0.00055076;

I1 = 0;
I2 = 0;

g = 9.82;


%kp1 = 25; kv1 = 10; kp2 = 25; kv2 = 10;
kp1 = 100; kv1 = 20; kp2 = 100; kv2 = 20; kp3 = 100; kv3 = 20;

% 1) simulation of the effect of compensating V and G
% 
dt = 0.001;

% definition of trajectory  
am1 = 5; am2 = 10; am3 = 10; % max angular accelerations
vm1 = 0.5; vm2 = 1; vm3 = 1;% max angular velocities
pa1 = vm1^2/am1/2; pa2 = vm2^2/am2/2; pa3 = vm3^2/am3/2; %acc distance
na1 = ceil(vm1/am1/dt); na2 = ceil(vm2/am2/dt); na3 = ceil(vm3/am3/dt); %acc samples
ta1 = na1*dt, ta2 = na2*dt; ta3 = na3*dt;
ac1 = vm1/na1/dt; ac1r = ones(1,na1)*ac1;
timear1 = 0:dt:dt*(length(ac1r)-1); 
discrete_integration = tf(dt,[1 -1],dt);
va1r = lsim(discrete_integration,ac1r,timear1);
pa1r = lsim(discrete_integration,va1r,timear1);
ac2 = vm2/na2/dt; ac2r = ones(1,na2)* ac2;
timear2 = 0:dt:dt*(length(ac2r)-1);
va2r = lsim(discrete_integration,ac2r,timear2);
pa2r = lsim(discrete_integration,va2r,timear2);
ac3 = vm3/na3/dt; ac3r = ones(1,na3)* ac3;
timear3 = 0:dt:dt*(length(ac3r)-1);
va3r = lsim(discrete_integration,ac3r,timear3);
pa3r = lsim(discrete_integration,va3r,timear3);

% definition of way points a, b, c, d and e
th1a = pi/4; th1b = 0; th1c = 0; th1d = pi/4; th1e = pi/4;
th2a = 0; th2b = 0; th2c = pi/2; th2d = pi/2; th2e = 0;
th3a = 0; th3b = 0; th3c = 0; th3d = 0; th3e = 0;
tab = (abs(th1b-th1a)-2*pa1)/vm1; nab=ceil(tab/dt); % time a->b
acc1ref_ab = [-ac1r 0*ones(1,nab-2*length(ac1r)) ac1r];
acc2ref_ab = [zeros(1,nab)];
tbc = (abs(th2c-th2b)-2*pa2)/vm2; nbc = ceil(tbc/dt); % time b->c
acc1ref_bc = [zeros(1,nbc)];
acc2ref_bc = [ac2r 0*ones(1,nbc-2*length(ac2r)) -ac2r];
tcd = (abs(th1d-th1c)-2*pa1)/vm1; ncd=ceil(tcd/dt); % tid c->d
acc1ref_cd = [ac1r 0*ones(1,ncd-2*length(ac1r)) -ac1r];
acc2ref_cd = [zeros(1,ncd)];
tde = (abs(th2e-th2d)-2*pa2)/vm2; nde = ceil(tde/dt); % tid b->c
acc1ref_de = [zeros(1,nde)];
acc2ref_de = [-ac2r 0*ones(1,nde-2*length(ac2r)) ac2r];
time = 0:dt:(nab+nbc+ncd+nde-1)*dt

acc1ref = [acc1ref_ab acc1ref_bc acc1ref_cd acc1ref_de];
omega1ref = lsim(discrete_integration,acc1ref,time);
theta1ref = lsim(discrete_integration,omega1ref,time)+th1a;
% figure(3), plot(time,acc1ref)
% figure(2), plot(time,omega1ref)
% figure(1), plot(time,theta1ref)
acc2ref = [acc2ref_ab acc2ref_bc acc2ref_cd acc2ref_de]
omega2ref = lsim(discrete_integration,acc2ref,time);
theta2ref = lsim(discrete_integration,omega2ref,time,th2a);
% figure(6), plot(time,acc2ref)
% figure(5), plot(time,omega2ref)
% figure(4), plot(time,theta2ref)
acc3ref = [acc2ref_ab acc2ref_bc acc2ref_cd acc2ref_de]
omega3ref = lsim(discrete_integration,acc3ref,time);
theta3ref = lsim(discrete_integration,omega3ref,time,th3a);

% Initial conditions
theta1o(1) = pi/4; dtheta1o(1) = 0; ddtheta1o(1) = 0;
theta2o(1) = 0; dtheta2o(1) = 0; ddtheta2o(1) = 0;
theta1(1) = pi/4; dtheta1(1) = 0; ddtheta1(1) = 0;
theta2(1) = 0; dtheta2(1) = 0; ddtheta2(1) = 0;
theta3(1) = 0; dtheta3(1) = 0; ddtheta3(1) = 0;

Ni = length(time)

% simulation with computed torque
for ii = 1:Ni-1
 H11 = (1/2)*(2*I1zz+2*I2zz+lc2.^2*m2+l1.^2*m2+l1.^2*m2*cos(2*theta2(ii) ...
  )+2*lc2*l1*m2*cos(theta3(ii))+lc2.^2*m2*cos(2*(theta2(ii)+theta3(ii)))+2* ...
  lc2*l1*m2*cos(2*theta2(ii)+theta3(ii)));
 H12 = (1/2)*((-1)*(I1yz+I1zy+I2yz+ ...
  I2zy)*cos(theta1(ii))+(I1xz+I1zx+I2xz+I2zx)*sin(theta1(ii)));
 H13 = (1/2)*((-1)*(I2yz+I2zy)*cos(theta1(ii))+(I2xz+I2zx)*sin(theta1(ii)));

 H21 = (1/2)*((-1)*(I1yz+I1zy+I2yz+I2zy)*cos(theta1(ii)) ...
     +(I1xz+I1zx+I2xz+I2zx)*sin(theta1(ii)));
 H22 = (1/2)*(I1xx+I1yy+I2xx+I2yy+2*lc2.^2*m2+2*l1.^2*m2+((-1)* ...
  I1xx+I1yy+(-1)*I2xx+I2yy)*cos(2*theta1(ii))+4*lc2*l1*m2*cos(theta3(ii) ...
  )+(-1)*I1xy*sin(2*theta1(ii))+(-1)*I1yx*sin(2*theta1(ii))+(-1)*I2xy* ...
  sin(2*theta1(ii))+(-1)*I2yx*sin(2*theta1(ii)));
 H23 = (1/2)*(I2xx+I2yy+2* ...
  lc2.^2*m2+((-1)*I2xx+I2yy)*cos(2*theta1(ii))+2*lc2*l1*m2*cos(theta3(ii)) ...
  +(-1)*I2xy*sin(2*theta1(ii))+(-1)*I2yx*sin(2*theta1(ii)));

 H31 = (1/2)*((-1)*(I2yz+I2zy)*cos(theta1(ii))+(I2xz+I2zx)*sin(theta1(ii)));
 H32 = (1/2)*(I2xx+I2yy+2*lc2.^2*m2+((-1)*I2xx+I2yy)*cos(2*theta1(ii))+2*lc2* ...
  l1*m2*cos(theta3(ii))+(-1)*I2xy*sin(2*theta1(ii))+(-1)*I2yx*sin(2*theta1(ii)));
 H33 = (1/2)*(I2xx+I2yy+2*lc2.^2*m2+((-1)*I2xx+I2yy)*cos(2* ...
  theta1(ii))+(-1)*(I2xy+I2yx)*sin(2*theta1(ii)));

 C11 = (-2)*m2*(l1*cos(theta2(ii))+lc2*cos(theta2(ii)+theta3(ii)))*((l1*sin(theta2(ii)) ...
  +lc2*sin(theta2(ii)+theta3(ii)))*dtheta2(ii)+lc2*sin(theta2(ii)+theta3(ii)) ...
  *dtheta3(ii));
 C12 = (1/2)*((I1xy+I1yx+I2xy+I2yx)*cos(2*theta1(ii) ...
  )+((-1)*I1xx+I1yy+(-1)*I2xx+I2yy)*sin(2*theta1(ii)))*dtheta2(ii);
 C13 = (1/2)*((I2xy+I2yx)*cos(2*theta1(ii))+((-1)*I2xx+I2yy)* ...
  sin(2*theta1(ii)))*(2*dtheta2(ii)+dtheta3(ii));

 C21 = (1/2)*((I1xz+I1zx+I2xz+I2zx)*cos(theta1(ii))+(I1yz+I1zy+I2yz+I2zy)* ...
  sin(theta1(ii))+m2*(l1.^2*sin(2*theta2(ii))+lc2.^2*sin(2*(theta2(ii)+theta3(ii)))+ ...
  2*lc2*l1*sin(2*theta2(ii)+theta3(ii))))*dtheta1(ii)+(-1)*(( ...
  I1xy+I1yx+I2xy+I2yx)*cos(2*theta1(ii))+((-1)*I1xx+I1yy+(-1)*I2xx+ ...
  I2yy)*sin(2*theta1(ii)))*dtheta2(ii)+(-1)*((I2xy+I2yx)* ...
  cos(2*theta1(ii))+((-1)*I2xx+I2yy)*sin(2*theta1(ii)))*dtheta3(ii);
 C22 = (-2)*lc2*l1*m2*sin(theta3(ii))*dtheta3(ii);
 C23 = (-1)*lc2*l1*m2*sin(theta3(ii))*dtheta3(ii);

 C31 = (1/2)*((I2xz+I2zx)*cos( ...
  theta1(ii))+(I2yz+I2zy)*sin(theta1(ii))+2*lc2*m2*(l1*cos(theta2(ii))+lc2*cos( ...
  theta2(ii)+theta3(ii)))*sin(theta2(ii)+theta3(ii)))*dtheta1(ii)+(-1)*(( ...
  I2xy+I2yx)*cos(2*theta1(ii))+((-1)*I2xx+I2yy)*sin(2*theta1(ii)))*( ...
  dtheta2(ii)+dtheta3(ii));
 C32 = lc2*l1*m2*sin(theta3(ii))*dtheta2(ii);
 C33 = 0;
 
 F1 = 0.25*2*(1/(1 + exp(-2*dtheta1(ii))) - 0.5) + (0.005*(dtheta1(ii))); 
 F2 = 0.685*2*(1/(1 + exp(-2*dtheta2(ii))) - 0.5) + (0.02*(dtheta2(ii)));
 F3 = 0.39*2*(1/(1 + exp(-5*dtheta3(ii))) - 0.5) + (0.008*(dtheta3(ii)));

M = [H11 H12 H13; H21 H22 H23; H31 H32 H33];
alpha = [H11 H12 H13; H21 H22 H23; H31 H32 H33];
G1 = 0;
G2 = g*((lc1*m1+l1*m2)*cos(theta2(ii))+lc2*m2*cos(theta2(ii)+theta3(ii)));
G3 = lc2*g*m2*cos(theta2(ii)+theta3(ii));   


beta = [C11 C12 C13; C21 C22 C23; C31 C32 C33]+[G1;G2;G3]+[F1; F2; F3];
taup1 = kp1*(theta1ref(ii)-theta1(ii))-kv1*dtheta1(ii)+kv1*omega1ref(ii)+acc1ref(ii);
taup2 = kp2*(theta2ref(ii)-theta2(ii))-kv2*dtheta2(ii)+kv2*omega2ref(ii)+acc2ref(ii);
taup3 = kp3*(theta3ref(ii)-theta3(ii))-kv3*dtheta3(ii)+kv3*omega3ref(ii)+acc3ref(ii);
tau = alpha*[taup1;taup2;taup3];%+beta; %With or without the compensator
%simulation step
theta1(ii+1) = theta1(ii)+dt*dtheta1(ii);
theta2(ii+1) = theta2(ii)+dt*dtheta2(ii);
theta3(ii+1) = theta3(ii)+dt*dtheta3(ii);
acc = alpha\(tau-beta);
ddtheta1(ii) = acc(1);
ddtheta2(ii) = acc(2);
ddtheta3(ii) = acc(3);
dtheta1(ii+1) = dtheta1(ii)+dt*acc(1);
dtheta2(ii+1) = dtheta2(ii)+dt*acc(2);
dtheta3(ii+1) = dtheta3(ii)+dt*acc(3);
end





time = dt*(0:length(theta1)-1);
figure(1), plot(time,theta1ref,time,theta1)
xlabel('time [sec]')
ylabel('\theta_1 [rad]')
legend('Reference','Computed torque')
figure(2), plot(time,omega1ref,time,dtheta1)
xlabel('time [sec]')
ylabel('\omega_1 [rad]')
legend('Reference','Computed torque')
figure(3), plot(time,acc1ref,time(1:length(ddtheta1)),ddtheta1)
xlabel('time [sec]')
ylabel('\alpha_1 [rad]')
legend('Reference','Computed torque')
figure(4), plot(time,theta2ref,time,theta2)
xlabel('time [sec]')
ylabel('\theta_2 [rad]')
legend('Reference','Computed torque')
figure(5), plot(time,omega2ref,time,dtheta2)
xlabel('time [sec]')
ylabel('\omega_2 [rad]')
legend('Reference','Computed torque')
figure(6), plot(time,acc2ref,time(1:length(ddtheta2)),ddtheta2)
xlabel('time [sec]')
ylabel('\alpha_1 [rad]')
legend('Reference','Computed torque')
figure(7), plot(time,theta3ref,time,theta3)
xlabel('time [sec]')
ylabel('\theta_3 [rad]')
legend('Reference','Computed torque')
figure(8), plot(time,omega3ref,time,dtheta3)
xlabel('time [sec]')
ylabel('\omega_3 [rad]')
legend('Reference','Computed torque')
figure(9), plot(time,acc3ref,time(1:length(ddtheta3)),ddtheta3)
xlabel('time [sec]')
ylabel('\alpha_3 [rad]')
legend('Reference','Computed torque')
