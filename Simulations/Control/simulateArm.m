function simulateArm(u)

%% Joint angles
theta1 = u(1); theta2 = u(2); theta3 = u(3);

%% Link properties

l1 = 0.235; % length [m]
l2 = 0.305; % length [m]

%% Transformation equations

% Link 1

X1 = l1 * cos(theta1) * cos(theta2);
Y1 = l1 * sin(theta1) * cos(theta2);
Z1 = l1 * sin(theta2);

% Link 2

X2 = l2 * cos(theta2 + theta3) * cos(theta1) + X1;
Y2 = l2 * cos(theta2 + theta3) * sin(theta1) + Y1;
Z2 = l2 * sin(theta2 + theta3) + Z1; 

%% Plot

figure(1)
clf
figure(1)
%hold on 
axis equal; grid on; axis([0 1 0 1 0 1]); 
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
set(gca,'NextPlot','replaceChildren');
%for j = 1 : N  
    plot3(0, 0, 0, 'ko', 'MarkerSize', 6, 'Linewidth', 2)  
    plot3(X1, Y1, Z1, 'bo', 'MarkerSize', 6, 'Linewidth', 2)
    plot3(X2, Y2, Z2, 'ro', 'MarkerSize', 6, 'Linewidth', 2)
    line([0 X1], [0 Y1], [0 Z2], 'Color', 'b', 'Linewidth', 2) 
    line([X1 X2], [Y1 Y2], [Z1 Z2], 'Color', 'r', 'Linewidth', 2)  
%    F(j) = getframe; 
%end
%hold off
%movie(F); 