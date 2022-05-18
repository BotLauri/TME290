% 2a) Robot simulation. 
% Derive a general expression for the position (x,y) and heading phi for
% the robot as a function of time and plot the resulting tajectory in your
% report. 
clear
tic

% Initialization. 
T = linspace(0, 10);
x = zeros(1, length(T)); 
y = zeros(1, length(T)); 
phi = zeros(1, length(T));
R = 0.12; 
v_0 = 0.5; 
t_1 = 10; 
t_2 = 5;
A = v_0*(t_1 + t_2)/(2*t_1*t_2);
B = v_0*(t_1 - t_2)/(2*t_1*t_2*R);

% Main loop. 
for i = 1:length(T)
    t = T(i);
    x(i) = A/B*(sin(B*t^2/2));
    y(i) = A/B*(cos(B*t^2/2)) + A/B;
    phi(i) = B*t^2/2;
end

% Plots.
hold on
plot(x, y)
xlabel('x-coord')
ylabel('y-coord')
title('Plot of the resulting trajectory.')
axis equal
hold off

toc
