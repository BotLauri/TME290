% 2a) Robot simulation. 
% Derive a general expression for the position (x,y) and heading phi for
% the robot as a function of time and plot the resulting tajectory in your
% report. 
clear

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
B = v_0*(t_2 - t_1)/(2*t_1*t_2*R);

% Main loop. 
for t = 2:length(T)
    x(t) = x(t-1) + A*(t*sin(t) + cos(t));
    y(t) = y(t-1) + A*(sin(t) - t*cos(t));
    phi(t) = phi(t-1) + B*t^2/2;
end

% Plots.
hold on
plot(x, y)
xlabel('x-coord')
ylabel('y-coord')
title('Plot of the resulting trajectory.')
axis equal
hold off
