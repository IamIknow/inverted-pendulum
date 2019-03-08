function [dxdt, y] = pendulum_continuous(x, u)
% Continuous model of a system
% x represents state vector:
% x(1) - position of the cart
% x(2) - velocity of the cart
% x(3) - pendulum's angle
% x(4) - pendulum's angular velocity

% system parameters
m = 0.3;  % cart mass
M = 0.6;  % pendulum mass
g = 9.81;   % gravity constant
l = 0.40;    % pendulum length
b = 1;    % cart damping
c = 0.019;

x_dot = x(2);
phi = x(3);
phi_dot = x(4);
F = u;
y = x;

dxdt = x;
dxdt(1) = x_dot;
 dxdt(2) = (F - b*x_dot + c*phi_dot*cos(phi)/l - m*l*phi_dot^2*sin(phi) - m*g*sin(phi)*cos(phi)) / (M + m*sin(phi)^2);

dxdt(3) = phi_dot;
 dxdt(4) = (-m^2*l^2*phi_dot^2*sin(phi)*cos(phi) + m*l*x_dot*b*cos(phi) - (M+m)*(c*phi_dot + m*g*l*sin(phi)) - m*l*F*cos(phi)) ...
    / (m*l^2*(M + m*(sin(phi))^2));

end

