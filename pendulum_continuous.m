function [dxdt, y] = pendulum_continuous(x, u)
% Continuous model of a system
% x represents state vector:
% x(1) - position of the cart
% x(2) - velocity of the cart
% x(3) - pendulum's angle
% x(4) - pendulum's angular velocity

% system parameters
M = 0.3; % mass of the cart
m = 0.2; % mass of the pendulum
g = 9.81;   % gravity constant
l = 0.3;    % length of the pendulum
b = 0.1;    % cart friction coefficient
c = 0.1;    % pendulum's friction coefficient

x_dot = x(2);
phi = x(3);
phi_dot = x(4);
F = u;
y = x;

dxdt = x;
dxdt(1) = x_dot;
% TODO change signs if smthng is not right
% dxdt(2) = (F - b*x_dot + c*phi_dot*cos(phi)/l - m*l*phi_dot^2*sin(phi) + m*g*sin(phi)*cos(phi)) / (M + m*sin(phi)^2);
  dxdt(2) = (F - b*x_dot - m*l*phi_dot^2*sin(phi) + m*g*sin(phi)*cos(phi)) / (m + M*sin(phi)^2);

dxdt(3) = phi_dot;
% dxdt(4) = (-m^2*l^2*phi_dot^2*sin(phi)*cos(phi) + m*l*x_dot*b*cos(phi) - (M+m)*(c*phi_dot + m*g*l*sin(phi)) - m*l*F*cos(phi)) ...
%    / (m*l^2*(M + m*(sin(phi))^2));
 dxdt(4) = ((F - b*x_dot - m*l*phi_dot^2*sin(phi))*cos(phi)/(m + M) + g*sin(phi)) / (l - m*l*cos(phi)^2/(m + M));

end

