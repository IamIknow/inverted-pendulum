function [c, ceq] = pendulumConstraintFCN(u,x,Ts,N)

% Inputs:
%   u:      optimization variable, from time k to time k+N-1 
%   x:      current state at time k
%   Ts:     controller sample time
%   N:      prediction horizon
%
% Output:
%   c:      inequality constraints applied across prediction horizon
%   ceq:    equality constraints (empty)


% Range of cart positions
zMin = -0.4;
zMax = 0.4;

c = zeros(N*2,1);
% Apply 2*N cart position constraints across prediction horizon, from time
% k+1 to k+N
xk = x;
uk = u(1);
for ct=1:N
    % obtain new cart position at next prediction step
    xk1 = pendulum_discrete(xk, uk, Ts);
    % -z + zMin < 0
    c(2*ct-1) = -xk1(1)+zMin;
    % z - zMax < 0
    c(2*ct) = xk1(1)-zMax;
    % update plant state and input for next step
    xk = xk1;
    if ct<N
        uk = u(ct+1);
    end
end
ceq = [];

