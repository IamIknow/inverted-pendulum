function [xk1, yk] = pendulum_discrete(xk, uk, Ts)
% Discretized model

M = 10;
delta = Ts/M;
xk1 = xk;
for ct = 1 : M
   xk1 = xk1 + delta*pendulum_continuous(xk1, uk); 
end
yk = xk;

end
