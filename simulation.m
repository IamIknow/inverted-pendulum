Ts = 0.1;           % Sampling time

x = [-0.4;0;-pi;0];    % Initial condition (lower equilibrium)
xref = [0;0;0;0];  % Reference state (upper equilibrium)

duration = 20;

prediction_horizon = 10;

uopt = zeros(prediction_horizon, 1);

options = optimoptions('fmincon','Algorithm','sqp','Display','none');

LB = -25*ones(prediction_horizon,1);
UB = 25*ones(prediction_horizon,1);

sim_data = x;
for i = 1 : (duration / Ts)
    COSTFUN = @(u) pendulumObjectiveFCN(u,x,Ts,prediction_horizon,xref,uopt(1));
    CONSFUN = @(u) pendulumConstraintFCN(u,x,Ts,prediction_horizon);
    uopt = fmincon(COSTFUN,uopt,[],[],[],[],LB,UB,CONSFUN,options);
    
    x = pendulum_discrete(x, uopt(1), Ts);
    sim_data = [sim_data x]; %#ok<AGROW>
end

figure;
subplot(2,2,1);
plot(0:Ts:duration,sim_data(1,:));
xlabel('time');
ylabel('x');
title('cart position');
subplot(2,2,2);
plot(0:Ts:duration,sim_data(2,:));
xlabel('time');
ylabel("x'");
title('cart velocity');
subplot(2,2,3);
plot(0:Ts:duration,sim_data(3,:));
xlabel('time');
ylabel('phi');
title('pendulum angle');
subplot(2,2,4);
plot(0:Ts:duration,sim_data(4,:));
xlabel('time');
ylabel("phi'");
title('pendulum velocity');
