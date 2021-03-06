Ts = 0.05;           % Sampling time

x = [0;0;-pi;0];    % Initial condition (lower equilibrium)
xref = [0;0;0;0];   % Reference state (upper equilibrium)
y = [x(1); x(3)];   % Measured states

duration = 20;

prediction_horizon = 10;

uopt = zeros(prediction_horizon, 1);

options = optimoptions('fmincon','Algorithm','sqp','Display','none');

LB = -50*ones(prediction_horizon,1);
UB = 50*ones(prediction_horizon,1);

EKF = extendedKalmanFilter(@pendulumStateFcn, @pendulumMeasurementFcn);
EKF.State = x;

sim_data = x;
for i = 1 : (duration / Ts)
    % Correct previous measured state
    xk = correct(EKF, y);
    
    % Solve optimization problem
    COSTFUN = @(u) pendulumObjectiveFCN(u,xk,Ts,prediction_horizon,xref,uopt(1));
    CONSFUN = @(u) pendulumConstraintFCN(u,xk,Ts,prediction_horizon);
    uopt = fmincon(COSTFUN,uopt,[],[],[],[],LB,UB,CONSFUN,options);
    
    % Predict next state
    predict(EKF, [uopt(1); Ts]);
    
    % Compute states with applied optimal control
    x = pendulum_discrete(x, uopt(1), Ts);
    
    % Sensor noise
    y = x([1 3]) + randn(2,1)*0.01;
    
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
