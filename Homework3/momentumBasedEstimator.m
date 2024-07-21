clear 
close all
clc

%% Quadrotor data
data = load('ws_homework_3_2024.mat');

% Model parameters
m = 1.5; % kg
Ib = diag([1.2416, 1.2416, 2*1.2416]);
g = 9.81; % m/s^2
e3 = [0; 0; 1];

% Measurement data
eta = data.attitude.signals.values;
etaDot = data.attitude_vel.signals.values;
pbDot = data.linear_vel.signals.values;
taub = data.tau.signals.values;
ut = data.thrust.signals.values;

t = data.linear_vel.time;

%% Estimator data
Ts = 0.001; % s
r = 1; % estimator order

% initial conditions and variable initialization
q = zeros(6,length(t));
gamma = zeros(6,length(t),r);
extWrench_estim = zeros(6,length(t)); 


%% Estimator

% using a multiple pole tf 
k0 = 50;
s = tf('s');
G = k0^r/(s + k0)^r; % p = k0
c = cell2mat(G.Denominator);
c = c(2:end);
k = zeros(r,1);

% obtaining ki
Ptmp = 1; % accumulator for prod 
for j = 1:r
    k(j) = c(j)/Ptmp;
    Ptmp = Ptmp*k(j);
end
k = flip(k);

% loop in time
for i = 1:size(t,1)-1
    % compute q
    Rb = attitude(eta(i,1), eta(i,2), eta(i,3));
    [Q, Qdot] = geometricTranform(eta(i,1), eta(i,2), etaDot(i,1), etaDot(i,2));
    M = Q'*Ib*Q;
    C = centrifugalMatrix(etaDot(i,:), Q, Qdot,Ib);

    q(:,i+1) = [m*eye(3), zeros(3,3); zeros(3,3), M]*[pbDot(i+1,:)'; etaDot(i+1,:)'];
    % compute gamma1
    gamma(:,i+1,1) = gamma(:,i,1) + k(1)*(q(:,i+1) - q(:,i) - Ts*extWrench_estim(:,i) - Ts*[(m*g*e3 - ut(i)*Rb*e3); C'*etaDot(i,:)' + Q'*taub(i,:)']);
    % loop for j-th gamma
    for j = 2:r        
        gamma(:,i+1,j) = gamma(:,i,j) + k(j)*Ts*(-extWrench_estim(:,i) + gamma(:,i,j-1)); 
    end
    % update the estimation
    extWrench_estim(:,i+1) = gamma(:,i+1,r);
end


%% Plot results
%expected value
fx = 0.5; % N
fy = fx; % N
tauz = 0.2; % Nm

figure('Name','Estimation Values vs Real Values')
subplot(2,3,1)
plot(t, extWrench_estim(1,:), 'LineWidth', 1)
hold on
plot(t, fx.*ones(size(t)), 'LineWidth', 1, 'LineStyle', '--')
grid on
hold off
xlabel('Time [s]')
ylabel('Force [N]')
legend('Disturbances around X-Axis','Real Disturbances')
title('Disturbances along X-Axis')

subplot(2,3,2)
plot(t, extWrench_estim(2,:), 'LineWidth', 1)
hold on
plot(t, fy.*ones(size(t)), 'LineWidth', 1, 'LineStyle', '--')
grid on
xlabel('Time [s]')
ylabel('Force [N]')
legend('Disturbances around Y-Axis','Real Disturbances')
title('Disturbances along Y-Axis')

subplot(2,3,3)
plot(t, extWrench_estim(3,:), 'LineWidth', 1)
grid on
xlabel('Time [s]')
ylabel('Force [N]')
legend('Disturbances around Z-Axis')
title('Disturbances along Z-Axis')

subplot(2,3,4)
plot(t, extWrench_estim(4,:), 'LineWidth', 1)
grid on
xlabel('Time [s]')
ylabel('Torque [Nm]')
legend('Disturbances around Roll-Axis')
title('Disturbances around Roll-Axis')

subplot(2,3,5)
plot(t, extWrench_estim(5,:), 'LineWidth', 1)
grid on
xlabel('Time [s]')
ylabel('Torque [Nm]')
legend('Disturbances around Pitch-Axis')
title('Disturbances around Pitch-Axis')

subplot(2,3,6)
plot(t, extWrench_estim(6,:), 'LineWidth', 1)
hold on
plot(t,tauz.*ones(size(t)),'LineWidth', 1, 'LineStyle', '--')
grid on
xlabel('Time [s]')
ylabel('Torque [Nm]')
legend('Disturbances around Yaw-Axis','Real disturbances')
title('Disturbances around Yaw-Axis')

%% Compute the mass of the UAV from the estimated disturbances
% qdot = m*g*e3 - u_t*Rb*e3 + fe if qdot = 0 and fe = mtilde*g
uz = ut(end)*Rb*e3;
uz = uz(3);
mEst = uz / g; % from thrust

% from disturbances
mtilde = extWrench_estim(3,end)/g;
mEst = m + mtilde;

%% Function definitions
function Rb = attitude(phi, theta, psi)
    % Compute the rotational matrix from the euler RPY angles
    Rb = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
        cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
        -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];
end

function [Q, Qdot] = geometricTranform(phi, theta, phi_d, theta_d)
    % Compute the transformation between rotational velocity (RPY angles)
    % to angular velocity
    Q = [1, 0, -sin(theta); 
        0, cos(phi), cos(theta)*sin(phi);
        0, -sin(phi), cos(theta)*cos(phi)];
    
    % derivative respect time
    Qdot = [0, 0, -cos(theta_d)*theta_d;
            0, -sin(phi_d)*phi_d, -sin(theta_d)*theta_d*sin(phi_d) + cos(theta_d)*cos(phi_d)*phi_d;
            0, -cos(phi_d)*phi_d, -sin(theta_d)*theta_d*cos(phi_d) - cos(theta_d)*sin(phi_d)*phi_d];
end

function S = skew(w)
    % compute the skew symmetric matrix
    S = [0, -w(3), w(2);
        w(3), 0, -w(1);
        -w(2), w(1), 0];

end

function C = centrifugalMatrix(etaDot,Q,Qdot,Ib)

    S = skew(Q*etaDot');
    
    % Computing the centrifugal matrix
    C = Q'*S*Ib*Q + Q'*Ib*Qdot;

end






