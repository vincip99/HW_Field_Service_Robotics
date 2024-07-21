clear all
close all
clc

%% Variables
% time variables
tf = 10;
ts = 0.001; 

% Velocity limits
vmax = 2;
wmax = 1;

% Initial configuration
qi = [0; 0; 0];

% Final Configuration
qf = rand(3,1);
qf = qf/norm(qf);

%% Exercise 3
%% Path Planning
% Computing T based on velocity limits
% T = optimalScaling(ts,qi,qf,vmax,wmax); 
[Topt, T]  = optScaling(0, tf, ts, qi, qf, vmax, wmax);
% Choosing tf to be equal to timespan T
tf = Topt;

% arc length 
[s, sdot, sddot, tau] = trapezioidalProfile(0, tf, ts, T);

% using the time unscaled for v and w
t = T * tau;

% trajectory
[x, y, theta, v, w] = cartesianPolyPlan(s, sdot, T, qi, qf);

%% Exercise 4 
%% Trajectory planning for a point B at a distance b from the centre
b = 0.2;

% trajectory x and y coordinates of B
y1 = x + b * cos(theta);
y2 = y + b * sin(theta);

% trajectory x and y velocities coordinates of B
y1dot = cos(theta) .* v - b * sin(theta) .* w;
y2dot = sin(theta) .* v + b * cos(theta) .* w;

% simulink input
y1des = timeseries(y1',t);
y2des = timeseries(y2',t);
y1dotdes = timeseries(y1dot',t);
y2dotdes = timeseries(y2dot',t);
thetades = timeseries(theta',t);

%% Plots
% Timing law plots
figure('Name','Timing law','NumberTitle','off')
subplot(3,1,1)
plot(t,s, 'LineWidth', 1);
title('Time law');
xlabel('s'); ylabel('m');
xlim([0 tf]);
grid on
subplot(3,1,2)
plot(t,sdot, 'LineWidth', 1);
title('Timing law Velocity');
xlabel('s'); ylabel('m/s');
xlim([0 tf]);
grid on
subplot(3,1,3)
plot(t,sddot, 'LineWidth', 1);
title('Timing law Acceleration');
xlabel('s'); ylabel('m/s^2');
xlim([0 tf]);
grid on

% Velocity plots
figure('Name','Velocity Data','NumberTitle','off')
subplot(2,1,1)
plot(t,v, 'LineWidth', 1);
title('Heading Velocity');
xlabel('time'); ylabel('m/s');
xlim([0 tf]);
grid on
subplot(2,1,2)
plot(t,w, 'LineWidth', 1);
title('Angular Velocity');
xlabel('time'); ylabel('rad/s');
xlim([0 tf]);
grid on


% XY plane 
figure('Name','XY plane','NumberTitle','off')
scatter(qi(1),qi(2),'g',"filled");
hold on
scatter(qf(1),qf(2),'r',"filled");
hold on
plot(x,y,'Color', [0 0.4470 0.7410],'LineWidth', 2);
title('X Y Coordinates');
xlabel('m'); ylabel('m');
legend('start','end','path');
grid on

figure('Name','XY plane','NumberTitle','off')
scatter(qi(1),qi(2),'g',"filled");
hold on
scatter(qf(1),qf(2),'r',"filled");
hold on
plot(x,y,'Color', [0 0.4470 0.7410],'LineStyle', "-.");
hold on
scatter(qi(1) + b*cos(qi(3)),qi(2) + b*sin(qi(3)),'y',"filled");
hold on
scatter(qf(1) + b*cos(qf(3)),qf(2) + b*sin(qf(3)),'m',"filled");
hold on
plot(y1,y2,'b','LineWidth', 2);
title('X Y Coordinates of B');
xlabel('m'); ylabel('m');
legend('start','end','path','B start','B end','B path');
grid on

%% Function definitions
% Time law
function [sp, spdot, spddot, t] = trapezioidalProfile(ti, tf, ts, T)
    
    % Calculating the timespan from initial to fianl time
    timespan = tf - ti;
    % time scaling
    timespan = timespan/T;

    % Generating an evenly spaced time vector from 0 to 1
    t = linspace(0, timespan, (tf - ti)/ts);

    % Cruise velocity definition
    v = 1.5 / timespan;
    % Constant acceleration
    a = v^2 / (v * timespan - 1);
    % constant acceleration time 
    ta = v / a;


    % Initialize s, sdot and sddot 
    sp = zeros(size(t));
    spdot = zeros(size(t));
    spddot = zeros(size(t));
    
    % sequence of polynomial for a trapezioidal velocity profile
    for i = 1 : length(t)
        % Parabolic trait
        if t(i) >= 0 && t(i) <= ta
           sp(i) = 0.5 * a * t(i)^2;
           spdot(i) = a * t(i);
           spddot(i) = a;
        % Linear trait
        elseif t(i) > ta && t(i) <= timespan - ta 
            sp(i) = v * t(i) - v^2 / (2*a);
            spdot(i) = v;
            spddot(i) = 0;
        % Parabolic trait
        elseif t(i) > timespan - ta && t(i) <= timespan 
            sp(i) = (2*a*v*timespan - 2*v^2 - a^2*(t(i)-timespan)^2) / (2*a); 
            spdot(i) = a * (timespan - t(i));
            spddot(i) = -a;
        end
    end

end

% Geometric Path
function [x, y, theta, v, w] = cartesianPolyPlan(s, sdot, T, qi, qf)
    
    % Cubic coefficients
    xi = qi(1); yi = qi(2);
    xf = qf(1); yf = qf(2);
    thetai = qi(3); thetaf = qf(3);
    
    k = 2;
    
    alphax = k*cos(thetaf) - 3*xf;
    alphay = k*sin(thetaf) - 3*yf;
    betax = k*cos(thetai) + 3*xi;
    betay = k*sin(thetai) + 3*yi;
    
    % cubic polynomials
    x = s.^3 * xf - (s - 1).^3 * xi + alphax * s.^2 .* (s - 1) + betax * s .*(s - 1).^2;
    y = s.^3 * yf - (s - 1).^3 * yi + alphay * s.^2 .* (s - 1) + betay * s .*(s - 1).^2;
    
    xp = 3*s.^2*xf - 3*(s-1).^2*xi + alphax*(2*s.*(s-1) + s.^2) + betax*((s-1).^2 + 2*s.*(s-1));
    yp = 3*s.^2*yf - 3*(s-1).^2*yi + alphay*(2*s.*(s-1) + s.^2) + betay*((s-1).^2 + 2*s.*(s-1));

    xpp = 6*s*xf - 6*(s-1)*xi + 2*alphax*(s-1) + 4*alphax*s + 4*betax*(s-1) + 2*betax*s; 
    ypp = 6*s*yf - 6*(s-1)*yi + 2*alphay*(s-1) + 4*alphay*s + 4*betay*(s-1) + 2*betay*s;
    
    % flat outputs
    theta = atan2(yp,xp);
    v = sqrt(xp.^2 + yp.^2);
    w = (ypp.*xp - xpp.*yp)./(xp.^2 + yp.^2);

    % input as time functions with uniform scaling
    v = v .* sdot / T;
    w = w .* sdot / T;
    
end

function [Topt, T] = optScaling(ti, tf, ts, qi, qf, vmax, wmax)
    % |v| < vmax , v(t) = v(s)*sdot/T => v(s)*sdot / vmax = T
    %% Optimal T
    % Timing law with uniform scaling, using tf - ti as scaling factor
    [s, sdot] = trapezioidalProfile(ti, tf, ts, tf - ti);
    
    % v and w without scaling to obtain v(s)*sdot and w(s)*sdot
    [~, ~, ~, v, w] = cartesianPolyPlan(s, sdot, 1, qi, qf);
    
    % Computing the optimal period to be in the velocity bounds (v(s)*sdot
    % / vmax = T)
    Topt = max(max(abs(v))/vmax, max(abs(w))/wmax);
    
    %% Suboptimal T
    % Timing law without uniform scaling, using tf - ti as scaling factor
    [s, sdot] = trapezioidalProfile(ti, tf, ts, tf - ti);
    
    % v and w without scaling to obtain v(s)*sdot and w(s)*sdot
    [~, ~, ~, v, w] = cartesianPolyPlan(s, sdot, 1, qi, qf);

    % Computing the new period only if needed (i.e. the velocities are
    % greater then the bounds)
    if max(abs(v))>vmax || max(abs(w))>wmax
        T = max(max(abs(v))/vmax, max(abs(w))/wmax);
    else
        T = tf - ti;
    end

end

% Computing T such to optimize scaling iterating T
function T = iterativeScaling(ts,qi,qf,vmax,wmax)
    % |v| < vmax , v(t) = v(s)*sdot/T => v(s)*sdot / vmax = T
    dt = 0.1;
    found = false;
    T = dt;
    
    % loop until the boundary condition on the velocity is ensured
    while found == false
    % Timing law with scaling period
    [s, sdot] = trapezioidalProfile(0, T, ts, T);
    
    % v and w without scaling
    [~, ~, ~, v, w] = cartesianPolyPlan(s, sdot, T, qi, qf);
    
    % if the velocities are mor than the limits, increase the period
    if max(max(abs(v)), max(abs(w))) > min(vmax, wmax)
        T = T + dt;
        found = false;
    else
        found = true;
    end

    %T = max(max(abs(v))/vmax, max(abs(w))/wmax);
    end

end



