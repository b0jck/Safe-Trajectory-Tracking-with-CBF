clear
close all 
clc

% Use global Parameters to simplify simulation
global A B K Kp Kd mu d d1 f Ox Oy R Obs alpha

global Pd Xd Yd Pd_dot Xd_dot Yd_dot Pd_dd s r a b Ox Oy Ux Uy Ob1 Ob2 H
syms Pd Xd Yd Pd_dot Xd_dot Yd_dot Pd_dd s

%% Trajectory Parameters (for feedforward)
% Origin
Ox = 20;
Oy = 20;

% Ellipse's axes
a = 20;
b = 10;

% Angular velocity (in Rad/s)
f = pi/8;

%% Feedforward (elliptic trajectory)

% Trajectory Reference
Pd = [  a*sin(f*s)+Ox  ;
        b*cos(f*s)+Oy  ]

% First derivative (velocity reference)
Pd_dot  = [  a*f*cos(f*s);
            -b*f*sin(f*s)];

Xd_dot = Pd_dot(1,:);
Yd_dot = Pd_dot(2,:);

% Second derivative (acceleration reference for feedforward term)
Pd_dd = [   -a*(f^2)*sin(f*s);
            -b*(f^2)*cos(f*s)];

%% Obstacles Coordinates

% In this scenario, all obstacles are moving. Otherwise, specify constant
% Position of each obstacle

%% Controller's parameters 
% Proportional And derivative Gains
Kp = 100;
Kd = 30;

% Gain Matrix
K = [   Kp  0   Kd  0   ;
        0   Kp  0   Kd  ];

% CBF Parameters
alpha = 5;
mu = 0.05;
d = 2;
d1 = 5;
r = 3;

% Controller saturation
sat = 20;

%% System Dynamics' Matrices

A = [   0 0 1 0 ;
        0 0 0 1 ;
        0 0 0 0 ;
        0 0 0 0 ];


B = [   0 0 ;
        0 0 ;
        1 0 ;
        0 1 ];

%% Closed Loop system simulation

% Initial state (robot in the origin, with zero velocity)
x0 = [0 0 0 0];

% Simulation time vector
tspan=0:0.05:30;

% Start simulation
[t, x]=ode45(@simulation,tspan,x0);

% Convert data format for plotting
[~,Xd,Yd,Ux, Uy, Ob1, Ob2, h] = cellfun(@(t,x) simulation(t,x.'), num2cell(t), num2cell(x,2),'uni',0);
H = cell2mat(h)

%% Plotting

% Robot Trajectory
figure(1)
title('Robot Trajectory')
xlabel('x')
ylabel('y')

% X,Y position
X = x(:,1);
Y = x(:,2);

% X,Y velocities
X_dot = x(:,3);
Y_dot = x(:,4);

% Plot robot trajectory
plot(X,Y,'color', 'b', 'LineWidth', 2)

axis equal
hold on



% Plot Obstacles
for i=1:size(Obs,2)
    circ1 = nsidedpoly(1000, 'Center', Obs(:,i)', 'Radius', d);
    
    plot(Obs(1,i), Obs(2,i),'bo','LineWidth',5);
    plot(circ1, 'FaceColor', 'r')
end


% Plot Reference Trajectory
xd = cell2mat(Xd);
yd = cell2mat(Yd);

plot(xd, yd ,'color', 'b', 'LineWidth', 2)



% Extract robot's "trail"
[X1, X2, Y1, Y2] = border(X,Y,X_dot, Y_dot);

% Plot the two curves that define the trail
plot(X1, Y1, 'b', 'LineWidth', 1);
plot(X2, Y2, 'b', 'LineWidth', 1);

% Fill area in between
for i=2:length(X1)
    plot([X1(i) X2(i)], [Y1(i) Y2(i)], 'b', 'LineWidth', 0.1)
end

% Control Effort
figure(2)
title('Control Effort')
xlabel('t')
ylabel('u(t) [m/s^2]')
ux = cell2mat(Ux);
uy = cell2mat(Uy);
plot(t,ux, 'b',t, uy,'r','LineWidth',2)
yline(sat, 'k--','Label','Saturation')
yline(-sat, 'k--','Label','Saturation')
legend('x acceleration','y acceleration')
ylim([min([ux;uy])-2,max([ux;uy])+2])

% Robot Velocity
figure(3)
title('Robot Velocities')
xlabel('t')
ylabel('v(t) [m/s]')
plot(t,X_dot, 'b',t, Y_dot,'r','LineWidth',2)
legend('x velocity','y velocity')
ylim([min([X_dot;Y_dot])-2,max([X_dot;Y_dot])+2])

%% Simulation Function
function [dx,Xd,Yd, Ux, Uy, Ob1, Ob2, h] =simulation(t,x)
global A B Pd Pd_dot Pd_dd s Kp Kd Obs mu alpha r a b f Ox Oy

d = 2;

% FeedForward
yref = [  a*sin(f*t)+Ox  ;
        b*cos(f*t)+Oy  ];

yref_dot  = [  a*f*cos(f*t);
            -b*f*sin(f*t)];

yref_dd = [   -a*(f^2)*sin(f*t);
            -b*(f^2)*cos(f*t)];

yref = double(yref);
yref_dot = double(yref_dot);
yref_dd = double(yref_dd);

% Obstacle Position (moving)
Ob1 = [   a*sin(-0.8*f*t-90)+Ox  ;
        b*cos(-0.8*f*t-90)+Oy+2  ];

Ob2 = [a*sin(-0.2*f*t+90)+Ox ;
        b*cos(-0.2*f*t+90)+Oy-2 ];

Ob1 = double(Ob1);
Ob2 = double(Ob2);

Obs = [Ob1, Ob2];

% Position and Velocity
Pi = x(1:2);
Pi_dot = x(3:4);

% Nominal control
U_nom = yref_dd + Kd*(yref_dot - Pi_dot) + Kp*(yref-Pi);
t

% Compute distances from obstacles
len = length(Obs);
dist = zeros(1, len);
for i=1:len
dist(1,i) = (Obs(:,i) - Pi)'*(Obs(:,i) - Pi);
end

% Find closest Obstacle
[~, index] = min(dist);
Pobs = Obs(:, index);
dist(index) = inf;

% Find second closest Obstacle
[~, ind] = min(dist);
Pobs2 = Obs(:, ind);

% Distance from closest Obstacle
z = Pi - Pobs;

% Distance between the two closest obstacles
l = sqrt((Pobs2 - Pobs)'*(Pobs2 - Pobs));

% If robot can't fit between the two closest obstacles, consider a
% single obstacle that "covers" the two
if sqrt(z'*z) < l+d + 2*r && l <= 2*(r + d)
    Pobs = (Pobs2 + Pobs)/2;
    d = 2*(l/2+d+r);
    z = Pi - Pobs ;
end

% Quadratic Programming optimization
Q = eye(2);
E = mu*z';
L = (2+alpha*mu)*(z'*Pi_dot) + mu*(Pi_dot'*Pi_dot) + alpha*(z'*z-r^2) - 8*alpha*(d+r);% + (z'*Pi_dot)/((Pobs-Pobs2)'*Pi_dot+r);

options = optimset('Display', 'off');
u = quadprog(Q, -2*U_nom, -E,L,[],[],[],[],[], options) ;

% Controller Saturation (if needed)
sat = 20;

if u(1) > sat
    u(1) = sat;
end

if u(1) < -sat
    u(1) = -sat;
end

if u(2) > sat
    u(2) = sat;
end

if u(2) < -sat
    u(2) = -sat;
end

% X,Y control
Ux = u(1);
Uy = u(2);

% X,Y reference 
Xd = yref(1);
Yd = yref(2);

% CBF evaluation index
h = (Pi-Pobs)'*(Pi-Pobs)+mu*(Pi-Pobs)'*Pi_dot - (d+r);

% Proceed to next simulation step
dx = A*x + B*u;
end

%% Trail generator function
% Compute trail based on robot velocities and position, given it's radius
function [X1, X2, Y1, Y2]=border(X,Y,X_dot, Y_dot)
    global r
    n = length(X);
    x1 = zeros(n,1);
    y1 = zeros(n,1);

    x2 = zeros(n,1);
    y2 = zeros(n,1);
    
   

    for i=2:n
    d = sqrt(X_dot(i)^2+Y_dot(i)^2);
    sin_a = Y_dot(i)/d;
    cos_a = X_dot(i)/d;
    
    x1(i) = X(i)-r*sin_a;
    y1(i) = Y(i)+r*cos_a;

    x2(i) = X(i)+r*sin_a;
    y2(i) = Y(i)-r*cos_a;
    end
    X1 = x1;
    Y1 = y1;
    X2 = x2;
    Y2 = y2;
end