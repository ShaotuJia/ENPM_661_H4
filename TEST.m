%function test

%Initialize parameters
%{
v_init = 0; 
w_init = 0; 
theta_init = 0;
a = -1.5;
gamma = 0.1571;
delta_epsilon = 0.25;
t_min = 3.1091;
x_start = 5;
y_start = 5;
%}

% Initialize start and goal points
start.x = 0;
start.y = 0;
start.theta = 7/4 * pi;
start. v = 0;
start. w = 0;

goal.x = 5;
goal.y = 5;


ReachNode = TwoBVP(start,goal);

%Trajectory(v_init,w_init,theta_init,a,gamma,delta_epsilon,t_min,x_start,y_start);
