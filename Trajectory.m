% @Author: Shaotu Jia
% @Brief: This function is to draw the trajectory between two points

function h = Trajectory(v_init, w_init, theta_init, a, gamma, t_min,x_start, y_start)

% initialize the function of displacement along x and y axis
delta_x = @(t) cos(w_init .* t + ((1/2) .* gamma .* (t.^2)) + theta_init) .* (v_init + a .* t);
delta_y = @(t) sin(w_init .* t + ((1/2) .* gamma .* (t.^2)) + theta_init) .* (v_init + a .* t);

% Initialize the start point to draw a increment line
seg_X = x_start;
seg_Y = y_start;

%Initialize step
step = t_min/20;

for t = 0: step : t_min

% obtain the distance between reached point to goal 
deltaX = integral(delta_x, 0 , t, 'ArrayValued',true);
deltaY = integral(delta_y, 0 , t, 'ArrayValued',true);

% Get end point of small increment
seg_X(end + 1) = x_start + deltaX;
seg_Y(end + 1) = y_start + deltaY;

end

% draw trajectory
h = line(seg_X, seg_Y);
hold on;

end