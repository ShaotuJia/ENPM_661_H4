% This function is to draw the trajectory between two points

function Trajectory(v_init, w_init, theta_init, a, gamma, delta_epsilon, t_min,x_start, y_start)

%initial time t and delta_t
t = 0;

% initialize the function of displacement along x and y axis
delta_x = @(t) cos(w_init .* t + ((1/2) .* gamma .* (t.^2)) + theta_init) .* (v_init + a .* t);
delta_y = @(t) sin(w_init .* t + ((1/2) .* gamma .* (t.^2)) + theta_init) .* (v_init + a .* t);

% Initialize the start point to draw a increment line
seg_X = x_start;
seg_Y = y_start;

% Iniitilaze v_s
v_s = v_init + a * t;

while t < t_min
    
delta_t = deltaT(v_s, a, delta_epsilon);

    if (t_min - t) < delta_t 
        t = t_min;
    else
        t = t + delta_t;
    end


% obtain the distance between reached point to goal 
deltaX = integral(delta_x, 0 , t);
deltaY = integral(delta_y, 0 , t);

% Get end point of small increment
seg_X(end + 1) = x_start + deltaX;
seg_Y(end + 1) = y_start + deltaY;

% update v_s
v_s = v_init + a * t;
end

% draw trajectory
figure(1), title('trajectory'), line(seg_X, seg_Y), hold on;

end