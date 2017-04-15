clc;
clear;
close all;

% This script is to test two points boundary value problem

% coordinate of start and goal
start_x = 5;
start_y = 5;

goal_x = 0;
goal_y = 0;

% Initial orientation theta
theta_init = 0;

% Initial delta_theta
delta_theta = atan((goal_y - start_y) / (goal_x - start_x));

% Initial forward velocity
v_init = 0;

% Initial steering veloctiy 
w_init = 0;

% Initialize delta increment
delta_epsilon = 0.25;

% Initilize distance in previous step
dist_prev = inf;

% Initilize distance between point to goal
dist = 0;

% Initilize minimum distance between point to goal
dist_min = inf;

% Find the reached point that closest to goal
for a = -2 : 0.5 : 2
    for gamma = -pi/4 : pi/10 : pi/4
        
        % Initialize time t
        t = 0;
        
        % initial thetaT
        thetaT = @(t) w_init .* t + ((1/2) .* gamma .* (t.^2));

        % initialize the function of displacement along x and y axis
        delta_x = @(t) cos(w_init .* t + ((1/2) .* gamma .* (t.^2)) + theta_init) .* (v_init + a .* t);
        delta_y = @(t) sin(w_init .* t + ((1/2) .* gamma .* (t.^2)) + theta_init) .* (v_init + a .* t);
     
        while abs(thetaT(t)) <= abs(delta_theta) && t<60
            % obtain the distance between reached point to goal 
            deltaX = integral(delta_x, 0 , t);
            deltaY = integral(delta_y, 0 , t);

            distX = goal_x - (start_x + deltaX);
            distY = goal_y - (start_y + deltaY);
            dist = sqrt(distX^2 + distY^2);

            % get the point in minmum distance
            if dist_min > dist
                dist_min = dist;
                x_min = start_x + deltaX;
                y_min = start_y + deltaY;
                a_min = a;
                gamma_min = gamma;
                t_min = t;
            end
            
            % find delta_t for each incrment
            v_s = v_init + a * t;
            delta_t = deltaT(v_s, a, delta_epsilon);
            % update time t
            t = t + 1;
            % update dist_prev
            dist_prev = dist;
        end
    end

end

% plot test
figure(1), plot(start_x, start_y), hold on;
plot(goal_x, goal_y), hold on;
plot(x_min, y_min, '*');












