% This function is to find the closest node that the car can achieve

function ReachNode = TwoBVP(start,goal)

% Initial delta_theta
%delta_theta = atan((goal.y - start.y) / (goal.x - start.x));
%startTOgoal_theta = atan((goal.y - start.y) / (goal.x - start.x));
%start_vector = [cos(start.theta) sin(start.theta)]; % unit vector to show oritentation
%start_goal = [(goal.y - start.y) (goal.x - start.x)];
%delta_theta = acos(dot(start_vector,start_goal)/(norm(start_vector)*norm(start_goal)));

normalVector = [-(goal.y - start.y) (goal.x - start.x)];

% Initialize delta increment
delta_epsilon = 0.25;

% Initilize minimum distance between point to goal
dist_min = inf;

% Initial detla_traj for collsion check
delta_traj = 0;

% Find the reached point that closest to goal
for a = -2 : 0.5 : 2
    for gamma = -pi/4 : pi/10 : pi/4
        
        % Initialize time t
        t = 0;
        
        % initial thetaT
       % thetaT = @(t) start.w .* t + ((1/2) .* gamma .* (t.^2));

        % initialize the function of displacement along x and y axis
        delta_x = @(t) cos(start.w .* t + ((1/2) .* gamma .* (t.^2)) + start.theta) .* (start.v + a .* t);
        delta_y = @(t) sin(start.w .* t + ((1/2) .* gamma .* (t.^2)) + start.theta) .* (start.v + a .* t);
     
        % Initialize side checker
        SameSide = true;
        result_prev = 0;
        while  SameSide && t<10 %%abs(thetaT(t)) <= abs(delta_theta)
            
            % get w and v in current momment
            current_w = start.w + gamma * t;
            current_v = start.v + a * t;
            
            %Check speed limit
            if abs(current_w)>(pi/2) && abs(current_v)>5
                break;
            end
            
            % obtain the distance between reached point to goal 
            deltaX = integral(delta_x, 0 , t);
            deltaY = integral(delta_y, 0 , t);
            
            % Side checker
            Node.x = start.x + deltaX;
            Node.y = start.y + deltaY;
            [SameSide, result_prev] = sideSame(result_prev, Node, normalVector, start);
            
            %Collsion Checker
            delta_traj = delta_traj + sqrt(deltaX^2 + deltaY^2);
            if delta_traj > 0.25 
                IsObstacle = InObstacle(Node);
                if IsObstacle == true
                    break;
                end
                delta_traj = 0;
            end
            
            distX = goal.x - (start.x + deltaX);
            distY = goal.y - (start.y + deltaY);
            dist = sqrt(distX^2 + distY^2);

            % get the point in minmum distance
            if dist_min > dist
                dist_min = dist;
                x_min = start.x + deltaX;
                y_min = start.y + deltaY;
                a_min = a;
                gamma_min = gamma;
                t_min = t;
            end

            % update time t
            t = t + 1;

        end
    end

end

ReachNode.x = x_min;
ReachNode.y = y_min;
ReachNode.theta =  mod((start.w * t_min + (1/2) * gamma_min^2 + start.theta), 2*pi);
ReachNode.w = start.w + gamma_min * t_min;
ReachNode.v = start.v + a_min * t_min;
ReachNode.t = t_min;

Trajectory(start.v,start.w,start.theta,a_min,gamma_min,delta_epsilon,t_min,start.x,start.y);

% plot test
plot(start.x, start.y,'*'), hold on;
plot(goal.x, goal.y,'*'), hold on;
plot(ReachNode.x, ReachNode.y , '*');












