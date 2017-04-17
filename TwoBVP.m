% @Author: Shaotu Jia
% @function sideSame
% @function InObstacle
% @Function Trajectory
% @Brief: This function is to find the closest node that the car can
% achieve within certain boundary condition. 
% The curved trajectory is separated to small straghit line segments. The
% segment increase in each time and will be checked by boundary condition.
% If the increment under the condtion, keep this increment. Otherwise, stop
% the increment. Finally, this function will output point that is the target
% or closest to the target. 
% The boundary conditions are as follow: 
% First, the trajectory must to go forward to the target point. 
% This is controlled by the function sideSame. If the new points goes away
% from the target point. The trajectory increment will stop.
% Second, x, y, theta, gamma, a, w, v must be under the limitation given in
% H4.txt. Once a parameter is beyond its range the increment will stop. 

function ReachNode = TwoBVP(start,goal,obstacles)

%Initial normal vector to check sides
normalVector = [-(goal.y - start.y) (goal.x - start.x)];

% Initilize minimum distance between point to goal
dist_min = inf;

% Initial parameters
x_min = [];
y_min = [];
a_min = [];
t_min = [];
gamma_min = [];

% Find the reached point that closest to goal
for a = -2 : 0.5 : 2
    for gamma = -pi/4 : pi/10 : pi/4
        % Initialize t, gamma, a
        t = 0;

        % Initial detla_traj for collsion check
        delta_traj = 0;

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
            
            %boundary checker
            if Node.x<0 || Node.x>100 || Node.y<0 || Node.y>100
                break;
            end
            
            %Collsion Checker
            delta_traj = delta_traj + sqrt(deltaX^2 + deltaY^2);
            if delta_traj > 0.25 
                IsObstacle = InObstacle(Node,obstacles);
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

h = Trajectory(start.v,start.w,start.theta,a_min,gamma_min,t_min,start.x,start.y);

ReachNode.x = x_min;
ReachNode.y = y_min;
ReachNode.theta =  mod((start.w * t_min + (1/2) * gamma_min^2 + start.theta), 2*pi);
ReachNode.w = start.w + gamma_min * t_min;
ReachNode.v = start.v + a_min * t_min;
ReachNode.t = t_min;
ReachNode.previous = start;
ReachNode.a = a_min;
ReachNode.gamma = gamma_min;
ReachNode.line = h;













