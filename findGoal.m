%@Author: Shaotu Jia
%@Brief: This function is to find whether the trajectory arrive the goal
%range
function logic = findGoal(NodeNew, goal)

dist = sqrt((NodeNew.x - goal.x)^2 + (NodeNew.y - goal.y)^2);

if dist <= goal.R
    
    logic = true;
    
else
    logic = false;
end
    
end