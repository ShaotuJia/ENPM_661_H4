%Collision Detector
 
function logic = InObstacle(NodeTemp)

obstacles = load ('obstacles.txt'); % load obstacle data

%draw obstacles in graph
drawCircle(obstacles);

%find the number of obstacles
N = length(obstacles(:,1));

logic = false;

for i = 1 : N
    
    OB = obstacles(i,:);
    
    %distance between obstacle center to new Node
    dist = sqrt((NodeTemp.x - OB(1))^2 + (NodeTemp.y - OB(2))^2);
    
    
   if dist <= OB(3)
        logic = true;
        break;
    end 

end
