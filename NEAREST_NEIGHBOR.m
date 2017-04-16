% This function is to find nearest Node to the rand Node

function NodeNear = NEAREST_NEIGHBOR(NodeRand, closeSet)

NodeNear = closeSet(1);

dist = sqrt( (NodeNear. x - NodeRand. x)^2 + (NodeNear.y - NodeRand. y)^2);

for i = 2 : length(closeSet)
    
    distTemp = sqrt( (NodeRand.x - closeSet(i).x)^2 + (NodeRand.y - closeSet(i).y)^2);
    
    if distTemp < dist
        
        dist = distTemp;
        
        NodeNear = closeSet(i);
    end    
    
end

end