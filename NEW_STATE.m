%This function generate the new Node

function NodeNew = NEW_STATE(NodeNear, NodeRand, epsilon)

dist = sqrt ((NodeNear.x - NodeRand.x)^2 + (NodeNear.y - NodeRand.y)^2);

NodeNew.x = NodeNear.x + (NodeRand.x - NodeNear.x) * (epsilon/dist);

NodeNew.y = NodeNear.y + (NodeRand.y - NodeNear.y) * (epsilon/dist);

NodeNew.previous = NodeNear;


end