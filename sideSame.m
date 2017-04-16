% check whether the trajectory cross the line between start to goal
%@param result_prev This is vector * normal in previous loop round
function [logical, result] = sideSame(result_prev, Node, normalVector, global_start)

vector_new = [Node.x-global_start.x Node.y-global_start.y];

result = dot(vector_new, normalVector); 
if result * result_prev >= 0
    logical = true;
else
    logical = false;
end

end