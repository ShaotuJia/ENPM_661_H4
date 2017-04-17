%@Author: Shaotu Jia
%@Brief: This function is to reconstruct the path when reaching the goal

function reconstructPath(start , goal)
 
 temp = goal.previous;
 
 plot (start.x , start.y , '*', 'MarkerEdgeColor' , 'black');
 hold on;
 plot (goal.x , goal.y , '-o' , 'MarkerEdgeColor' , 'black');
 hold on;
 
 while temp.x ~= start.x || temp.y ~= start.y
     
     h = line(temp.line.XData, temp.line.YData);
     h.Color = 'green';
     hold on;
     
     temp = temp.previous;

end
