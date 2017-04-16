%This function is to reconstruct the path when reaching the goal

function reconstructPath(start , goal)
 
 temp = goal;
 
 plot (start.x , start.y , '*', 'MarkerEdgeColor' , 'black');
 hold on;
 plot (goal.x , goal.y , '-o' , 'MarkerEdgeColor' , 'black');
 hold on;
 
 while temp.x ~= start.x || temp.y ~= start.y
     
     Path.x = [temp.previous.x temp.x];
     Path.y = [temp.previous.y temp.y];
     
     h = line(Path.x , Path.y );
     h.Color = 'green';
     hold on;
     
     temp = temp.previous;

end
