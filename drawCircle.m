% This function is to draw obstacle in the graph

function drawCircle(obstacles)

N = length(obstacles(:,1));

for i = 1 : N

    xc = obstacles(i,1);
    yc = obstacles(i,2);
    r = obstacles(i,3);
    theta = linspace(0,2*pi);
    x = r*cos(theta) + xc;
    y = r*sin(theta) + yc;
    plot(x,y,'red');
    hold on;

end

end
