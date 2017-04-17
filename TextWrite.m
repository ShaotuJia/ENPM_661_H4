% This script is to generate text file 

temp = goal.previous;
i = [];
a = [];
gamma =[];
w = [];
v = [];
theta = [];
x = [];
y = [];
t = [];
N = 0;

 while temp.x ~= start.x || temp.y ~= start.y
     N = N + 1;
     i(end+1) = N;
     a(end+1) = temp.a;
     gamma(end+1) = temp.gamma;
     w(end+1) = temp.w;
     v(end+1) = temp.v;
     theta(end+1) = temp.theta;
     x(end+1) = temp.x;
     y(end+1) = temp.y;
     t(end+1) = temp.t;
    

     %update temp
     temp = temp.previous;
       
 end
 
 % flip and reshape array
 a = fliplr(a);
 gamma = fliplr(gamma);
 w = fliplr(w);
 v = fliplr(v);
 theta = fliplr(theta);
 x = fliplr(x);
 y = fliplr(y);
 t = fliplr(t);
 
 % update to global time
 for k = 2 : N
    t(k) = t(k-1) + t(k);
 end
 
 
% output text file
A = [t; x; y; theta; v; w; a];

fileID = fopen('text_P5.txt','w');
fprintf(fileID,'%s      %s          %s        %s      %s       %s        %s\n','t','x','y','theta', 'v','w','a');
fprintf(fileID,'%d     %.2f     %.2f     %.2f     %.2f     %.2f     %.2f\n',A);
fclose(fileID);

 
 