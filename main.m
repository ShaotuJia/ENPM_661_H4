%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%ENPM661 Homework 3
%Author: Shaotu Jia
%Brief: This program is to find fesiable path using RRT algorithm
%Output Explantion: the output is figures and saved in 'Problem_Results'
%folder. In each figure, 'star point' means the start point, 'circle point'
%means goal point, 'green line' means the feasible path

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear;
close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% !!!change this section for different inputs!!!
% Initialize Problem_1 requirements
start.x = 1;
start.y = 99;
start.theta = pi;
start.v = 0;
start.w = 0;
start.previous.x = -1;
start.previous.y = -1;

goal.x = 100;
goal.y = 0;
goal.R = 20;
epsilon = 5; % the increment of every step

%Initialize Maximum Node Number
K = 4000;

%Initialize the figure
figure(1), title('Problem 1 RRT Graph'), axis ([0 100 0 100]), hold on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize NodeInitial
NodeInit = start;

% Add NodeInit to closeSet
closeSet = NodeInit;

%Initialize NodeNew
NodeNew = start;


% for loop to generate RRT
for k = 1 : K
    
    NodeRand = RANDOM_STATE();
    
    NodeNear = NEAREST_NEIGHBOR(NodeRand, closeSet);
    
    NodeTemp = NEW_STATE(NodeNear, NodeRand, epsilon);
    
    if InObstacle(NodeTemp) == false
        NodeNew = NodeTemp;
        
       closeSet(end+1) = NodeNew;
    
    %Generate Edge for draw
    Edge.x = [NodeNew.x  NodeNear.x];
    Edge.y = [NodeNew.y  NodeNear.y];
 
    %Draw RRT trees  
    line(Edge.x , Edge.y);
    hold on;
    end
    
    if findGoal(NodeNew,goal) == true 
        goal.previous = NodeNew;
        reconstructPath(start , goal);
        print('-f1','Problem_5','-djpeg');
        disp('find the goal');
        return;
        
    end           
    
end





