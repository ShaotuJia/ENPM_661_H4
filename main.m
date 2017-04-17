%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%ENPM661 Homework 4
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
start.x = 40;
start.y = 40;
start.theta = pi/4;
start.v = 0;
start.w = 0;
start.t = 0;
start.a = [];
start.gamma =[];
start.line.XData = [];
start.line.YData = [];
start.previous.x = -1;
start.previous.y = -1;

goal.x = 0;
goal.y = 0;
goal.R = 20;
epsilon = 20; % the increment of every step

%Initialize Maximum Node Number
K = 4000;

%Initialize the figure
figure(1), title('Problem 5 RRT Graph'), axis ([0 100 0 100]), hold on;

%Load obstacle and draw obstracles on figure 1
obstacles = load ('obstaclesH4.txt'); % load obstacle data
drawCircle(obstacles);

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
    
    
    NodeNew = TwoBVP(NodeNear,NodeTemp,obstacles);
    
    %add new node in closeSet
    closeSet(end+1) = NodeNew;
    
    if findGoal(NodeNew,goal) == true 
        goal.previous = NodeNew;
        reconstructPath(start , goal);
        print('-f1','Problem_5','-djpeg');
        disp('find the goal');
        return;
        
    end
      
end





