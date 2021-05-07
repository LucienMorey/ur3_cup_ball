clear;
clc;
close all;

% Initialise a figure with 2 subplots
% Also make a UR3 and put it in one

%% --------VARIABLES--------
%  -------------------------

% the pushbotton should set this true, when its time to FIRE
go      = false;
robot_q = zeros(1, 6);
start   = zeros(1, 3);
goal    = zeros(1, 3);


%% Create a maximised figure
fig = figure('units','normalized','outerposition',[0 0 1 1]);

%% --------SUBPLOT 1--------
%  -------------------------

%{
    This plot shows the UR3 in 3d. 
    In the main loop, the joint states are recieved and the ur3 position
    should be updated.

    When the go button is pressed, calculate the trajectory then draw it in
    3D on the plot so we can see where the ball will go.
%}

% UR3 subplot
h_plot1 = subplot(1, 2, 1);

% create ur3
ur3 = UR3m(trotz(pi/2));

% set view properties
view(30, 20);
daspect([1 1 1]);

%% --------SUBPLOT 2--------
%  -------------------------

%{
    This subplot just displays the 2D trajectory
%}

h_plot2 = subplot(1, 2, 2);
grid on;
title('Trajectory 2D x,y');
xlabel('X (m)');
ylabel('Y (m)');
daspect([1 1 1]);
set(gcf,'color','w');
xlim([0 1.2]);
ylim([0 1.2])
pos = get(gca, 'OuterPosition');
set(gca, 'OuterPosition', pos);


%% Create goal indicators for x, y, z

%{
    These are just output text so you can see the current X,Y,Z of the cup.
    The labels dont need to change, but each loop should update the goal
    fields. This can be done using the line I commented out below, for each
    x, y, z.
%}

% these values should update as the cup pose changes
% TODO, subscribe to cup pose and convert to XYZ
xlabel = uicontrol('Style', 'text', 'String', 'x-goal', 'position', [20 80 100 15]);
xgoal = uicontrol('Style', 'text', 'String', num2str(0), 'position', [20 40 100 30]);

ylabel = uicontrol('Style', 'text','String', 'y-goal', 'position', [150 80 100 15]);
ygoal = uicontrol('Style', 'text','String', num2str(0), 'position', [150 40 100 30]);

zlabel = uicontrol('Style', 'text','String', 'z-goal', 'position', [280 80 100 15]);
zgoal = uicontrol('Style', 'text','String', num2str(0), 'position', [280 40 100 30]);

% set(xgoal, 'String', num2str(n));


%% Create GO button

%{
    Create the go button. When this is pressed -> calculate trajectory ->
    display it on figure -> publish go message
%}
gobutton = uicontrol('String', 'Launch!', 'position', [150 120 100 40]);
gobutton.Callback = @gocb;


%% --------MAIN GUI LOOP--------
%  -----------------------------

% ROS master address
rosinit();

% subscriber
% joint states for disp
roboqsub = rossubscriber('/joint_states');

% transform tree
tree = rostf;

%{
    Main GUI loop, update figures, get msg from subscriber, get tf from
    tree.

    If go message get publish, calcualte trajectory, output 3d on plot1
    (with ur3)
    and 2d on plot2.

%}
while true
    
    % get joint states
    msg = recieve(roboqsub, 1);
    q_robo = msg.Position;
    ur3.model.animate(q_robo);
    
    
    % dunno if necessary, but since we animate here maybe slow it down
    pause(0.01);
end

% finish ROS
rosshutdown;



%% Callbacks

%{
    Callback for go button.

    Currently sets the go variable, use this to check if its time to
    calculate trajectory.

    TODO: THIS CALLBACK SHOULD PUBLISH THE GO MESSAGE SO THAT THE ROBOT
    KNOWS TO MOVE
%}
function gocb(src, event)
    
    % button press
    g = true;
    
    % send this value to the base workspace variable 'go'
    assignin('base', 'go', g);
    
    % PUBLISH MESSAGE GOGOGO
end

