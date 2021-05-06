clear;
clc;
close all;

% Initialise a figure with 2 subplots
% Also make a UR3 and put it in one

%% --------VARIABLES--------
%  -------------------------

% the pushbotton should set this true, when its time to FIRE
go = false;


%% Create a maximised figure
fig = figure('units','normalized','outerposition',[0 0 1 1]);

%% --------SUBPLOT 1--------
%  -------------------------
% UR3 subplot
subplot(1, 2, 1);

% create ur3
ur3 = UR3m(trotz(pi/2));

% set view properties
view(30, 20);
daspect([1 1 1]);

%% Create goal indicators for x, y, z
% these values should update as the cup pose changes
% TODO, subscribe to cup pose and convert to XYZ
xlabel = uicontrol('Style', 'text', 'String', 'x-goal', 'position', [20 80 100 15]);
xgoal = uicontrol('Style', 'text', 'String', num2str(0), 'position', [20 40 100 30]);

ylabel = uicontrol('Style', 'text','String', 'y-goal', 'position', [150 80 100 15]);
ygoal = uicontrol('Style', 'text','String', num2str(0), 'position', [150 40 100 30]);

zlabel = uicontrol('Style', 'text','String', 'z-goal', 'position', [280 80 100 15]);
zgoal = uicontrol('Style', 'text','String', num2str(0), 'position', [280 40 100 30]);

% use this to update the values in the fields
% set(xgoal, 'String', num2str(n));

%% Create GO button
gobutton = uicontrol('String', 'Launch!', 'position', [150 120 100 40]);
gobutton.Callback = @gocb;

%% --------MAIN GUI LOOP--------
%  -----------------------------
while go == false
    pause(0.1);
    disp('nup');
end
disp('go became true');

%% Functions
function gocb(src, event)
    
    % button press
    g = true;
    
    % send this value to the base workspace variable 'go'
    assignin('base', 'go', g);
end

