%% UR3 model
% defines DH parameters
% imports ply files

classdef UR3m < handle
    properties
        model;
        workspace;
    end
    
    methods
        %% Constructor
        % base is a 4x4 homogeneous transform for base pose
        % path is the path to ply files
        function obj = UR3m(base)

            obj.workspace = [-0.3 2 -0.5 0.5 0 1];

            % define DH parameters and create serial link
            obj.GetRobot();

            % if a base was given, move the robo
            if nargin > 0
                obj.model.base = obj.model.base * base;
            end

            % Create plot with robot, get colours from ply file
            q0 = ones(1, 6);
            obj.model.plot(q0, 'workspace', obj.workspace);

        end
        
        %% getRobot
        function GetRobot(obj)

            % UR3 DH parameters
            % 0 -> d -> a -> alpha
            L(1) = Link([0    0.172    0       -pi/2   0]);
            L(2) = Link([0    0.1235   0.249    0      0]);
            L(3) = Link([0   -0.1      0.2185   0      0]);
            L(4) = Link([0    0.091     0       -pi/2   0]);
            L(5) = Link([0    0.0915    0        pi/2   0]);
            L(6) = Link([0    0.0745    0        0	   0]);

            % UR3 joint limits
            L(1).qlim = [-360 360]*pi/180;
            L(2).qlim = [-200 20]*pi/180;
            L(3).qlim = [-360 360]*pi/180;
            L(4).qlim = [-360 360]*pi/180;
            L(5).qlim = [-360 360]*pi/180;
            L(6).qlim = [-360 360]*pi/180;
            
            % offsets
            %L(2).offset = -pi/2;
            %L(4).offset = -pi/2;

            % Create the robot with serial link
            obj.model = SerialLink(L);
        end
    end
end