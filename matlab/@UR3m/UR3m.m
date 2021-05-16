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
            obj.PlotAndColourRobot();

            % if a base was given, move the robo
            if nargin > 0
                obj.model.base = obj.model.base * base;
            end

            % Create plot with robot, get colours from ply file
            q0 = ones(1, 6);
            %obj.model.plot(q0, 'workspace', obj.workspace);

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

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(obj)
            % Load Ply files
            % for each link...
            for linkIndex = 0:obj.model.n
                % read play file
                [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['@UR3m/ply2/l',num2str(linkIndex),'.ply'],'tri');
                
                % add to model
                obj.model.faces{linkIndex+1} = faceData;
                obj.model.points{linkIndex+1} = vertexData;
            end

            % Display robot (plot3d)
            obj.model.plot3d(zeros(1,obj.model.n),'noarrow','workspace',obj.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            obj.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            % for each link...
            for linkIndex = 0:obj.model.n
                handles = findobj('Tag', obj.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end  
    end
end