classdef GUI < matlab.apps.AppBase & handle
    %GUI Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        
    end

    properties(Constant)
        SUBSCIRIBER_TIMEOUT = 0.5;
        CARTESIAN_JOG_DIST =0.2;
        UR3_JOINT_NAMES = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
        PROJECTILE_MASS = 0.0027;
        PROJETILE_DIAMETER = 0.04;
        COEFFICENT_OF_DRAG = 0;
        COEFFICIENT_OF_RESTITUTION = 0.86;
        LAUNCH_POSITION = [0.1, 0.0, 0.35];
    end
    
    properties(Access = private)
        fig;

        robotPlot_h;
        trajPlot_h;
        trajLine_h;
        trajXs;
        trajYs;
        
        cupLocationXField
        cupLocationYField
        cupLocationZField
        cupLocationXLabel
        cupLocationYLabel
        cupLocationZLabel
        
        
        openButton
        closeButton
        homeButton
        getCupPoseButton
        fireButton;
        abortButton;
        exitButton;
        xPlusButton;
        xMinusButton;
        yPlusButton;
        yMinusButton;
        zPlusButton;
        zMinusButton;

        servoPublisher;
        estopSubscriber;
        jointStateSubscriber;
        tree;
        cupRobotFrame;
        actionClient;
        trajGoal;
        
        ur3;
        trajectoryGenerator;
        projectileGenerator;
    end
    
    methods
        function obj = GUI()
            %GUI Construct an instance of this class
            %   Detailed explanation goes here
            
            obj.guiElementGenerate();

            % ROS master address
            %%TODO add ros master address
            % rosinit('192.168.0.253');
            rosinit();

            obj.servoPublisher = rospublisher('servo_closed_state', 'std_msgs/Bool', 'IsLatching', false);
            obj.estopSubscriber = rossubscriber('estop', 'std_msgs/Header');
            obj.jointStateSubscriber = rossubscriber('joint_states', 'sensor_msgs/JointState');
            [obj.actionClient, obj.trajGoal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
            obj.trajGoal.Trajectory.JointNames = obj.UR3_JOINT_NAMES;
            obj.trajGoal.GoalTimeTolerance = rosduration(0.05);
            obj.tree = rostf;
    
            reload_position = transl(-0.1,0,0.55);
            throw_position = transl(0,0.3,0.3);
            
            startQ = obj.ur3.model.ikcon(reload_position, ones(1,6));

            obj.ur3.model.animate([0, 0, 0, 0, 0, 0]);
            obj.cupRobotFrame = NaN(4);

            obj.trajectoryGenerator = TrajectoryGenerator(obj.ur3.model, throw_position, 0.1,0.45, reload_position);
            obj.projectileGenerator = Projectile(obj.PROJECTILE_MASS, obj.PROJETILE_DIAMETER, obj.COEFFICENT_OF_DRAG, obj.COEFFICIENT_OF_RESTITUTION);

            
        end
    end

    methods(Access = private)
        
        function onOpenServoButton(obj, app, event)
            servoState = rosmessage('std_msgs/Bool');
            servoState.Data = true;
            send(obj.servoPublisher, servoState);
        end

        function onCloseServoButton(obj, app, event)
            servoState = rosmessage('std_msgs/Bool');
            servoState.Data = false;
            send(obj.servoPublisher, servoState);
        end

        function onExitButton(obj, app, event)
            rosshutdown;
            delete(obj.fig);
        end

        function onGetCupPoseButton(obj, app, event)
            % check for joined tf tree
            try
                % transform origin of cup to robot frame
                pose = rosmessage('geometry_msgs/PoseStamped');
                pose.Header.FrameId = 'cup';
                pose.Pose.Position.X = 0.0;
                pose.Pose.Position.Y = 0.0;
                pose.Pose.Position.Z = 0.0;
                pose.Pose.Orientation.X = 0.0;
                pose.Pose.Orientation.Y = 0.0;
                pose.Pose.Orientation.Z = 0.0;
                pose.Pose.Orientation.W = 1.0;

                transformedPose = transform(obj.tree, 'robot', pose);
                %create rotation matrix from tranformed quaternion
                rotm = quat2rotm([transformedPose.Pose.Orientation.X, transformedPose.Pose.Orientation.Y, transformedPose.Pose.Orientation.Z, transformedPose.Pose.Orientation.W]);
                %compound rotation matrix and translation into homogenous transform
                obj.cupRobotFrame = [rotm, [transformedPose.Pose.Position.X; transformedPose.Pose.Position.Y; transformedPose.Pose.Position.Z]; zeros(1,3), 1];
                %add arbitrary height for cup height
                obj.cupRobotFrame(3,4) = obj.cupRobotFrame(3,4) + 0.1
                % set ui element info
                obj.cupLocationXField.String = num2str(obj.cupRobotFrame(1,4));
                obj.cupLocationYField.String = num2str(obj.cupRobotFrame(2,4));
                obj.cupLocationZField.String = num2str(obj.cupRobotFrame(3,4));

                v0 = 1.0;

                [vRequired, xi, dx, h, theta] = obj.projectileGenerator.calcLaunch(obj.LAUNCH_POSITION, obj.cupRobotFrame(1:3,4)', 1, v0);

                % get initial velocity for plot
                vi = v0 * [cos(theta), sin(theta)];

                % 2D simulation of projectile
                [x, y, t] = obj.projectileGenerator.simulatep(xi, vi, 1);
                axes(obj.trajPlot_h);
                obj.trajLine_h.XData = x;
                obj.trajLine_h.YData = y;
                drawnow();

            % catch broken tf tree
            catch
                disp('tf error check for joined trees');
                % set ui element info
                obj.cupLocationXField.String = 'error';
                obj.cupLocationYField.String = 'error';
                obj.cupLocationZField.String = 'error';
            end
        end

        function onXPlusButton(obj, app, event)
            disp('jog XPlus');
            obj.cartesianJog([obj.CARTESIAN_JOG_DIST,0, 0]);
        end

        function onXMinusButton(obj, app, event)
            disp('jog XMinus');
            obj.cartesianJog([-obj.CARTESIAN_JOG_DIST,0, 0]);
        end

        function onYPlusButton(obj, app, event)
            disp('jog YPlus');
            obj.cartesianJog([0,obj.CARTESIAN_JOG_DIST, 0]);
        end

        function onYMinusButton(obj, app, event)
            disp('jog YMinus');
            obj.cartesianJog([0,-obj.CARTESIAN_JOG_DIST, 0]);
        end

        function onZPlusButton(obj, app, event)
            disp('jog ZPlus');
            obj.cartesianJog([0,0, obj.CARTESIAN_JOG_DIST]);
        end

        function onZMinusButton(obj, app, event)
            disp('jog ZMinus');
            obj.cartesianJog([0,0, -obj.CARTESIAN_JOG_DIST]);
        end

        function cartesianJog(obj, direction)
            axes(obj.robotPlot_h);
            try
                latestMessage = receive(obj.jointStateSubscriber,obj.SUBSCIRIBER_TIMEOUT);
                currentJointState_321456 = (latestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
                currentJointState_123456 = [currentJointState_321456(1,3:-1:1),currentJointState_321456(1,4:6)];
                [qMatrix,vMatrix,tMatrix] = obj.trajectoryGenerator.jog(currentJointState_123456, direction);
                obj.ur3.model.plot(qMatrix, 'trail', 'r', 'fps', 10);
                obj.trajGoal.Trajectory.Points = [];
                for i=1:1:size(qMatrix,1)
                    trajPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                    trajPoint.Positions = qMatrix(i,:);
                    trajPoint.Velocities = vMatrix(i,:);
                    trajPoint.TimeFromStart = rosduration(tMatrix(i,1));
                    obj.trajGoal.Trajectory.Points = [obj.trajGoal.Trajectory.Points; trajPoint];
                    
                end
                obj.trajGoal.Trajectory.Header.Stamp = rostime('now');
                sendGoal(obj.actionClient, obj.trajGoal);

            catch
                disp('No message received');
            end
        end


        function guiElementGenerate(obj)
            %PLOT 1
            obj.fig = figure('units','normalized','outerposition',[0 0 1 1]);
            obj.fig.CloseRequestFcn = @obj.onExitButton;
            
            % UR3 subplot
            obj.robotPlot_h = subplot(1, 2, 1);

            % create ur3
            obj.ur3 = UR3m(trotz(pi/2));

            % set view properties
            view(obj.robotPlot_h, 30, 20);
            daspect(obj.robotPlot_h,[1 1 1]);

            % PLOT 2
            obj.trajPlot_h = subplot(1, 2, 2);
            
            obj.trajXs = [0];
            obj.trajYs = [0];

            obj.trajLine_h = plot(obj.trajPlot_h, obj.trajXs, obj.trajYs);
            grid(obj.trajPlot_h, 'on');
            title(obj.trajPlot_h, 'Trajectory 2D x,y');
            xlabel(obj.trajPlot_h, 'X (m)');
            ylabel(obj.trajPlot_h, 'Y (m)');
            daspect(obj.trajPlot_h, [1 1 1]);
            set(gcf,'color','w');
            xlim(obj.trajPlot_h, [0 1.2]);
            ylim(obj.trajPlot_h, [0 1.2])
            pos = get(gca, 'OuterPosition');
            set(gca, 'OuterPosition', pos);
            
            %CUP LOCATION DATA FIELDS
            obj.cupLocationXLabel = uicontrol('Style', 'text', 'String', 'x-goal', 'position', [20 120 100 15]);
            obj.cupLocationXField = uicontrol('Style', 'text', 'String', num2str(0), 'position', [20 80 100 30]);

            obj.cupLocationYLabel = uicontrol('Style', 'text','String', 'y-goal', 'position', [150 120 100 15]);
            obj.cupLocationYField = uicontrol('Style', 'text','String', num2str(0), 'position', [150 80 100 30]);

            obj.cupLocationZLabel = uicontrol('Style', 'text','String', 'z-goal', 'position', [280 120 100 15]);
            obj.cupLocationZField = uicontrol('Style', 'text','String', num2str(0), 'position', [280 80 100 30]);

            %BUTTONS
            %create fire button
            obj.fireButton = uicontrol('String', 'Launch!', 'position', [400 120 100 30]);
            %attach button callback
            %CALLBACK

            %create Open Servo button
            obj.openButton = uicontrol('String', 'Open Servo', 'position', [400 80 100 30]);
            %attach button callback
            obj.openButton.Callback = @obj.onOpenServoButton;


            %create Close servo button
            obj.closeButton = uicontrol('String', 'Close Servo', 'position', [510 120 100 30]);
            %attach button callback
            obj.closeButton.Callback = @obj.onCloseServoButton;

            %create return home button
            obj.homeButton = uicontrol('String', 'Return Home', 'position', [510 80 100 30]);
            %attach button callback
            %CALLBACK

            %create get Cup Pose button button
            obj.getCupPoseButton = uicontrol('String', 'Get Cup Pose', 'position', [620 120 100 30]);
            %attach button callback
            obj.getCupPoseButton.Callback = @obj.onGetCupPoseButton;

            %create abort button
            obj.abortButton = uicontrol('String', 'Abort', 'position', [620 80 100 30]);
            %attach button callback
            %CALLBACK

            %create exit button
            obj.exitButton = uicontrol('String', 'Exit', 'position', [730 120 100 30]);
            %attach button callback
            obj.exitButton.Callback = @obj.onExitButton;

            %create exit button
            obj.xPlusButton = uicontrol('String', 'X+', 'position', [1100 90 100 30]);
            %attach button callback
            obj.xPlusButton.Callback = @obj.onXPlusButton;

            %create exit button
            obj.xMinusButton = uicontrol('String', 'X-', 'position', [900 90 100 30]);
            %attach button callback
            obj.xMinusButton.Callback = @obj.onXMinusButton;

            %create exit button
            obj.zPlusButton = uicontrol('String', 'Z+', 'position', [1000 120 100 30]);
            %attach button callback
            obj.zPlusButton.Callback = @obj.onZPlusButton;

            %create exit button
            obj.zMinusButton = uicontrol('String', 'Z-', 'position', [1000 60 100 30]);
            %attach button callback
            obj.zMinusButton.Callback = @obj.onZMinusButton;

            %create exit button
            obj.yPlusButton = uicontrol('String', 'Y+', 'position', [1100 120 100 30]);
            %attach button callback
            obj.yPlusButton.Callback = @obj.onYPlusButton;

            %create exit button
            obj.yMinusButton = uicontrol('String', 'Y-', 'position', [900 120 100 30]);
            %attach button callback
            obj.yMinusButton.Callback = @obj.onYMinusButton;
        end
    end
end

