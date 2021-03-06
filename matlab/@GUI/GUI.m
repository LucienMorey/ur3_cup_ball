classdef GUI < matlab.apps.AppBase & handle
    %GUI Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        
    end

    properties(Constant)
        SUBSCIRIBER_TIMEOUT = 0.5;
        CARTESIAN_JOG_DIST =0.05;
        JOINT_JOG_ANGLE = pi/36;
        UR3_JOINT_NAMES = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
        PROJECTILE_MASS = 0.0027;
        PROJETILE_DIAMETER = 0.04;
        COEFFICENT_OF_DRAG = 0;
        COEFFICIENT_OF_RESTITUTION = 0.7;
        LAUNCH_POSITION = transl([-0.15, 0.3, 0.4]); % These positions are relative to the robot
        RELOAD_POSITION = transl([-0.0,0.4,0.4]);
        LAUNCH_VELOCITY_MAGNITUDE = 1;
        DESIRED_NUMBER_OF_BOUNCES = 1;
        HEIGHT_OF_CUP = 0.12;
        NETWORK_BUFFER_TIME = 0.6;
        GROUND_NORMAL = [0, 0, 1];
        GROUND_POINT = [0, 0, 0];
        PRE_THROW_DISTANCE = 0.3;
        POST_THROW_DISTANCE = 0.1;
    end
    
    properties(Access = private)
        fig;

        robotPlot_h;
        trajPlot_h;
        
        traj2DLine_h;
        traj3DLine_h;
        cupLocation2D_h;
        cupLocation3D_h;
        throwLocation3D_h;
        reloadLocation3D_h;
        robotLine_h
        
        cupLocationXField
        cupLocationYField
        cupLocationZField
        cupLocationXLabel
        cupLocationYLabel
        cupLocationZLabel
        
        
        openButton
        closeButton
        homeButton
        calcTrajButton
        executeActionButton;
        abortButton;
        exitButton;
        xPlusButton;
        xMinusButton;
        yPlusButton;
        yMinusButton;
        zPlusButton;
        zMinusButton;
        q1PlusButton;
        q1MinusButton;
        q2PlusButton;
        q2MinusButton;
        q3PlusButton;
        q3MinusButton;
        q4PlusButton;
        q4MinusButton;
        q5PlusButton;
        q5MinusButton;
        q6PlusButton;
        q6MinusButton;

        gamePadJog;

        servoPublisher;
        estopSubscriber;
        jointStateSubscriber;
        gamePadSubscriber
        tree;
        cupRobotFrame;
        robotWorldFrame
        actionClient;
        trajGoal;
        
        ur3;
        table_top;
        mounting_plate;
        cup;
        trajectoryGenerator;
        projectileGenerator;
        qMatrix;
        vMatrix;
        tMatrix;
    end
    
    methods
        function obj = GUI()
            %GUI Construct an instance of this class
            %   Detailed explanation goes here
            
            rosinit();
            obj.tree = rostf;
            
            obj.guiElementGenerate();

            obj.servoPublisher = rospublisher('servo_closed_state', 'std_msgs/Bool', 'IsLatching', false);
            obj.estopSubscriber = rossubscriber('estop', 'std_msgs/Bool');
            obj.jointStateSubscriber = rossubscriber('joint_states', 'sensor_msgs/JointState');
            [obj.actionClient, obj.trajGoal] = rosactionclient('/arm_controller/follow_joint_trajectory');
            obj.gamePadSubscriber = rossubscriber('joystick', 'sensor_msgs/Joy');
            obj.trajGoal.Trajectory.JointNames = obj.UR3_JOINT_NAMES;
            obj.trajGoal.GoalTimeTolerance = rosduration(0.05);

            obj.ur3.model.animate([0, 0, 0, 0, 0, 0]);
            obj.cupRobotFrame = NaN(4);

            obj.trajectoryGenerator = TrajectoryGenerator(obj.ur3.model, obj.robotWorldFrame*obj.LAUNCH_POSITION, obj.PRE_THROW_DISTANCE, obj.POST_THROW_DISTANCE, obj.robotWorldFrame*obj.RELOAD_POSITION);
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
            %shutdown ros and delete the gui
            rosshutdown;
            delete(obj.fig);
        end

        function onHomeButton(obj, app, event)
            % current pose
            q = obj.getJointState();

            qh = [0, -pi/2, 0, -pi/2, 0, 0]

            % error to home pose
            %qe = zeros(1, 6) - q;
            qe = qh - q;
            
            % jjog
            [obj.qMatrix, obj.vMatrix, obj.tMatrix] = obj.trajectoryGenerator.jjog(q, qe);
            obj.executeActionButton.Enable = 'on';
            for i=1:1:size(obj.qMatrix,1)
                obj.ur3.model.animate(obj.qMatrix(i,:));
                pause(0.04);
            end
            drawnow();

        end

        function onCalcTrajButton(obj, app, event)
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

                transformedPose = transform(obj.tree, 'world', pose);
                %create rotation matrix from tranformed quaternion
                rotm = quat2rotm([transformedPose.Pose.Orientation.W, transformedPose.Pose.Orientation.X, transformedPose.Pose.Orientation.Y, transformedPose.Pose.Orientation.Z]);
                %compound rotation matrix and translation into homogenous transform
                obj.cupRobotFrame = [rotm, [transformedPose.Pose.Position.X; transformedPose.Pose.Position.Y; transformedPose.Pose.Position.Z]; zeros(1,3), 1];
                % set ui element info
                obj.cupLocationXField.String = num2str(obj.cupRobotFrame(1,4));
                obj.cupLocationYField.String = num2str(obj.cupRobotFrame(2,4));
                obj.cupLocationZField.String = num2str(obj.cupRobotFrame(3,4));

            catch
                disp('tf error check for joined trees');
                % set ui element info
                obj.cupLocationXField.String = 'error';
                obj.cupLocationYField.String = 'error';
                obj.cupLocationZField.String = 'error';
                return
            end

%             try
                % transform relative reloaf and launch pose into world frame                
                reload = obj.robotWorldFrame * obj.RELOAD_POSITION;
                launch = obj.robotWorldFrame * obj.LAUNCH_POSITION;

                % extract position from pose
                reload = reload(1:3,4)';
                launch = launch(1:3,4)';
                

                %add arbitrary height for cup height
                obj.cupRobotFrame(3,4) = obj.HEIGHT_OF_CUP;
                cup = obj.cupRobotFrame(1:3,4)';
                obj.cup.pose = obj.cupRobotFrame;
                obj.cup.animate();

                % initial velocity & simulate
                [vThrow] = obj.projectileGenerator.calcLaunch(launch, obj.cupRobotFrame(1:3,4)', obj.DESIRED_NUMBER_OF_BOUNCES, obj.LAUNCH_VELOCITY_MAGNITUDE);
                xyz = obj.projectileGenerator.simulateP(launch, vThrow, obj.DESIRED_NUMBER_OF_BOUNCES);

                % 3d plot
                axes(obj.robotPlot_h)
                [obj.traj3DLine_h.XData, obj.traj3DLine_h.YData, obj.traj3DLine_h.ZData] = deal(xyz(:, 1), xyz(:, 2), xyz(:, 3));
                [obj.cupLocation3D_h.XData, obj.cupLocation3D_h.YData, obj.cupLocation3D_h.ZData] = deal(cup(1),cup(2),cup(3));
                [obj.throwLocation3D_h.XData, obj.throwLocation3D_h.YData, obj.throwLocation3D_h.ZData] = deal(launch(1),launch(2),launch(3));
                [obj.reloadLocation3D_h.XData, obj.reloadLocation3D_h.YData, obj.reloadLocation3D_h.ZData] = deal(reload(1),reload(2),reload(3));

                % calculate 2d points from 3d ones
                xPoints = sqrt(xyz(:, 1).^2 + xyz(:, 2).^2);
                yPoints = xyz(:, 3);
                cup2dx = sqrt(cup(1)^2 + cup(2)^2);
                cup2dy = cup(3);

                % 2D plot
                axes(obj.trajPlot_h);
                [obj.traj2DLine_h.XData, obj.traj2DLine_h.YData] = deal(xPoints, yPoints);

                [obj.cupLocation2D_h.XData, obj.cupLocation2D_h.YData] = deal(cup2dx, cup2dy);

                % SEND CALCULATED TRAJ
                % using velocity vector and throw location
                q = obj.getJointState();
                [obj.qMatrix, obj.vMatrix, obj.tMatrix, xMatrix] = obj.trajectoryGenerator.GenerateThrow(vThrow, q);

                % calculate trajectory
                axes(obj.robotPlot_h);

                [obj.robotLine_h.XData, obj.robotLine_h.YData, obj.robotLine_h.ZData] = deal(xMatrix(:,1),xMatrix(:,2), xMatrix(:,3));
                obj.executeActionButton.Enable = 'on';
                drawnow();
                for i=1:1:size(obj.qMatrix,1)
                    obj.ur3.model.animate(obj.qMatrix(i,:));
                    pause(0.01);
                end;


                % Perform collision checking of trajectory
                collision = checkCollision(obj.ur3, obj.qMatrix, obj.GROUND_POINT, obj.GROUND_NORMAL);
                if collision
                    obj.executeActionButton.Enable = 'off';
                else
                    obj.executeActionButton.Enable = 'on';
                end

            % catch broken tf tree
%             catch
%                 disp('unable to calculate and plot trajectory');
%             end
        end

        function onGamePad(obj, app, event)
            cls = 0;
            obj.executeActionButton.Enable = 'off';
            obj.gamePadJog.Enable = 'off';
            try
                while cls == 0 %make this close on the clear button
                    %    vz = joy.axis[1];
                    %    vx = joy.axis[2];
                    %    vy = joy.axis[3];
                    %    r  = joy.axis[1];
                    %    p = joy.axis[2];
                    %    y = joy.axis[3];
                    msg = receive(obj.gamePadSubscriber ,obj.SUBSCIRIBER_TIMEOUT);
                    axis_value = (msg.Axes);
                    button_value = (msg.Buttons);

                    xyz2rpy = button_value(2); %Left trigger
                    % TODO implement jogging in rpy
                    cls = button_value(9); %Back button will close out of joystick mode
                    
                    %check for the maximum sjoystick value so robot moves in one direction at a time
                    [max_axis_value,max_axis_value_index] = max(abs(axis_value));

                    %robot will move 10 mm in the desired direction
                    switch max_axis_value_index
                        case 1
                        disp('jog X');
                        obj.cartesianJog([axis_value(max_axis_value_index)/100.0,0, 0]);
                        case 2
                        disp('jog Y');
                        obj.cartesianJog([0,axis_value(max_axis_value_index)/100, 0]);
                        case 4
                        disp('jog Z');
                        obj.cartesianJog([0,0, -axis_value(max_axis_value_index)/100]);
                    otherwise
                        disp('error in joystick')
                    end

                    %create a ros message and send the trajectory
                    obj.makeTrajMsg(obj.qMatrix, obj.vMatrix, obj.tMatrix);
                    obj.trajGoal.Trajectory.Header.Stamp = rostime('now') + rosduration(obj.NETWORK_BUFFER_TIME);
                    try
                        sendGoalAndWait(obj.actionClient, obj.trajGoal);    
                    catch
                        disp('Action sever error');
                    end
                end
            catch
                disp('fail')
            end
            obj.gamePadJog.Enable = 'on';
        end

        function onAbortButton(obj, app, event)
            %cancel the current action goal and disable the abort button untl another goal is sent
            cancelGoal(obj.actionClient);  
            obj.abortButton.Enable = 'off';  
        end

        function onExecuteActionButton(obj, app, event)
            %convert created trajectory into ros message
            obj.makeTrajMsg(obj.qMatrix, obj.vMatrix, obj.tMatrix);
            %give message a current time stamp with a slight buffer for network delay
            obj.trajGoal.Trajectory.Header.Stamp = rostime('now') + rosduration(obj.NETWORK_BUFFER_TIME);
            %try to send an action goal
            %this will also disable the  execute action button until another trajectory has been calculated
            try
                sendGoal(obj.actionClient, obj.trajGoal);
                obj.executeActionButton.Enable = 'off';
                obj.abortButton.Enable = 'on';       
            catch
                disp('Action sever error');
                return
            end

            
            %try to fire the servo
            fired = false;
            while fired == false
                %try to get the current joint state and determine the distance to the launch position
                try
                    current_pose = obj.ur3.model.fkine(obj.getJointState());
                    robot_throw_pose = (obj.robotWorldFrame*obj.LAUNCH_POSITION);
                    distanceToFirePosition = robot_throw_pose(1:3,4) - current_pose(1:3,4);
                    
                    %if the end effector is within 25mm of launch position then throw the ball
                    if norm(distanceToFirePosition) < 0.025
                        % open servo
                        servoState = rosmessage('std_msgs/Bool');
                        servoState.Data = true;
                        send(obj.servoPublisher, servoState);
                        fired = true;
                    end
                    disp(norm(distanceToFirePosition));
                    
                catch
                    disp('subscriber timeout');
                end
            end
            
        end

        function onXPlusButton(obj, app, event)
            disp('jog XPlus');
            obj.executeActionButton.Enable = 'on';
            obj.cartesianJog([obj.CARTESIAN_JOG_DIST,0, 0]);
        end

        function onXMinusButton(obj, app, event)
            disp('jog XMinus');
            obj.executeActionButton.Enable = 'on';
            obj.cartesianJog([-obj.CARTESIAN_JOG_DIST,0, 0]);
        end

        function onYPlusButton(obj, app, event)
            disp('jog YPlus');
            obj.executeActionButton.Enable = 'on';
            obj.cartesianJog([0,obj.CARTESIAN_JOG_DIST, 0]);
        end

        function onYMinusButton(obj, app, event)
            disp('jog YMinus');
            obj.executeActionButton.Enable = 'on';
            obj.cartesianJog([0,-obj.CARTESIAN_JOG_DIST, 0]);
        end

        function onZPlusButton(obj, app, event)
            disp('jog ZPlus');
            obj.executeActionButton.Enable = 'on';
            obj.cartesianJog([0,0, obj.CARTESIAN_JOG_DIST]);
        end

        function onZMinusButton(obj, app, event)
            disp('jog ZMinus');
            obj.executeActionButton.Enable = 'on';
            obj.cartesianJog([0,0, -obj.CARTESIAN_JOG_DIST]);
        end

        function onQ1PlusButton(obj,event, app)
            disp('jog q1Plus');
            obj.executeActionButton.Enable = 'on';
            obj.jointJog(1,obj.JOINT_JOG_ANGLE);
        end

        function onQ1MinusButton(obj,event, app)
            disp('jog q1Minus');
            obj.executeActionButton.Enable = 'on';
            obj.jointJog(1,-obj.JOINT_JOG_ANGLE);
        end

        function onQ2PlusButton(obj,event, app)
            disp('jog q2Plus');
            obj.executeActionButton.Enable = 'on';
            obj.jointJog(2,obj.JOINT_JOG_ANGLE);
        end

        function onQ2MinusButton(obj,event, app)
            disp('jog q2Minus');
            obj.executeActionButton.Enable = 'on';
            obj.jointJog(2,-obj.JOINT_JOG_ANGLE);
        end

        function onQ3PlusButton(obj,event, app)
            disp('jog q3Plus');
            obj.executeActionButton.Enable = 'on';
            obj.jointJog(3,obj.JOINT_JOG_ANGLE);
        end

        function onQ3MinusButton(obj,event, app)
            disp('jog q3Minus');
            obj.executeActionButton.Enable = 'on';
            obj.jointJog(3,-obj.JOINT_JOG_ANGLE);
        end

        function onQ4PlusButton(obj,event, app)
            disp('jog q4Plus');
            obj.executeActionButton.Enable = 'on';
            obj.jointJog(4,obj.JOINT_JOG_ANGLE);
        end

        function onQ4MinusButton(obj,event, app)
            disp('jog q4Minus');
            obj.executeActionButton.Enable = 'on';
            obj.jointJog(4,-obj.JOINT_JOG_ANGLE);
        end

        function onQ5PlusButton(obj,event, app)
            disp('jog q5Plus');
            obj.executeActionButton.Enable = 'on';
            obj.jointJog(5,obj.JOINT_JOG_ANGLE);
        end

        function onQ5MinusButton(obj,event, app)
            disp('jog q1Minus');
            obj.executeActionButton.Enable = 'on';
            obj.jointJog(5,-obj.JOINT_JOG_ANGLE);
        end

        function onQ6PlusButton(obj,event, app)
            disp('jog q6Plus');
            obj.executeActionButton.Enable = 'on';
            obj.jointJog(6,obj.JOINT_JOG_ANGLE);
        end

        function onQ6MinusButton(obj,event, app)
            disp('jog q6Minus');
            obj.executeActionButton.Enable = 'on';
            obj.jointJog(6,-obj.JOINT_JOG_ANGLE);
        end

        function cartesianJog(obj, direction)
            axes(obj.robotPlot_h);

            % get the current joint state and then jog from the current state in the desired direction
            q = obj.getJointState();
            % only jog if the current joint state was sucessfully obtained
            if ~isempty(q)
                [obj.qMatrix, obj.vMatrix, obj.tMatrix] = obj.trajectoryGenerator.cjog(q, direction);
                for i=1:1:size(obj.qMatrix,1)
                    obj.ur3.model.animate(obj.qMatrix(i,:));
                    pause(0.01);
                end;
            end
        end

        function jointJog(obj, jointNumber, jogAmount)
            axes(obj.robotPlot_h);

            % get the current joint state and then jog from the current state in the desired direction
            q = obj.getJointState();

            % only jog if the current joint state could be obtained
            if ~isempty(q)

                qDesired = zeros(1,size(q,2));
                qDesired(1,jointNumber) = jogAmount;

                [obj.qMatrix, obj.vMatrix, obj.tMatrix] = obj.trajectoryGenerator.jjog(q, qDesired)
                for i=1:1:size(obj.qMatrix,1)
                    obj.ur3.model.animate(obj.qMatrix(i,:));
                    pause(0.01);
                end;
            end
        end

        function makeTrajMsg(obj, q, v, t)
            %empty the current trajectory points array and refill it with new data
            obj.trajGoal.Trajectory.Points = [];
            for i=1:1:size(q,1)
                trajPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                trajPoint.Positions = q(i,:);
                trajPoint.Velocities = v(i,:);
                trajPoint.TimeFromStart = rosduration(t(i,1));
                obj.trajGoal.Trajectory.Points = [obj.trajGoal.Trajectory.Points; trajPoint];
            end
        end

        function q = getJointState(obj)
            try
                %get the latest joint state message
                msg = receive(obj.jointStateSubscriber,obj.SUBSCIRIBER_TIMEOUT);
                q321456 = (msg.Position)';

                % Note the default order of the joints is 3,2,1,4,5,6 and needs to be rearranged
                q = [q321456(1,3:-1:1), q321456(1,4:6)];
            catch
                q = [];
                disp('No message received');
            end
        end

        function guiElementGenerate(obj)
            %PLOT 1
            obj.fig = figure('units','normalized','outerposition',[0 0 1 1]);
            %attatch close button callback to close gui as well so ros is always shutdown
            obj.fig.CloseRequestFcn = @obj.onExitButton;
            
            % UR3 subplot
            obj.robotPlot_h = subplot(1, 2, 1);

            success = false;
            %block ntil the robot can be plotted sucessfully
            while success == false
                try
                    pose = rosmessage('geometry_msgs/PoseStamped');
                    pose.Header.FrameId = 'base_link';
                    pose.Pose.Position.X = 0.0;
                    pose.Pose.Position.Y = 0.0;
                    pose.Pose.Position.Z = 0.0;
                    pose.Pose.Orientation.X = 0.0;
                    pose.Pose.Orientation.Y = 0.0;
                    pose.Pose.Orientation.Z = 0.0;
                    pose.Pose.Orientation.W = 1.0;

                    transformedPose = transform(obj.tree, 'world', pose);
                    %create rotation matrix from tranformed quaternion
                    rotm = quat2rotm([transformedPose.Pose.Orientation.W, transformedPose.Pose.Orientation.X, transformedPose.Pose.Orientation.Y, transformedPose.Pose.Orientation.Z]);
                    %compound rotation matrix and translation into homogenous transform
                    obj.robotWorldFrame = [rotm, [transformedPose.Pose.Position.X; transformedPose.Pose.Position.Y; transformedPose.Pose.Position.Z]; zeros(1,3), 1];
                    % first transform is always 0 translation so wait until a better result is recieved
                    if obj.robotWorldFrame(1:3,4) ~= [0; 0; 0]
                        success = true;
                    end

                    % set robot Z distance to 0 because it is known to be on the table
                    obj.robotWorldFrame(3, 4) = 0.0;
                catch
                    disp('error getting robot transform. will keep retrying')
                end
            end
                
            % create ur3
            obj.robotWorldFrame(3,4) = 0.0;
            obj.ur3 = UR3m(obj.robotWorldFrame);

            % set view properties
            hold(obj.robotPlot_h, 'on');
            view(obj.robotPlot_h, 30, 20);
            daspect(obj.robotPlot_h,[1 1 1]);

            % create empty plots for all lines  so be adjusted once new data comes in
            obj.traj3DLine_h = plot3([0], [0], [0]);
            obj.cupLocation3D_h = plot3([0], [0], [0],'ro');
            obj.throwLocation3D_h = plot3([0], [0], [0],'ro');
            obj.reloadLocation3D_h = plot3([0], [0], [0],'ro');
            obj.robotLine_h = plot3([0],[0], [0]);
            xlim(obj.robotPlot_h, [-1.5, 1]);
            ylim(obj.robotPlot_h, [-1, 1]);
            zlim(obj.robotPlot_h, [-.04, 1]);
            
            %create the environment
            obj.table_top = Environment(transl(-1.1, 0.55, 0.001),'table top.PLY');
            obj.mounting_plate = Environment(obj.robotWorldFrame*transl(0, 0, 0.005),'metal table thing.PLY');
            obj.cup = Environment(transl(0, 0, 0) * rpy2tr(0, 0, 0, 'deg'),'cup.PLY')

            % PLOT 2
            obj.trajPlot_h = subplot(1, 2, 2);

            obj.traj2DLine_h = plot([0], [0]);
            hold(obj.trajPlot_h, 'on');
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

            obj.cupLocation2D_h = plot([0], [0], 'ro');
            
            %CUP LOCATION DATA FIELDS
            obj.cupLocationXLabel = uicontrol('Style', 'text', 'String', 'x-goal', 'position', [20 120 100 15]);
            obj.cupLocationXField = uicontrol('Style', 'text', 'String', num2str(0), 'position', [20 80 100 30]);

            obj.cupLocationYLabel = uicontrol('Style', 'text','String', 'y-goal', 'position', [150 120 100 15]);
            obj.cupLocationYField = uicontrol('Style', 'text','String', num2str(0), 'position', [150 80 100 30]);

            obj.cupLocationZLabel = uicontrol('Style', 'text','String', 'z-goal', 'position', [280 120 100 15]);
            obj.cupLocationZField = uicontrol('Style', 'text','String', num2str(0), 'position', [280 80 100 30]);

            %BUTTONS
            %create fire button
            obj.executeActionButton = uicontrol('String', 'Execute!', 'position', [400 80 100 30], 'Enable', 'off');
            %attach button callback
            obj.executeActionButton.Callback = @obj.onExecuteActionButton;

            %create Open Servo button
            obj.openButton = uicontrol('String', 'Open Servo', 'position', [510 120 100 30]);
            %attach button callback
            obj.openButton.Callback = @obj.onOpenServoButton;

            %create Close servo button
            obj.closeButton = uicontrol('String', 'Close Servo', 'position', [510 80 100 30]);
            %attach button callback
            obj.closeButton.Callback = @obj.onCloseServoButton;

            %create return home button
            obj.homeButton = uicontrol('String', 'Return Home', 'position', [620 120 100 30]);
            %attach button callback
            obj.homeButton.Callback = @obj.onHomeButton;

            %create get Cup Pose button button
            obj.calcTrajButton = uicontrol('String', 'Calc Traj', 'position', [400 120 100 30]);
            %attach button callback
            obj.calcTrajButton.Callback = @obj.onCalcTrajButton;

            %create abort button
            obj.abortButton = uicontrol('String', 'Abort', 'position', [620 80 100 30]);
            %attach button callback
            obj.abortButton.Callback = @obj.onAbortButton;

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
            
            %create gamepad control jog
            obj.gamePadJog = uicontrol('String', 'GamePadEnable', 'position', [1000 90 100 30]);
            obj.gamePadJog.Callback = @obj.onGamePad

            %create joint jog buttons
            obj.q1PlusButton = uicontrol('String', 'q1+', 'position', [1300 120 100 30]);
            obj.q1PlusButton.Callback = @obj.onQ1PlusButton;
            obj.q1MinusButton = uicontrol('String', 'q1-', 'position', [1400 120 100 30]);
            obj.q1MinusButton.Callback = @obj.onQ1MinusButton;

            obj.q2PlusButton = uicontrol('String', 'q2+', 'position', [1300 90 100 30]);
            obj.q2PlusButton.Callback = @obj.onQ2PlusButton;
            obj.q2MinusButton = uicontrol('String', 'q2-', 'position', [1400 90 100 30]);
            obj.q2MinusButton.Callback = @obj.onQ2MinusButton;

            obj.q3PlusButton = uicontrol('String', 'q3+', 'position', [1300 60 100 30]);
            obj.q3PlusButton.Callback = @obj.onQ3PlusButton;
            obj.q3MinusButton = uicontrol('String', 'q3-', 'position', [1400 60 100 30]);
            obj.q3MinusButton.Callback = @obj.onQ3MinusButton;

            obj.q4PlusButton = uicontrol('String', 'q4+', 'position', [1550 120 100 30]);
            obj.q4PlusButton.Callback = @obj.onQ4PlusButton;
            obj.q4MinusButton = uicontrol('String', 'q4-', 'position', [1650 120 100 30]);
            obj.q4MinusButton.Callback = @obj.onQ4MinusButton;

            obj.q5PlusButton = uicontrol('String', 'q5+', 'position', [1550 90 100 30]);
            obj.q5PlusButton.Callback = @obj.onQ5PlusButton;
            obj.q5MinusButton = uicontrol('String', 'q5-', 'position', [1650 90 100 30]);
            obj.q5MinusButton.Callback = @obj.onQ5MinusButton;

            obj.q6PlusButton = uicontrol('String', 'q6+', 'position', [1550 60 100 30]);
            obj.q6PlusButton.Callback = @obj.onQ6PlusButton;
            obj.q6MinusButton = uicontrol('String', 'q6-', 'position', [1650 60 100 30]);
            obj.q6MinusButton.Callback = @obj.onQ6MinusButton;
        end
    end
end

