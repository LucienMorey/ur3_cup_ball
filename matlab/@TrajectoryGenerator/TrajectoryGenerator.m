classdef TrajectoryGenerator < handle
    %TRAJECTORYGENERATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot;
        throwPosition;
        preThrowDistance;
        postThrowDistance;
        reloadLocation;
        cartesianTrajectory;
        cartesianWaypoints
    end

    properties(Constant)
        EPSILON = 1.0;
        VELOCITY_WEIGHTING = diag([1,1,1,1,1,0.1]);
        STEPS = 50;
    end
    
    methods
        function obj = TrajectoryGenerator(robot, throwPosition, preThrowDistance, postThrowDistance, reloadLocation)
            %TRAJECTORYGENERATOR Construct an instance of this class
            %   Detailed explanation goes here
            % set the robot
            obj.robot = robot;

            % set throw location
            obj.throwPosition = throwPosition;

            % set desired distance before throw
            obj.preThrowDistance = preThrowDistance;

            % set desired distance after throw
            obj.postThrowDistance = postThrowDistance;

            % set reload location
            obj.reloadLocation = reloadLocation;
        end
        
        function [qMatrix,vMatrix, tMatrix] = GenerateTrajectory(obj, velocityVector)
            qMatrix = [];
            vMatrix = [];
            tMatrix = [];
            
            % velocity magnitude
            velocityMagnitude = norm(velocityVector);

            % get velocity unit vector
            velocityDirection = velocityVector / velocityMagnitude;

            % get rotation matrix, z-axis in line with velocity vector
            obj.throwPosition(1:3,1:3) = obj.doubleCross(velocityDirection);
            
            % Get start and end of throw, by extending velocity direcion by pre-throw dist and post-throw dist
            % Rotation component is not touched to ensure end-effector stays straight
            pStart = obj.throwPosition + [zeros(3,3), -obj.preThrowDistance * velocityDirection; zeros(1,4)];
            pEnd   = obj.throwPosition + [zeros(3,3), obj.postThrowDistance * velocityDirection; zeros(1,4)]; 
            
            % concatinate all points in trajectory
            % five 4x4 transforms
            obj.cartesianWaypoints = zeros(4,4,5);
            obj.cartesianWaypoints(:,:,1) = obj.reloadLocation;
            obj.cartesianWaypoints(:,:,2) = pStart;
            obj.cartesianWaypoints(:,:,3) = obj.throwPosition;
            obj.cartesianWaypoints(:,:,4) = pEnd;
            obj.cartesianWaypoints(:,:,5) = obj.reloadLocation;
            obj.cartesianWaypoints

            % calc vars
            x = [];
            theta = [];
            trajectoryDeltaT = [];

            % For each segment of the trajectory
            for i = 1:size(obj.cartesianWaypoints, 3) - 1

                % get the interpolated segment
                [x_local,theta_local, t_segment] = obj.interpolateSegment(obj.cartesianWaypoints(:,:,i), obj.cartesianWaypoints(:,:,i+1), velocityMagnitude);

                % Add the segments to the total path
                x = [x, x_local];
                theta = [theta, theta_local];
                trajectoryDeltaT = [trajectoryDeltaT, t_segment];
            end

            % total path
            obj.cartesianTrajectory = [x; theta];


            [qMatrix, vMatrix, tMatrix] = obj.GenerateRMRCSegment(obj.cartesianTrajectory, trajectoryDeltaT, ones(1,6));

        end

        % TODO change to take in theta matrix x matrix delta time matrix and joint seed
        function [qMatrix, vMatrix, tMatrix] = GenerateRMRCSegment(obj, cartesianTrajectory, pointTimeDelta, jointSeed)
            % Preallocate return arrays  
            qMatrix = zeros(size(cartesianTrajectory,2), obj.robot.n);
            vMatrix = zeros(size(cartesianTrajectory,2), obj.robot.n);
            tMatrix = zeros(1, size(cartesianTrajectory,2));
            
            
            % populate segment time array 
            tMatrix(1) = 0;
            for i=2:1:size(cartesianTrajectory,2)
                tMatrix(1, i) = tMatrix(1, i-1) + pointTimeDelta(1,i);
            end

            
            % get transform of first point
            T =  [rpy2r(cartesianTrajectory(4,1), cartesianTrajectory(4,2), cartesianTrajectory(4,1)) cartesianTrajectory(1:3,1);zeros(1,3) 1];
            % obj.orientation = cat(3, obj.orientation, T);

            % initial joint state
            qMatrix(1,:) = obj.robot.ikcon(T,jointSeed);

            % create joint state traj
            for i=1:1:size(cartesianTrajectory,2)-1
                % determine forward kinematics solution of current joint states
                currentT = obj.robot.fkine(qMatrix(i,:));
      
                % determine the position delta to the next traj point
                deltaX = cartesianTrajectory(1:3,i+1) - currentT(1:3,4);

                % determine linear velocity for this timestep
                linearVelocity = (1/pointTimeDelta(1,i))*deltaX;

                % determine current rotation matrix
                currentR = currentT(1:3,1:3);

                % determine future rotation matrix
                nextR = rpy2r(cartesianTrajectory(4,i+1),cartesianTrajectory(5,i+1),cartesianTrajectory(6,i+1));

                % determine rotation change over time
                deltaR = (1/pointTimeDelta(1,i))*(nextR - currentR);

                % skew the matrix and determine the angular velocity
                S = deltaR*currentR';
                angularVelocity = [S(3,2);S(1,3);S(2,1)];
                % concatinate velocity vectors and scale
                cartesianVelocity = obj.VELOCITY_WEIGHTING * [linearVelocity; angularVelocity];

                % determine the current jacobian
                J = obj.robot.jacob0(qMatrix(i,:));

                % determine the current measure of manipulibility
                m = sqrt(det(J*J'));

                % check if a damped least squares solution is required
                if m < obj.EPSILON
                    % least squares
                    lambda = (1 - m/obj.EPSILON)*5E-2;
                else
                    % not required
                    lambda = 0;
                end
                
                % apply least squares if required and invert jacobian
                invJ = inv(J'*J + lambda *eye(6))*J';

                % determine joint velocities
                vMatrix(i,:) = invJ*cartesianVelocity;

                % check if expected to exceed joint limits
                for j=1:1:obj.robot.n % Loop through joints 1 to 6
                    if qMatrix(i,j) + pointTimeDelta(1,i)*vMatrix(i,j) < obj.robot.qlim(j,1) % If next joint angle is lower than joint limit...
                        vMatrix(i, j) = ( obj.robot.qlim(j,1) - qMatrix(i,j) )/ pointTimeDelta(1,i);
                        %vMatrix(i,j) = 0;  % Stop the motor
                    elseif qMatrix(i,j) + pointTimeDelta(1,i)*vMatrix(i,j) > obj.robot.qlim(j,2) % If next joint angle is greater than joint limit ...
                        vMatrix(i, j) = ( obj.robot.qlim(j,1) - qMatrix(i,j) )/ pointTimeDelta(1,i);
                        %vMatrix(i,j) = 0; % Stop the motor
                    end
                end

                % update next joint state in qMatrix
                qMatrix(i+1,:) = qMatrix(i,:) + pointTimeDelta(1,i)*vMatrix(i,:);
            end

        end

        function [xMatrix, thetaMatrix, tMatrix] = interpolateSegment(obj, segmentStart, segmentEnd, velocityMagnitude)
            
            % segment delta x,y,z
            seg = segmentEnd(1:3,4) - segmentStart(1:3,4);
            
            % segment length
            segMagnitude = norm(seg);
            
            % segment unit vector
            segDirection = seg / segMagnitude;

            % time interval for each step
            dt = segMagnitude / velocityMagnitude / obj.STEPS;

            % xmatrix -> x,y,z positions of segment
            % thetaMatrix -> rpy of segment
            xMatrix     = zeros(3, obj.STEPS);
            thetaMatrix = zeros(3, obj.STEPS);
            tMatrix     = repmat(dt, 1, obj.STEPS);

            % get the rpy increment for each step
            startRPY = tr2rpy(segmentStart(1:3,1:3))';
            endRPY = tr2rpy(segmentEnd(1:3,1:3))';
            rpyIncrement = (endRPY - startRPY) / obj.STEPS;

            % interpolate translation and rotation
            for i = 1:obj.STEPS
                xMatrix(:,i)     = segmentStart(1:3,4) + (i-1)*(velocityMagnitude*dt)*segDirection;
                thetaMatrix(:,i) = startRPY + (i-1)*rpyIncrement;
            end
        end

        function rMatrix = doubleCross(obj, zLocal)
            % arbitrary y is determined by taking the cross product of local z and the global z;
            yLocal = cross(zLocal, [0; 0; 1]);
            
            % subsequent x vector can be found with the cross from y to z local
            xLocal = cross(yLocal, zLocal);
            
            % determine rpy from rotation matrix
            rMatrix = [xLocal, yLocal, zLocal];
        end
    end
end
