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
        JOG_VEL = 0.1;
        JOG_STEPS = 10;
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
        
        function [qMatrix,vMatrix, tMatrix] = GenerateThrow(obj, velocityVector)
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

            % calc vars
            x = [];
            theta = [];
            trajectoryDeltaT = [];

            % For each segment of the trajectory
            for i = 1:size(obj.cartesianWaypoints, 3) - 1
                % get the interpolated segment
                [x_local,theta_local, t_segment] = obj.interpolateSegment(obj.cartesianWaypoints(:,:,i), obj.cartesianWaypoints(:,:,i+1), velocityMagnitude, obj.STEPS);

                % Add the segments to the total path
                x = [x; x_local];
                theta = [theta; theta_local];
                trajectoryDeltaT = [trajectoryDeltaT; t_segment];
            end

            % total path
            obj.cartesianTrajectory = [x, theta];
            [qMatrix, vMatrix, tMatrix] = obj.GenerateRMRCSegment(x, theta, trajectoryDeltaT, ones(1,6));
        end

        % TODO change to take in theta matrix x matrix delta time matrix and joint seed
        function [qMatrix, vMatrix, tMatrix] = GenerateRMRCSegment(obj, xyz, rpy, dt, initialguess)
            % Preallocate return arrays  
            qMatrix = zeros(size(xyz, 1), obj.robot.n);
            vMatrix = qMatrix;
            tMatrix = qMatrix(:, 1);
            
            % populate segment time array 
            for i = 2:size(tMatrix)
                tMatrix(i) = tMatrix(i-1) + dt(i);
            end

            % get transform of first point
            T =  [ rpy2r(rpy(1, :)), xyz(1, :)'; zeros(1,3), 1 ];

            % initial joint state
            qMatrix(1,:) = obj.robot.ikcon(T, initialguess);

            % create joint state traj
            for i = 1:size(qMatrix, 1) - 1
                % determine forward kinematics solution of current joint states
                currentT = obj.robot.fkine(qMatrix(i,:));

                % determine the position delta to the next traj point
                deltaX = xyz(i+1, :)' - currentT(1:3,4);

                % determine linear velocity for this timestep
                linearVelocity = deltaX / dt(i);

                % determine rotation change over time
                deltaR = ( rpy2r(rpy(i, :)) - currentT(1:3,1:3) ) / dt(i);

                % skew the matrix and determine the angular velocity
                S = deltaR * currentT(1:3,1:3)';
                angularVelocity = [S(3,2); S(1,3); S(2,1)];

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
                for j = 1:obj.robot.n % Loop through joints 1 to 6
                    if qMatrix(i,j) + dt(i)*vMatrix(i,j) < obj.robot.qlim(j,1) % If next joint angle is lower than joint limit...
                        vMatrix(i, j) = ( obj.robot.qlim(j,1) - qMatrix(i,j) )/ dt(1);
                        %vMatrix(i,j) = 0;  % Stop the motor
                    elseif qMatrix(i,j) + dt(i)*vMatrix(i,j) > obj.robot.qlim(j,2) % If next joint angle is greater than joint limit ...
                        vMatrix(i, j) = ( obj.robot.qlim(j,2) - qMatrix(i,j) )/ dt(1);
                        %vMatrix(i,j) = 0; % Stop the motor
                    end
                end

                % update next joint state in qMatrix
                qMatrix(i+1,:) = qMatrix(i,:) + dt(i)*vMatrix(i,:);
            end

        end

        function [q, v, t] = jog(obj, qs, d)
            % dir should be unit vector in direction of movement
            % start/end transformy
            tStart = obj.robot.fkine(qs)
            tEnd   = transl(d) * tStart

            % make xyz, rpy path
            [x, r, dt] = obj.interpolateSegment(tStart, tEnd, obj.JOG_VEL, obj.JOG_STEPS);
            [q, v, t] = obj.GenerateRMRCSegment(x, r, dt, qs);
        end

        function [q] = jointJog(obj, qs, dq)
            % qs -> current joint state
            % dq -> delta for each joint
            qf = qs + dq;

            % Since this is all joint-space, not going to rmrc
            % quintic polynomial interpolation
            q = jtraj(qs, qf, obj.JOG_STEPS);
        end

        function [xMatrix, thetaMatrix, tMatrix] = interpolateSegment(obj, segmentStart, segmentEnd, velocityMagnitude, steps)
            
            % segment delta x,y,z
            seg = segmentEnd(1:3,4) - segmentStart(1:3,4);
            
            % segment length
            segMagnitude = norm(seg);
            
            % segment unit vector
            segDirection = seg / segMagnitude;

            % time interval for each step
            dt = segMagnitude / velocityMagnitude / steps;

            % xmatrix -> x,y,z positions of segment
            % thetaMatrix -> rpy of segment
            xMatrix     = zeros(steps, 3);
            thetaMatrix = zeros(steps, 3);
            tMatrix     = repmat(dt, steps, 1);

            % get the rpy increment for each step
            startRPY = tr2rpy(segmentStart(1:3,1:3))';
            endRPY = tr2rpy(segmentEnd(1:3,1:3))';
            rpyIncrement = (endRPY - startRPY) / steps;

            % interpolate translation and rotation
            for i = 1:steps
                xMatrix(i, :)     = (segmentStart(1:3,4) + (i-1)*(velocityMagnitude*dt)*segDirection)';
                thetaMatrix(i, :) = (startRPY + (i-1)*rpyIncrement)';
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
