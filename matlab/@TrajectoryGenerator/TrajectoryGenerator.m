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
        cartesianWaypoints;
    end

    properties(Constant)
        EPSILON = 0.05; % tune this if not moving well
        VELOCITY_WEIGHTING = diag([1,1,1,1,1,0.1]);
        STEPS = 75;     % more steps make the movement better
        LINEAR_CART_VEL = 0.1;
        JOINT_JOG_VEL = pi/9;
        JOG_STEPS = 30; % more steps make the movement better
        maxJointVelocities = [pi, pi, pi, 2*pi, 2*pi, 2*pi];
        VEL_FINAL = 0.2;
        VEL_PRE_THROW = 0.1;
        VEL_POST_THROW = 0.8;
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
        
        function [qMatrix,vMatrix, tMatrix, xMatrix] = GenerateThrow(obj, velocityVector, qInitial)
            qMatrix = [];
            vMatrix = [];
            tMatrix = [];
            
            % velocity magnitude
            velocityMagnitude = norm(velocityVector);

            % get velocity unit vector
            velocityDirection = (velocityVector)' / velocityMagnitude;

            % get rotation matrix, z-axis in line with velocity vector
            obj.throwPosition(1:3,1:3) = obj.doubleCross(velocityDirection);
            
            % Get start and end of throw, by extending velocity direcion by pre-throw dist and post-throw dist
            % Rotation component is not touched to ensure end-effector stays straight
            pStart = obj.throwPosition + [zeros(3,3), -obj.preThrowDistance * velocityDirection; zeros(1,4)];
            pEnd   = obj.throwPosition + [zeros(3,3), obj.postThrowDistance * velocityDirection; zeros(1,4)]; 
            pInitial = obj.robot.fkine(qInitial);
            
            % concatinate all points in trajectory
            % five 4x4 transforms
            obj.cartesianWaypoints = zeros(4,4,5);
            obj.cartesianWaypoints(:,:,1) = pInitial;
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

                % find the velocity for this segment
                if i == 1
                    vel = velocityMagnitude * obj.VEL_PRE_THROW;
                elseif i == 3
                    vel = velocityMagnitude * obj.VEL_POST_THROW;
                elseif i == 4
                    vel = velocityMagnitude * obj.VEL_FINAL;
                else
                    vel = velocityMagnitude;
                end

                % get the interpolated segment
                [x_local,theta_local, t_segment] = obj.interpolateSegment(obj.cartesianWaypoints(:,:,i), obj.cartesianWaypoints(:,:,i+1), vel, obj.STEPS);

                % Add the segments to the total path
                x = [x; x_local];
                theta = [theta; theta_local];
                trajectoryDeltaT = [trajectoryDeltaT; t_segment];
            end

            % total path
            obj.cartesianTrajectory = [x, theta];
            xMatrix = x;
            [qMatrix, vMatrix, tMatrix] = obj.GenerateRMRCSegment(x, theta, trajectoryDeltaT, qInitial);
        end

        % TODO change to take in theta matrix x matrix delta time matrix and joint seed
        function [qMatrix, vMatrix, tMatrix] = GenerateRMRCSegment(obj, xyz, rpy, dt, qInitial)
            % Preallocate return arrays  
            qMatrix = zeros(size(xyz, 1), obj.robot.n);
            vMatrix = qMatrix;
            tMatrix = qMatrix(:, 1);
            
            % populate segment time array 
            for i = 2:size(tMatrix)
                tMatrix(i) = tMatrix(i-1) + dt(i);
            end

            % initial joint state
            qMatrix(1,:) = qInitial;

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
%                 marray = [marray; m];
                %m = 0;

                % check if a damped least squares solution is required
                if m < obj.EPSILON
                    % least squares
                    lambda = (1 - (m/obj.EPSILON)^2)*0.005;
                else
                    % not required
                    lambda = 0;
                end
                
                % apply least squares if required and invert jacobian
                invJ = inv(J'*J + lambda *eye(6))*J';

                % determine joint velocities
                vMatrix(i,:) = invJ*cartesianVelocity;
                
                %scale velocity if going to exceed max velocity
                [vMax, vMaxIndex] = max(abs(vMatrix(i,:)));
                if vMax > obj.maxJointVelocities(1,vMaxIndex)
                    vMatrix(i,:) = ((obj.maxJointVelocities(1,vMaxIndex))/vMax) .*vMatrix(i,:);
                end

                % check if expected to exceed joint limits
                for j = 1:obj.robot.n % Loop through joints 1 to 6
                    if qMatrix(i,j) + dt(i)*vMatrix(i,j) < obj.robot.qlim(j,1) % If next joint angle is lower than joint limit...
                        vMatrix(i, j) = 0;
                        %vMatrix(i,j) = 0;  % Stop the motor
                    elseif qMatrix(i,j) + dt(i)*vMatrix(i,j) > obj.robot.qlim(j,2) % If next joint angle is greater than joint limit ...
                        vMatrix(i, j) = 0;
                        %vMatrix(i,j) = 0; % Stop the motor
                    end
                end

                % update next joint state in qMatrix
                qMatrix(i+1,:) = qMatrix(i,:) + dt(i)*vMatrix(i,:);
            end

        end

        function [q, v, t] = cjog(obj, qs, d)
            % dir should be unit vector in direction of movement
            % start/end transformy
            tStart = obj.robot.fkine(qs);
            tEnd   = transl(d) * tStart;

            % make xyz, rpy path
            [x, r, dt] = obj.interpolateSegment(tStart, tEnd, obj.LINEAR_CART_VEL, obj.JOG_STEPS);
            [q, v, t] = obj.GenerateRMRCSegment(x, r, dt, qs);
        end

        function [q, v, t] = jjog(obj, qs, dq)
            % qs -> current joint state
            % dq -> delta for each joint
            qf = qs + dq;

            % Since this is all joint-space, not going to rmrc
            % quintic polynomial interpolation
            q = jtraj(qs, qf, obj.JOG_STEPS);

            % time increment, based on largest joint error
            dt = max(abs(dq)) / obj.JOINT_JOG_VEL / obj.JOG_STEPS;
            t = zeros(obj.JOG_STEPS, 1);
            for i = 1:obj.JOG_STEPS
                t(i) = i*dt;
            end

            % calc velocity based on constant time increment
            v = zeros(obj.JOG_STEPS, 6);
            
            for i = 1:obj.JOG_STEPS - 1
                v(i, :) = (q(i+1, :) - q(i, :)) ./ dt;
            end
        end

        function [xMatrix, thetaMatrix, tMatrix] = interpolateSegment(obj, segmentStart, segmentEnd, velocityMagnitude, steps)
            
            % get the rpy increment for each step
            startRPY = tr2rpy(segmentStart(1:3,1:3))';
            dr = (tr2rpy(segmentEnd(1:3,1:3))' - startRPY) / steps;

            % get the xyz increment for each steps
            startXYZ = segmentStart(1:3,4);
            dx = (segmentEnd(1:3,4) - startXYZ) / steps;

            % segment delta x,y,z
            seg = segmentEnd(1:3,4) - segmentStart(1:3,4);

            % time interval for each step
            dt = norm(seg) / velocityMagnitude / steps;

            % xmatrix -> x,y,z positions of segment
            % thetaMatrix -> rpy of segment
            xMatrix     = zeros(steps, 3);
            thetaMatrix = zeros(steps, 3);
            tMatrix     = repmat(dt, steps, 1);

            % interpolate translation and rotation
            for i = 1:steps
                xMatrix(i, :)     = (startXYZ + i*dx)';
                thetaMatrix(i, :) = (startRPY + i*dr)';
            end
        end

        function rMatrix = doubleCross(obj, zLocal)
            % arbitrary y is determined by taking the cross product of local z and the global z;
            yLocal = cross(zLocal, [0; 0; 1]);
            
            % subsequent x vector can be found with the cross from y to z local
            xLocal = cross(yLocal, zLocal);
            
            % determine rpy from rotation matrix
            rMatrix = [xLocal,  yLocal, zLocal];
        end
    end
end
