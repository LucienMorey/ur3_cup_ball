classdef TrajectoryGenerator < handle
    %TRAJECTORYGENERATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot;
        throwPosition;
        preThrowDistance;
        postThrowDistance;
        reloadLocation
        traj;
        orientation;
    end

    properties(Constant)
        EPSILON = 0.1;
        VELOCITY_WEIGHTING = diag([1,1,1,0.1,0.1,0.1]);
        STEPS = 20;
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
            
            % determine velocity magnitude
            velocityMagnitude = sqrt(sum(sum(velocityVector.^2)));

            % project along velocity vector to find start and stop locations of throw
            velocityDirection = velocityVector./velocityMagnitude;
            pStart = obj.throwPosition + [zeros(3,3), (-1 * obj.preThrowDistance) .* velocityDirection; zeros(1,4)];
            pEnd = obj.throwPosition + [zeros(3,3), obj.postThrowDistance .* velocityDirection; zeros(1,4)]; 
            
            %concatinate all points in trajectory
            cartesianTrajectory = zeros(4,4,5);
            cartesianTrajectory(:,:,1) = obj.reloadLocation;
            cartesianTrajectory(:,:,2) = pStart;
            cartesianTrajectory(:,:,3) = obj.throwPosition;
            cartesianTrajectory(:,:,4) = pEnd;
            cartesianTrajectory(:,:,5) = obj.reloadLocation;

            % interpolate RMRC segment for each cartesian trajectory segment
            for i=1:1:size(cartesianTrajectory,3) - 1
                % get matrices from interpolator
                if size(tMatrix,1) == 0
                    [segmentQMatrix, segmentVMatrix, segmentTMatrix] = obj.GenerateRMRCSegment(cartesianTrajectory(:,:,i), cartesianTrajectory(:,:,i+1), velocityVector, 0, zeros(1,6));
                else
                    [segmentQMatrix, segmentVMatrix, segmentTMatrix] = obj.GenerateRMRCSegment(cartesianTrajectory(:,:,i), cartesianTrajectory(:,:,i+1), velocityVector, tMatrix(end),qMatrix(end,:));
                end

                % concatinate matrices
                qMatrix = [qMatrix; segmentQMatrix];
                vMatrix = [vMatrix; segmentVMatrix];
                tMatrix = [tMatrix, segmentTMatrix];
            end

        end

        function [qMatrix, vMatrix, tMatrix] = GenerateRMRCSegment(obj, startPoint, endPoint, desiredVelocity, segmentStartTime, jointSeed)
            % Preallocate return arrays  
            qMatrix = zeros(obj.STEPS, obj.robot.n);
            vMatrix = zeros(obj.STEPS, obj.robot.n);
            tMatrix = zeros(1, obj.STEPS);
            
            %determine time delta for segments
            segDist = sqrt(sum(sum((endPoint(1:3,4) - startPoint(1:3,4)).^2)));
            velocityMagnitude = sqrt(sum(sum(desiredVelocity.^2)));
            segTime = segDist/velocityMagnitude;
            deltaT = segTime/obj.STEPS;
            
            % populate segment time array 
            tMatrix(1) = segmentStartTime;
            for i=2:1:obj.STEPS
                tMatrix(1, i) = tMatrix(1, i-1) + deltaT;
            end

            % break up traj segment into cartesian point array of size steps
            [x,theta] = obj.interpolateSegment(startPoint,endPoint,velocityMagnitude,deltaT);
            obj.traj = [obj.traj, x];
            
            % get transform of first point
            T =  [rpy2r(theta(1,1), theta(2,1), theta(3,1)) x(:,1);zeros(1,3) 1];
            obj.orientation = cat(3, obj.orientation, T);

            % initial joint state
            qMatrix(1,:) = obj.robot.ikcon(T,jointSeed);

            % create joint state traj
            for i=1:1:obj.STEPS-1
                % determine forward kinematics solution of current joint states
                currentT = obj.robot.fkine(qMatrix(i,:));

                % determine the position delta to the next traj point
                deltaX = x(:,i+1) - T(1:3,4);

                % determine linear velocity for this timestep
                linearVelocity = (1/deltaT)*deltaX;

                % determine current rotation matrix
                currentR = currentT(1:3,1:3);

                % determine future rotation matrix
                nextR = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));

                % determine rotation change over time
                deltaR = (1/deltaT)*(nextR - currentR);

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
                    if qMatrix(i,j) + deltaT*vMatrix(i,j) < obj.robot.qlim(j,1) % If next joint angle is lower than joint limit...
                        vMatrix(i,j) = 0;  % Stop the motor
                    elseif qMatrix(i,j) + deltaT*vMatrix(i,j) > obj.robot.qlim(j,2) % If next joint angle is greater than joint limit ...
                        vMatrix(i,j) = 0; % Stop the motor
                    end
                end

                % update next joint state in qMatrix
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*vMatrix(i,:);
            end

        end

        function [xMatrix, thetaMatrix] = interpolateSegment(obj, segmentStart, segmentEnd, velocityMagnitude, deltaT)
            seg = segmentEnd(1:3,4) - segmentStart(1:3,4);
            % determine segment
            segMagnitude = sqrt(sum(sum(seg.^2)));
            
            % determine directional unit vector of segment
            segDirection = seg./segMagnitude;

            % preallocate cartesian point sizes
            xMatrix = zeros(3,obj.STEPS);      

            % grab the translation components from the tf
            xMatrix(:,1) = segmentStart(1:3,4);

            %interpolate translation
            for i=2:1:obj.STEPS
                xMatrix(:,i) = xMatrix(:,i-1) + (velocityMagnitude*deltaT)*segDirection;
            end

            % determine orientation of velocity vector wiht double cross
            %z direction is along the line of travel
            z_local = segDirection;
            
            % arbitrary y is determined by taking the cross product of local z and the global z;
            y_local = cross(z_local, [0; 0; 1]);
            
            % subsequent x vector can be found with the cross from y to z local
            x_local = cross(y_local, z_local);
            
            % determine rpy from rotation matrix
            rpy = tr2rpy([x_local, y_local, z_local]);

            % assume constant orientation
            thetaMatrix = rpy' .* ones(3,obj.STEPS);
        end
    end
end
