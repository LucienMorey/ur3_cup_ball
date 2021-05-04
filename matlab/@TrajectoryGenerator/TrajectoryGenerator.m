classdef TrajectoryGenerator < handle
    %TRAJECTORYGENERATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot;
        throwPosition;
        preThrowDistance;
        postThrowDistance;
        reloadLocation
    end

    properties(Constant)
        EPSILON = 0.1;
        VELOCITY_WEIGHTING = diag([1,1,1,0.1,0.1,0.1]);
        STEPS = 200;
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
            % determine velocity magnitude
            velocityMagnitude = sqrt(sum(velocityVector.^2));

            % project along velocity vector to find start and stop locations of throw
            velocityDirection = velocityVector./velocityMagnitude;
            p_start = obj.throwPosition + -1 * velocityDirection * obj.preThrowDistance;
            p_end = obj.throwPosition + velocityDirection * obj.postThrowDistance; 
            
            %concatinate all points in trajectory
            %TODO ensure all points are in tf matrix form
            cartesian_trajectory = [obj.reloadLocation; p_start; obj.throwPosition; p_end; obj.reloadLocation]

            % interpolate RMRC segment for each cartesian trajectory segment
            for i=1:1:size(cartesian_trajectory,3) - 1
                % get matrices from interpolator

                % concatinate matrices
            end

        end

        function [qMatrix, vMatrix, tMatrix] = GenerateRMRCSegment(obj, startPoint, endPoint, startTime, desiredVelocity, segmentStartTime)
            % Preallocate return arrays  
            qMatrix = zeros(obj.STEPS, obj.robot.n);
            vMatrix = zeros(obj.STEPS, obj.robot.n);
            tMatrix = zeros(obj.STEPS);
            
            %determine time delta for segments
            segDist = sqrt(sum(sum((endPoint(1:3,4) - startPoint(1:3,4)).^2)));
            segTime = segDist/desiredVelocity;
            deltaT = segTime/obj.STEPS;
            
            % populate segment time array 
            tMatrix(1) = segmentStartTime;
            for i=2:1:obj.STEPS
                tMatrix(i) = tMatrix(i-1) + deltaT;
            end

            % break up traj segment into cartesian point array of size steps
            [x,theta] = obj.interpolateSegment(startPoint,endPoint,desiredVelocity,deltaT);

            % get transform of first point
            T =  [rpy2r(theta(1,1), theta(1,2), theta(1,2)) x(:,1);zeros(1,3) 1];
            
            % estimation for initial joint state
            % TODO find method for better starting estimate
            q0 = zeros(1,6);

            % initial joint state
            qMatrix(1,:) = robot.ikcon(T,q0);

            % create joint state traj
            for i=1:1:obj.STEPS
                % determine forward kinematics solution of current joint states

                % determine the position delta to the next traj point

                % determine linear velocity for this timestep

                % determine current rotation matrix

                % determine future rotation matrix

                % determine rotation error

                % skew the matrix and determine the angular velocity

                % concatinate velocity vectors and scale

                % determine the current jacobian

                % determine the current measure of manipulibility

                % check if a damped least squares solution is required
                %if m < obj.EPSILON
                    %least squares
                %else
                    %not required
                %end
                
                % apply least squares if required and invert jacobian

                % determine joint velocities

                % check if expected to exceed joint limits

                % update next joint state in qMatrix
                % update next velocity state in vMatrix
                % update next times step in tMatrix 


            end

        end

        function [xMatrix, thetaMatrix] = interpolateSegment(obj, segmentStart, segmentEnd, velocityVector, deltaT)
            % determine velocity magnitude
            velocityMagnitude = sqrt(sum(sum(velocityVector.^2)));

            % project along velocity vector to find start and stop locations of throw
            velocityDirection = velocityVector./velocityMagnitude;

            % preallocate cartesian point sizes
            xMatrix = zeros(3,obj.STEPS);      

            % grab the translation components from the tf
            xMatrix(:,1) = segmentStart(1:3,4);

            %interpolate translation
            for i=2:1:obj.STEPS
                xMatrix(:,i) = x(:,i-1) + (velocityMagnitude*deltaT)*velocityDirection;
            end

            % determine orientation of velocity vector
            rpy = tr2rpy(segmentStart)

            % assume constant orientation
            thetaMatrix = rpy .* ones(3,obj.STEPS);
        end
    end
end
