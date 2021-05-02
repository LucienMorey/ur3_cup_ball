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
        minManipMeasure = 0.1;
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
        
        function [qMatrix,vMatrix] = GenerateTrajectory(obj, velocityVector)
            % determine velocity magnitude
            velocityMagnitude = sqrt(sum(velocityVector.^2));

            % project along velocity vector to find start and stop locations of throw
            velocityDirection = velocityVector./velocityMagnitude;
            p_start = obj.throwPosition + -1 * velocityDirection * obj.preThrowDistance;
            p_end = obj.throwPosition + velocityDirection * obj.postThrowDistance; 
            
            %concatinate all points in trajectory
            cartesian_trajectory = [obj.reloadLocation; p_start; obj.throwPosition; p_end; obj.reloadLocation]
            qMatrix = [obj.robot.ikcon(transl(obj.reloadLocation(1,1), obj.reloadLocation(1,2), obj.reloadLocation(1,3)))];
            vMatrix = [];

            for steps = 1:1:size(cartesian_trajectory,1)-1
                % calculate the time delta between current and next step
                timeDelta = obj.CalculateTimeDelta(cartesian_trajectory(steps,:), cartesian_trajectory(steps+1,:),velocityMagnitude);
                % calculate velocity vector 
                cartesian_vel = (cartesian_trajectory(steps+1,:) - cartesian_trajectory(steps,:))./timeDelta;
                cartesian_vel = [cartesian_vel,0,0,0];
                % determine the jacobian
                J = obj.robot.jacob0(qMatrix(1,:));
                % determine the measure of manipulibiility
                measureOfManipulibility = sqrt(det(J*J'));
                % solve for joint velocities
                if measureOfManipulibility < obj.minManipMeasure
                    qdot = inv(J'*J + 0.01*eye(6))*J'*cartesian_vel';
                else
                    qdot = inv(J) * cartesian_vel'; % Solvevelocitities via RMRC    
                end
                % back subsitute for the qMatrix
                vMatrix = [vMatrix; qdot'];
                qMatrix(steps+1,:) = qMatrix(steps,:) + timeDelta * qdot';
            end
            vMatrix = [vMatrix; zeros(1,6)];
            

        end

        
        function timeDelta = CalculateTimeDelta(obj, p1, p2, desiredVelocity)
            timeDelta = sqrt(sum((((p2-p1)./desiredVelocity).^2)));
        end
    end
end
