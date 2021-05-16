% Check the collision of each joint of the robot with the specified plane
% Interates through all joint states in the trajectory
% Collision plane is specified in point-normal format
function col = checkCollision(robot, qList, ppoint, pnormal)
    col = 0;
        
    % for each joint-state in traj
    for i = 1:size(qList, 1)

        % for each link
        for k = 1:robot.model.n

            % get the endpoints of the link
            [p1, p2] = getLinkn(robot, k, qList(i, :));

            % check if that link collides with plane
            col = linePlaneIntersection(p1, p2, ppoint, pnormal);

            % if there is collision, stop
            if col
                % plot this collision so we can see where
                plotCollision(robot, qList(i, :), pnormal, ppoint);
                return;
            end
        end
    end
end

% Gets the start/end pointsn of the robots nth link when at joint state q
function [p1, p2] = getLinkn(robot, n, q)

    % base transform
    tfn = robot.model.base;

    % if we want the first link, the startpoint is the base
    if n == 1
        p1 = tfn(1:3, 4)';
    end

    % Do fkine for each link individually
    for i = 1:n
        tfn = tfn * trotz(q(i));
        tfn = tfn * transl(robot.model.a(i), 0, robot.model.d(i));
        tfn = tfn * trotx(robot.model.alpha(i));

        % Set startpoint of the link
        if i == n-1
            p1 = tfn(1:3, 4)';
        end
    end
    
    % Set endpoint
    p2 = tfn(1:3, 4)';
end

% Checks collision of line segment and plane
function ret = linePlaneIntersection(p1, p2, pp, pn)
    % check if line is parallel to plane
    u = p2 - p1;
    w = p1 - pp;
    d = dot(pn, u);

    % check line parallel
    if abs(d) < 0.001
        % parallel-coincident
        if dot(pn, w) == 0
            ret = 1;

        % parallel-disjoint
        else
            ret = 0;
        end

    % single intersection
    else
        N = dot(-pn, w);
        q = N / d;

        % intersects within line segment
        if q > 0 && q < 1
            ret = 1;

        % no intersect within line segment
        else
            ret = 0;
        end
    end
end

% Plots the robot links and collision plane in the case of a collision
function plotCollision(robot, q, n, p)

    figure()

    % plot plane
    w = null(n);                    % Find two orthonormal vectors which are orthogonal to v
    ms = linspace(-0.2, 0.2, 4);
    [P,Q] = meshgrid(ms);           % Provide a gridwork (you choose the size)
    X = p(1)+w(1,1)*P+w(1,2)*Q;     % Compute the corresponding cartesian coordinates
    Y = p(2)+w(2,1)*P+w(2,2)*Q;     % Using the two vectors in w
    Z = p(3)+w(3,1)*P+w(3,2)*Q;
    surf(X,Y,Z)

    hold on;

    % plot links
    for i = 1:robot.model.n
        [p1, p2] = getLinkn(robot, i, q);

        plot3(p1(1), p1(2), p1(3), 'ro');
        plot3(p2(1), p2(2), p2(3), 'ro');

        p = [p1; p2];
        plot3(p(:, 1), p(:, 2), p(:, 3));
    end

    % scale axis equally
    daspect([1 1 1])
end

