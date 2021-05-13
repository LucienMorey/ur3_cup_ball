classdef Projectile < handle
    
    % Projectile class
    % Defines a projectile which has methods for determining launch
    % parameters given desired target.
    
    properties
        m;          % kg
        d;          % diameter
        k;          % coefficient of drag, adjusted to include other constants
        cor;        % coefficient of resititution 
        g = 9.81;
    end
    
    methods
        
        % Projectile constructor
        function obj = Projectile(mass_, d_, cd_, cor_)
            obj.m    = mass_;
            obj.d    = d_;
            obj.k    = 0.5 * 1.2754 * cd_ * pi * d_^2/4 / mass_;
            obj.cor  = cor_;
        end
        

        % x1 -> x,y,z
        % x2 -> x,y,z
        % n  -> num bounce
        % v0 -> vel magnitude
        function [v] = calcLaunch(obj, x1, x2, n, v0)
            
            % Dist in x-y plane (horizontal, flat on table)
            d = sqrt( (x2(1)-x1(1))^2 + (x2(2)-x1(2))^2 );
                        
            % initial position
            % use x0 = 0, since we are just looking at the throw in-plane,
            % we can take the launch point as directly above the origin
            xi = [0, x1(3)];
            
            % height of goal
            h = x2(3);

            % solve for theta
            if n == 0
                
                % CASE 1: no bounces - this has very limited range so if the
                % cup is too far it won't work
                c = dx - xi(1);
                q1 = v0^2/ (obj.g*c);
                q2 = v0^2 * (v0^2 - 2*obj.g*h + 2*obj.g*xi(2)) / (obj.g * c)^2;
                theta = atan( q1 + sqrt(q2 - 1) );
            elseif n == 1
                
                % CASE 2: 1 bounce - uses optimisation solver with initial guess
                for i = 0:4
                    ig = pi/4 + i*pi/18;

                    % calculate throw angle
                    [theta, exitflag] = solveQuad(xi, v0, d, h, obj.cor, ig);

                    if exitflag < 1
                        continue;
                    end

                    % make sure the ball is going down when it passes through goal
                    tgoal = d / v0*cos(theta);
                    c2 = sqrt(v0^2*sin(theta)^2 + 2*obj.g*x1(3));
                    tbounce = (v0*sin(theta) + c2)/obj.g;
                    vbounce = obj.cor * c2;
                    tpeak = tbounce + v0*sin(theta) / obj.g;

                    % if going down, then we can stop
                    if tgoal > tpeak
                        break;
                    end

                    % if not, keep incremementing initial guess to get a different solution
                end
                

                
            end        
            
            % Convert in-plane velocity/angle to 3D vector
            % given theta, phi - we basically have spherical coordinates so
            % its just stardard spherical->cartesian conversion
            phi = atan((x2(2)-x1(2)) / (x2(1)-x1(1)));
            vx = v0*cos(theta)*cos(phi);
            vy = v0*cos(theta)*sin(phi);
            vz = v0*sin(theta);
            
            % 3D velocity output vec
            v = [vx, vy, vz];
        end
        
        % Simulates the projectile for n bounces
        % Returns [x, y, t] vectors
        function [xMat] = simulateP(obj, x0, v0, n)

            if n > 0
                e  = eye(3);
                e(3, 3) = -obj.cor;
            end

            [xMat, vMat, tMat] = deal([]);

            for i = 0:n
                [x, v, t] = obj.runProjectile(x0, v0);
                % add movements together
                xMat = [xMat; x];
                vMat = [vMat; v];

                if ~isempty(tMat)
                    t = t + tMat(end);
                end
                t = [tMat; t];

                % Calculate new initial velocity/position after bounce
                if n > 0
                    v0 = (e * v(end, :)')';
                    x0 = x(end, :);
                end
            end
        end
        
        % Performs discrete time simulation of projectile
        % input collum vectors
        function [xMat, vMat, tMat] = runProjectile(obj, x0, v0)
            % Time & timestep
            t = 0;
            dt = 0.0005;

            % x, v at current timestep, initially x0, v0
            x = x0';
            v = v0';

            % initially empty
            [xMat, vMat, tMat, a] = deal([]);

            % constant vec, for constant accerlleration terms
            c = [0; 0; -obj.g];

            % z coord is > 0. Has not hit the ground yet
            while x(3) >= 0

                % update variables
                a = -obj.k * eye(3) * sign(v).^2 + c;
                v = v + dt * a;
                x = x + dt * v;
                t = t + dt;

                % add to mat
                if x(3) >= 0
                    xMat = [xMat; x'];
                    vMat = [vMat; v'];
                    tMat = [tMat; t];
                end
            end
        end
    end
end

