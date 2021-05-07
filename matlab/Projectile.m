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
        
        function [vi, xi, dx, h, theta] = calcLaunch(obj, s, e, n, v0_)
            % s -> launch position [x, y, z]
            % e -> end position [x, y, z]
            % n -> number of bounces
            
            % First, calculate the throw parameter: dx
            dx = sqrt( (e(1)-s(1))^2 + (e(2)-s(2))^2 );
                        
            % initial position
            % use x0 = 0, since we are just looking at the throw in-plane,
            % we can take the launch point as directly above the origin
            xi = [0, s(3)];

            % initial velocity magnitude
            % max value is 1, this must be know to get the launch angle
            v0 = v0_;
            
            % height of goal
            h = e(3);

            % solve for theta
            if n == 0
                
                % CASE 1: no bounces - this has very limited range so if the
                % cup is too far it won't work
                c = dx - xi(1);
                q1 = v0^2/ (obj.g*c);
                q2 = v0^2 * (v0^2 - 2*obj.g*h + 2*obj.g*xi(2)) / (obj.g * c)^2;
                theta = atan( q1 + sqrt(q2 - 1) );
            elseif n == 1
                
                % CASE 2: 1 bounce - uses optimisation solver with initial
                % guess 45 deg
                theta = solveQuad(xi, v0, dx, h, obj.cor);
            end        
            
            % Convert in-plane velocity/angle to 3D vector
            % given theta, phi - we basically have spherical coordinates so
            % its just stardard spherical->cartesian conversion
            phi = atan( (e(2)-s(2))/(e(1)-s(2)) );
            vx = v0*cos(theta)*cos(phi);
            vy = v0*cos(theta)*sin(phi);
            vz = v0*sin(theta);
            
            % 3D velocity output vec
            vi = [vx; vy; vz];
        end
        
        % Simulates the projectile for n bounces
        % Returns [x, y, t] vectors
        function [x, y, t] = simulatep(obj, xi, vi, n)
            % Initial projectile motion
            [x, y, vx, vy, t] = obj.runProjectile(xi, vi);

            for i = 1:n
                % Calculate new initial velocity/position after bounce
                % This is used for the second stage of motion
                vi = [vx(end), -obj.cor * vy(end)];
                xi = [x(end), y(end)];

                % second bounce
                [x2, y2, vx2, vy2, t2] = obj.runProjectile(xi, vi);

                % add movements together
                x = [x; x2];
                y = [y; y2];
                vx = [vx; vx2];
                vy = [vy; vy2];
                t2 = t2 + t(end);
                t = [t; t2];
            end
        end
        
        % Performs simple projectile motion simulation
        % this is an internal function, used by simulatep()
        function [x_, y_, vx_, vy_, t_] = runProjectile(obj, x0, v0)
            % Time & timestep
            t = 0;
            dt = 0.0005;

            x = x0(1);
            y = x0(2);
            vx = v0(1);
            vy = v0(2);

            x_ = [];
            y_ = [];
            t_ = [];
            vx_ = [];
            vy_ = [];

            while y >= 0
                x_ = [x_; x];
                y_ = [y_; y];
                vx_ = [vx_; vx];
                vy_ = [vy_; vy];
                t_ = [t_; t];

                ay = -obj.g - obj.k * vy^2 * sign(vy);
                ax = -obj.k * vx^2 * sign(vx);

                vx = vx + ax*dt;
                vy = vy + ay*dt;

                x = x + vx*dt;
                y = y + vy*dt;

                t = t + dt;
            end
        end
    end
end

