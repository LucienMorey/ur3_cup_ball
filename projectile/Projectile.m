classdef Projectile < handle
    
    % Projectile class
    % Defines a projectile which has methods for determining launch
    % parameters given desired target.
    
    properties
        m;   % kg
        d;      % diameter
        k;      % coefficient of drag, adjusted to include other constants
        cor;    % coefficient of resititution 
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
        
        function vi = calcLaunch(obj, s, e, a)
            % s -> start position [x, y, z]
            % e -> end position [x, y, z]
            % a -> inclination angle at end [degrees]
            % n -> number of bounces
            
            % First get total time as a
        end
        
        % Simulates the projectile for n bounces
        % Returns [x, y, t] vectors
        function [x, y, t] = simulatep(obj, xi, vi, n)
            % Initial projectile motion
            [x, y, vx, vy, t] = obj.runProjectile(xi, vi);

            for i = 1:n
                % Calculate new initial velocity/position after bounce
                % This is used for the second stage of motion
                vi = [obj.cor * vx(end), -obj.cor * vy(end)];
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
        function [x_, y_, vx_, vy_, t] = runProjectile(obj, x0, v0)
            t = 0;
            dt = 0.001;

            x = x0(1);
            y = x0(2);
            vx = v0(1);
            vy = v0(2);

            x_ = [];
            y_ = [];
            vx_ = [];
            vy_ = [];

            while y >= 0
                x_ = [x_; x];
                y_ = [y_; y];
                vx_ = [vx_; vx];
                vy_ = [vy_; vy];

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

