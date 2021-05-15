classdef Environment < handle
    %loads ply files into the gui for the robot
    
    properties
        %pose of the object
        pose;
        old_pose
        
        %ply stuff
        faces;
        verts;
        vert_colours;
        
        %plot handle
        mesh_h;
    end
    
    methods
        
        function obj = Environment(position, path)
            obj.pose = position;
            obj.old_pose = zeros(4)
            %create objects
            %read ply data
            [f, v, data] = plyread(path);
            obj.faces = f;
            obj.verts = v;
            
            % vert colours
            obj.vert_colours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            
            % Shift mesh
            obj.shiftMesh();
            
            % add it to a plot
            obj.plot();
        end
        
        function plot(obj)
            % transform verticies based on current object pose
            tvert = [obj.pose * [obj.verts, ones(size(obj.verts, 1), 1)]']';

            % plot
            obj.mesh_h = trisurf(obj.faces, tvert(:, 1), tvert(:, 2), tvert(:, 3), ...
                        'FaceVertexCData', obj.vert_colours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');

            % update last drawn pose
            obj.old_pose = obj.pose;
        end
        
                        % Updates the graphic in the plot
        function animate(obj)
            % Only plot if the pose is different from the last drawn pose.
            % Otherwise leave it.
            if ~isequal(obj.pose, obj.oldpose)
                
                % transform verticies based on current object pose
                tvert = [obj.pose * [obj.verts, ones(size(obj.verts, 1), 1)]']';
                
                % plot
                obj.mesh_h.Vertices = tvert(:, 1:3);
                drawnow();
                
                % update last drawn pose
                obj.oldpose = obj.pose;
            end
        end
        function shiftMesh(obj)
            vc = size(obj.verts, 1);
            avgp = sum(obj.verts) / vc;
            avgp(3) = min(obj.verts(:, 3));
            
            obj.verts = obj.verts - repmat(avgp, vc, 1);
        end           
    end
end
        