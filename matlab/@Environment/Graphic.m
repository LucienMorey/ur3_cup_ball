classdef Graphic < handle
    % Graphic class defines an object represented by a 4x4 homogeneous
    % transform for pose and a .ply file model.
    
    properties
        % pose of the object
        pose;
        oldpose;
        
        % ply file model
        faces;
        verts;
        vcolour;
        
        % plot handle
        mesh_h;
        
        %data logging robot
        robot;
        initialPose;
        effectorPoseOnPick;
        effectorPoseOnPlace;
        destination;
        
    end
    
    methods
        
        % Create a graphics object
        % initialise appearance on plot
        function obj = Graphic(position, path)
            
            % object position
            % 4x4 transform
            obj.pose = position;
            obj.oldpose = zeros(4);
            
             % read ply file
            [f, v, data] = plyread(path,'tri');

            % add to model
            obj.faces = f;
            obj.verts = v;
            
            % vert colours
            obj.vcolour = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            
            % Shift mesh
            obj.shiftMesh();
            
            % add it to a plot
            obj.plot();
        end
        
        % Updates the graphic in the plot
        function plot(obj)
                % transform verticies based on current object pose
                tvert = [obj.pose * [obj.verts, ones(size(obj.verts, 1), 1)]']';
                
                % plot
                obj.mesh_h = trisurf(obj.faces, tvert(:, 1), tvert(:, 2), tvert(:, 3), ...
                            'FaceVertexCData', obj.vcolour, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
                
                % update last drawn pose
                obj.oldpose = obj.pose;
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
        
        % Shifts the mesh origin of the object for consistency
        % Sets x origin -> average x
        % Sets y origin -> average y
        % Sets z origin -> minimum z (bottom of object)
        function shiftMesh(obj)
            vc = size(obj.verts, 1);
            avgp = sum(obj.verts) / vc;
            avgp(3) = min(obj.verts(:, 3));
            
            obj.verts = obj.verts - repmat(avgp, vc, 1);
        end           
    end
end