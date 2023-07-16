function plot_3D_all(plot_data)

handle = plot_data{1};
ptCloud_room = plot_data{2};
patches_wall = plot_data{3};
rcd  = plot_data{4};
ground_plane = plot_data{5};
obj = plot_data{6};
numClusters = plot_data{7};
% Clear the current axes
cla(handle);
% Plot walls and ground together with the point cloud
%pcshow(ptCloud_room, "Parent", handle);
plot3(ptCloud_room.Location(:, 1), ptCloud_room.Location(:, 2), ptCloud_room.Location(:, 3), '.', 'MarkerSize', 10,'Parent', handle)
%hold on
%plot(handle, shp)
hold(handle,'on')
% Walls
rcd.plot_patches2(patches_wall,handle);
hold(handle,'on')
%Ground Plane
patch(handle, ground_plane(1,:),ground_plane(2,:),ground_plane(3,:), [0.5 0.5 0.5]);
%plot(plane_model_gc{1,1},'Parent', handle);

%Floor Plane
% Define the vertices and faces for the patch
 floor_z = min(patches_wall(3,:,:),[], "all");
% floor_x_max = max(patches_wall(1,:,:),[], "all");
% floor_x_min = min(patches_wall(1,:,:),[], "all");
% floor_y_max = max(patches_wall(2,:,:),[], "all");
% floor_y_min = min(patches_wall(2,:,:),[], "all");
% 
% vertices = [floor_x_min floor_y_min floor_z;  % Vertex 1
%             floor_x_max floor_y_min floor_z;  % Vertex 2
%             floor_x_max floor_y_max floor_z;  % Vertex 3
%             floor_x_min floor_y_max floor_z]; % Vertex 4
% faces = [1 2 3 4];  % Face connecting the vertices
% 
% % Plot the patch
% patch(handle, 'Faces', faces, 'Vertices', vertices, 'FaceColor', 'green', 'FaceAlpha', 0.9);

% Konstruktion der Objekte im Raum
%construct_Objects(floor_z, obj, numClusters, handle);

% hold(handle,'on')
% for i = 1:numClusters
%     hold(handle,'on')
%     plot(obj{i,1},'Parent', handle)
% end

end