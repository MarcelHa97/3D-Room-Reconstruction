% clc;
% close all;
% 
% imageDir = 'C:\Users\Marcel.H\Documents\TUM\3.Semester\Computer Vision\CV_Project\delivery_area_dslr_undistorted\delivery_area\images\dslr_images_undistorted\parts';
% %imageDir = 'C:\Users\Marcel.H\Documents\TUM\3.Semester\Computer Vision\CV_Project\kicker_dslr_undistorted\kicker\images\dslr_images_undistorted\';
% 
% cameraParam = importdata('cameras.txt',' ',3);

%% ------------------------------------------------------------------------
% Generate 3D points
function plot_data = script_3D_reconstruction(handle, points)

%[points, ~] = Main_get3dpoints(imageDir, cameraParam);

% Remove outliers
points(any(points > 60, 2), :) = [];
points = double(points);
ptCloud = pointCloud(points);

% Rotate point cloud according to the determined angle 
deg = 280;% 280;
rotationAngles = [deg 0 0];
translation = [0 0 0];
tform1 = rigidtform3d(rotationAngles,translation);
rot_ptCloud = pctransform(ptCloud,tform1);
points2 = rot_ptCloud.Location(:,:);

% Create object
rcd = reconstructModel;

[ptCloud_room, patches_wall, ground_plane, plane_model_gc, denoised_ptCloud] = reconstruct_room(points2);

% Remove point that belong to the ground and ceiling plane 
ptCloud_room = rcd.remove_ceiling_points(ptCloud_room);

% Segments the point cloud into several segments
%[segmentedPtCloud, numClusters] = rcd.segment_dataPt(ptCloud_room, 0.2, 10); % 0.2 10

% First try from me to reconstruct the objects. I guess their are better
% methods. Maybe you have to do some filtering and checks if enough points
% are available. 
% Another option would be to use the whole point cloud of the room and to
% find suitable objects.



%obj = {numClusters,1};
obj = 0;

%for i = 1:numClusters
%   x = segmentedPtCloud{i,1}.Location(:,1);
%   y = segmentedPtCloud{i,1}.Location(:,2);
%   z = segmentedPtCloud{i,1}.Location(:,3);
%   shp = alphaShape(x,y,z,1);
%   obj{i,1} = shp;
%end

numClusters = 0;


plot_data = {handle, ptCloud_room, patches_wall, rcd, ground_plane, obj, numClusters};
end


function [ptCloud_room, patches_wall, ground_plane, plane_model_gc, denoised_ptCloud] = reconstruct_room(data)

    % Convert the data to a point cloud
    %data2 = pointCloud(data);
    
    % Remove outlyers of the point cloud
    %data = pcdenoise(dataIn, 'NumNeighbors', 40, 'Threshold', 0.1);
    
    % Create object to get access to all functions
    rcd = reconstructModel;
    bd = boundDetection;

    % Remove outliers of the point cloud
    [denoised_ptCloud, denoised_ptCloud1] = rcd.filter_ptCloud_init(data);
    
    % Detect walls into the point cloud, locally and globally 
    %[plane_model_v, plane_model_h, plane_model_gc, rot_ptCloud, denoised_ptCloud, seg_PtCloud, dist_v, dist_h, deg] = rcd.seg_in_plane(denoised_ptCloud, denoised_ptCloud1);
    %[rot_ptCloud, rot_ptCloud2, denoised_ptCloud] = rcd.rotated_ptCloud(denoised_ptCloud, denoised_ptCloud1);
    [plane_model_v, plane_model_h, plane_model_gc, ~, dist_v, dist_h] = rcd.seg_in_plane(denoised_ptCloud, denoised_ptCloud, 0.9, 30);% rot_ptCloud2);
    [g_plane_model_v, g_plane_model_h, g_dist_v, g_dist_h] = rcd.global_plane(denoised_ptCloud);
    
    % Select the planes that contain the most featrue points
    [~, ~, v_plane_model, dist1] = rcd.select_plane2(g_plane_model_v, plane_model_v, g_dist_v, dist_v, 0.9);
    [~, ~, h_plane_model, dist2] = rcd.select_plane2(g_plane_model_h, plane_model_h, g_dist_h, dist_h, 0.9);
    
    % Detect the outer planes of the point cloud
    [out_plane_v, out_plane_h, out_dist_v, out_dist_h] = rcd.outer_planes(denoised_ptCloud);

    
    [v_plane_model] = rcd.select_outer_plane(v_plane_model, out_plane_v, dist1, out_dist_v, 0.8);
    [h_plane_model] = rcd.select_outer_plane(h_plane_model, out_plane_h, dist2, out_dist_h, 0.8);
    
    % Calculate the intersection points between the planes
    % Store it in a polygon format
    [~, patch_points_h] = rcd.calculate_Intersection_Points(v_plane_model, h_plane_model, plane_model_gc);
    
    % Find the conture of the room
    %fi_ptCloud = pcdenoise(denoised_ptCloud, 'NumNeighbors', 5, 'Threshold', 0.5);
    cont = denoised_ptCloud.Location(:,1:2);
    x = cont(:,1);
    y = cont(:,2);
    
    % Detect the conture of the room
    [scale] = rcd.find_scaling_factor2(denoised_ptCloud);

    if scale > 1
        scale = 1;
    elseif scale < 0
        scale = 0;
    end
    
    k = boundary(x,y,scale); %0.7 %0.01
    x_new = x(k);
    y_new = y(k);
    
    % Optimize Conture
    [opti_data_x] = bd.optimize_data_pos(x_new, 0.2); % 0.15
    [opti_data_y] = bd.optimize_data_pos(y_new, 0.2); % 0.15
    
    [opti_data_x] = bd.optimize_data_pos(opti_data_x, 0.6);
    [opti_data_y] = bd.optimize_data_pos(opti_data_y, 0.6);
    
    [opti_data_x] = bd.optimize_data_pos(opti_data_x, 0.6);
    [opti_data_y] = bd.optimize_data_pos(opti_data_y, 0.6);
    
    % Set corner parameters
    [data_pt, ~, ~] = bd.set_corner_points(opti_data_x,opti_data_y, 1.1, 0.7);
    [data_pt] = bd.remove_data_points(data_pt);
    
    % Find nearest conture points in previous calculated grid 
    patch_points_h = patch_points_h * (-1);
    patch_points_h = patch_points_h';
    patch_points_h = patch_points_h(:,1:2);

    % Find corner values that are lying close to the generated boundary
    a = knnsearch(patch_points_h, data_pt);
    b = patch_points_h(a,:);
    
    %[patch_visu] = rcd.generate_patches(b, -plane_model_gc{1,1}.Parameters(4), -plane_model_gc{2,1}.Parameters(4));
    [patches_wall] = rcd.generate_patches(b, -plane_model_gc{1,1}.Parameters(4), -plane_model_gc{2,1}.Parameters(4));
    z = ones(1,length(b(:,1))) * -plane_model_gc{1,1}.Parameters(4);
    ground_plane = [b'; z];
    
    % Remove double patches
    % [patches_wall, ~] = rcd.remove_dup_patches(patches_wall);
    
    % Find points that are lying within the room
    [~, ~, ptCloud_room] = rcd.remove_outer_points(denoised_ptCloud, ground_plane, 0.5);

    delete(rcd);
    delete(bd);
end

