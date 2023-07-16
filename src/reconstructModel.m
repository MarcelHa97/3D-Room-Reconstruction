classdef reconstructModel
   methods
        function delete(obj)
            % obj is always scalar
        end
        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function Name :   Point cloud filtering
        %
        % Description   :   The function receives data points and converts
        %                   the data points into a point cloud and then
        %                   removes the outliers of the data point
        % 
        % Inputs        :   array of data points (x,y,z)
        %
        % Outputs       :   Two filtered point clouds
        % =========================================================================
        function [denoised_ptCloud, denoised_ptCloud1] = filter_ptCloud_init(obj, data_points)
             %Convert incoming data points to a point cloud for processing
             ptCloud = pointCloud(data_points);
        
             % Remove outlyers from the point cloud
             % First filtering step - Define threshold and number of
             % neighbors
             denoised_ptCloud = pcdenoise(ptCloud, 'NumNeighbors', 40, 'Threshold', 0.1);
             denoised_ptCloud1 = denoised_ptCloud;
        
             %Apply further filtering to reduce number of points
             %ptCloud_fi = pcdenoise(denoised_ptCloud1, 'NumNeighbors', 5, 'Threshold', 0.1);
             %denoised_ptCloud = pcdenoise(denoised_ptCloud1, 'NumNeighbors', 5, 'Threshold', 0.9);
             %denoised_ptCloud = pcdenoise(ptCloud_fi, 'NumNeighbors', 5, 'Threshold', 0.05); %10
        end

        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function Name :   Rotated Point Cloud
        %
        % Description   :   The function checks the angular between the
        %                   x-axis and the points lying in a plane. The
        %                   caluclated angular is used to rotated the whole
        %                   point cloud that walls are lying in parallel to
        %                   the x-axis and y-axis
        % 
        % Inputs        :   1. Filtered point cloud
        %                   2. Filtered point cloud
        %
        % Outputs       :   1. Rotated point cloud
        %                   2. Filtered rotated point cloud
        %                   3. Filtered rotated point cloud
        % =========================================================================
        function [rot_ptCloud, rot_ptCloud2, denoised_ptCloud] = rotated_ptCloud(obj, ptCloud_fi, ptCloud)
            %Create clusters from the filter point cloud
            minDistance = 0.5;
            minPoints = 100;
            [labels, ~] = pcsegdist(ptCloud_fi, minDistance, 'NumClusterPoints',minPoints);
            idxValidPoints = find(labels == 1);
            segmentedPtCloud = select(ptCloud_fi,idxValidPoints);
        
            %Rotate the point cloud that it is parallel to the vertical and
            %horizontal axis
            %Assume that the data includes a wall which can be fitted by a plane
            %model
            maxDistance = 0.1; % Old 0.1
            maxAngularDistance = 5;
        
            [model] = pcfitplane(segmentedPtCloud, maxDistance,  'maxAngularDistance' , maxAngularDistance);
            norm_vector = model.Normal;
            % Determine angle between normal vector of plane and x-axis
            x1 = [1;0];
            x2 = [norm_vector(1); norm_vector(2)];
            rad = subspace(x1,x2);
            deg = rad2deg(rad);
        
            % Rotate point cloud according to the determined angle 
            rotationAngles = [0 0 deg];
            translation = [0 0 0];
            tform1 = rigidtform3d(rotationAngles,translation);
            rot_ptCloud = pctransform(ptCloud,tform1);
        
            % Filter rotated point cloud
            rot_ptCloud2 = pcdenoise(rot_ptCloud, 'NumNeighbors', 5, 'Threshold', 0.1);
            denoised_ptCloud = rot_ptCloud2;
            rot_ptCloud2 = pcdenoise(rot_ptCloud2, 'NumNeighbors', 10, 'Threshold', 0.05);
        end

        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function Name :   Detect ground and ceiling planes
        %
        % Description   :   The function searches for the ground and
        %                   ceiling and returns to planes models together with the 
        %                   belonging points
        % 
        % Inputs        :   1. Point cloud
        %
        % Outputs       :   1. Ground and ceiling plane model (cell array)
        % =========================================================================
        function [plane_model_gc] = ground_plane_det(obj, ptCloud)
            % Find plane models in z-direction (ground and ceiling)
            maxDistance = 4; % 0.5
            maxAngularDistance = 30;
            referenceVector = [0,0,1];
            ptCloud_z = ptCloud;
            
            % Define ROI to find a plane in the upper region of the point cloud for
            % the ceiling and in the lower region to find a ground plane
            % Usage of the filter point cloud to find a suitable plane
        
            % Search for a ground plane
            % Specify new z limits to only search in the lower region of the point
            % cloud
            model_arr3 = {};
            a = obj.find_scaling_factor(ptCloud_z);
            i = 1;
            while(i < 10)
                zLimit = [ptCloud_z.ZLimits(1) ptCloud_z.ZLimits(1)+a]; % Redefine this parameter!!!!!! 0.2  obj.find_scaling_factor(ptCloud_z)
                roi = [ptCloud_z.XLimits(1) ptCloud_z.XLimits(2) ptCloud_z.YLimits(1) ptCloud_z.YLimits(2) zLimit(1) zLimit(2)];
                indices = findPointsInROI(ptCloud_z,roi);
                ptCloud_Ground = select(ptCloud_z,indices);
                model_arr3{1,2} = ptCloud_Ground;
            
                [model, ~, ~] = pcfitplane(ptCloud_Ground, maxDistance, 'referenceVector', referenceVector, 'maxAngularDistance' , maxAngularDistance);
                if model.Parameters == [0 0 0 0]
                    a = a + 1;
                    continue;
                end
                tform = normalRotation(model,referenceVector);
                planeParams = model.Parameters * tform.T;
                transformedPlane = planeModel(planeParams);
                model_arr3{1,1} = transformedPlane;
                i = i + 1;
                break;
            end
            
            % Search for a ceiling in the upper region of the point cloud
            a = obj.find_scaling_factor(ptCloud_z);
            i = 1;
            while(i < 10)
                zLimit = [ptCloud_z.ZLimits(2)-a ptCloud_z.ZLimits(2)]; % Redefine this parameter!!!!!! obj.find_scaling_factor(ptCloud_z)
                roi = [ptCloud_z.XLimits(1) ptCloud_z.XLimits(2) ptCloud_z.YLimits(1) ptCloud_z.YLimits(2) zLimit(1) zLimit(2)];
                indices = findPointsInROI(ptCloud_z,roi);
                ptCloud_Ground = select(ptCloud_z,indices);
                model_arr3{2,2} = ptCloud_Ground;
            
                [model, ~, ~] = pcfitplane(ptCloud_Ground, maxDistance, 'referenceVector', referenceVector, 'maxAngularDistance' , maxAngularDistance);
                if model.Parameters == [0 0 0 0]
                    a = a + 0.5;
                    continue;
                end
                tform = normalRotation(model,referenceVector);
                planeParams = model.Parameters * tform.T;
                transformedPlane = planeModel(planeParams);
                diff = abs(model_arr3{1,1}.Parameters(4) - transformedPlane.Parameters(4));
                if diff > 5
                    model_arr3{2,1} = transformedPlane;
                else
                    mod_Par = transformedPlane.Parameters;
                    mod_Par(4) = -ptCloud_z.ZLimits(2);
                    model_arr3{2,1} = planeModel(mod_Par);
                end
                i = i + 1;
                break;
            end
            
            plane_model_gc = model_arr3;
        end

        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function Name :   Detect planes of single segements
        %
        % Description   :   Segment point cloud into multiple clusters.
        %                   Afterwards the segments are used to find plane
        %                   models in x and y-direction. The plane models
        %                   will be stored together with the segment of the
        %                   point cloud.
        % 
        % Inputs        :   1. Point cloud
        %
        % Outputs       :   1. Ground and ceiling plane model (cell array)
        % =========================================================================
        function [plane_model_v, plane_model_h, plane_model_gc, seg_PtCloud, dist_v, dist_h] = seg_in_plane(obj, denoised_ptCloud, rot_ptCloud2, minDistance, minPoints)
            % Remove Ceiling and Ground plane from point cloud
            plane_model_gc = obj.ground_plane_det(rot_ptCloud2);
            denoised_ptCloud = obj.remove_ground_points(denoised_ptCloud, plane_model_gc);
        
            % Cluster point before further porcessing 
            %minDistance = 0.9; % 0.5
            %minPoints = 30; %10
            [labels, numClusters] = pcsegdist(denoised_ptCloud, minDistance, 'NumClusterPoints',minPoints);
            idxValidPoints = find(labels);
            seg_PtCloud{1}{2} = labels(idxValidPoints);
            seg_PtCloud{1}{1} = select(denoised_ptCloud,idxValidPoints);
        
            % Find plane models in x-direction
            referenceVector = [1,0,0];
            ptCloud_x = denoised_ptCloud;
            model_arr = {};
            dist_v = [];
        
            % Define maximum angular and maximum distance of the points to
            % the fitted plane
            maxDistance = 0.5; %0.2 
            maxAngularDistance = 20; % 5
        
            for i = 1:numClusters
                idxValidPoints = find(labels == i);
                seg_ptCloud = select(ptCloud_x, idxValidPoints);
                [model, inlierIndices, ~] = pcfitplane(seg_ptCloud, maxDistance, 'referenceVector', referenceVector, 'maxAngularDistance' , maxAngularDistance);
                model_arr{i,2} = seg_ptCloud;
                if (model.Normal == [0 0 0])
                    model_arr{i,3} = [];
                    model_arr{i,1} = [];
                    continue;
                end
                model_arr{i,3} = select(seg_ptCloud, inlierIndices);
                tform = normalRotation(model,referenceVector);
                planeParams = model.Parameters * tform.T;
                transformedPlane = planeModel(planeParams);
                model_arr{i,1} = transformedPlane;
                dist_v = horzcat(dist_v, [model_arr{i,1}.Parameters(4); i]);
            end
        
            % Find plane models in y-direction
            referenceVector = [0,1,0];
            ptCloud_y = denoised_ptCloud;
            model_arr2 = {};
            dist_h = [];
        
            % Define maximum angular and maximum distance of the points to
            % the fitted plane
            maxDistance = 0.1;
            maxAngularDistance = 5;
            % Test
        
            for i = 1:numClusters
                idxValidPoints = find(labels == i);
                seg_ptCloud = select(ptCloud_y, idxValidPoints);
                [model, inlierIndices, ~] = pcfitplane(seg_ptCloud, maxDistance, 'referenceVector', referenceVector, 'maxAngularDistance' , maxAngularDistance);
                model_arr2{i,2} = seg_ptCloud;
                if (model.Normal == [0 0 0])
                    model_arr2{i,3} = [];
                    model_arr2{i,1} = [];
                    continue;
                end
                model_arr2{i,3} = select(seg_ptCloud, inlierIndices);
                tform = normalRotation(model,referenceVector);
                planeParams = model.Parameters * tform.T;
                transformedPlane = planeModel(planeParams);
                model_arr2{i,1} = transformedPlane;
                dist_h = horzcat(dist_h, [model_arr2{i,1}.Parameters(4); i]);
            end
        
            plane_model_v = model_arr;
            plane_model_h = model_arr2; 
        end
        
        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function that clusters the point cloud, rotates the complete corrdinate
        % system and provides a filter point cloud and planes according to the
        % segements
        % =========================================================================
        
        function [plane_model_v, plane_model_h, plane_model_gc, rot_ptCloud, denoised_ptCloud, seg_PtCloud, dist_v, dist_h, deg] = seg_in_plane2(obj, ptCloud_fi, ptCloud)
            %Create clusters from the filter point cloud
            minDistance = 0.5;
            minPoints = 100;
            [labels, ~] = pcsegdist(ptCloud_fi, minDistance, 'NumClusterPoints',minPoints);
            idxValidPoints = find(labels == 1);
            segmentedPtCloud = select(ptCloud_fi,idxValidPoints);
        
            %Rotate the point cloud that it is parallel to the vertical and
            %horizontal axis
            %Assume that the data includes a wall which can be fitted by a plane
            %model
            maxDistance = 0.1; % Old 0.1
            maxAngularDistance = 5;
        
            [model] = pcfitplane(segmentedPtCloud, maxDistance,  'maxAngularDistance' , maxAngularDistance);
            norm_vector = model.Normal;
            % Determine angle between normal vector of plane and x-axis
            x1 = [1;0];
            x2 = [norm_vector(1); norm_vector(2)];
            rad = subspace(x1,x2);
            deg = rad2deg(rad);
        
            % Rotate point cloud according to the determined angle 
            rotationAngles = [0 0 deg];
            translation = [0 0 0];
            tform1 = rigidtform3d(rotationAngles,translation);
            rot_ptCloud = pctransform(ptCloud,tform1);
        
            % Filter rotated point cloud
            rot_ptCloud2 = pcdenoise(rot_ptCloud, 'NumNeighbors', 5, 'Threshold', 0.1);
            denoised_ptCloud = rot_ptCloud2;
            rot_ptCloud2 = pcdenoise(rot_ptCloud2, 'NumNeighbors', 10, 'Threshold', 0.05);
        
            % Find plane models in z-direction (ground and ceiling)
            maxDistance = 0.2;
            maxAngularDistance = 15;
            referenceVector = [0,0,1];
            ptCloud_z = rot_ptCloud2;
            
            % Define ROI to find a plane in the upper region of the point cloud for
            % the ceiling and in the lower region to find a ground plane
            % Usage of the filter point cloud to find a suitable plane
        
            % Search for a ground plane
            % Specify new z limits to only search in the lower region of the point
            % cloud
            model_arr3 = {};
          
            zLimit = [ptCloud_z.ZLimits(1) ptCloud_z.ZLimits(1)+obj.find_scaling_factor(ptCloud_z)]; % Redefine this parameter!!!!!! 0.2
            roi = [ptCloud_z.XLimits(1) ptCloud_z.XLimits(2) ptCloud_z.YLimits(1) ptCloud_z.YLimits(2) zLimit(1) zLimit(2)];
            indices = findPointsInROI(ptCloud_z,roi);
            ptCloud_Ground = select(ptCloud_z,indices);
            %[groundPtsIdx,nonGroundPtCloud,groundPtCloud] = segmentGroundSMRF(ptCloud);
            model_arr3{1,2} = ptCloud_Ground;
        
            [model, ~, ~] = pcfitplane(ptCloud_Ground, maxDistance, 'referenceVector', referenceVector, 'maxAngularDistance' , maxAngularDistance);
            tform = normalRotation(model,referenceVector);
            planeParams = model.Parameters * tform.T;
            transformedPlane = planeModel(planeParams);
            model_arr3{1,1} = transformedPlane;
        
            % Search for a ceiling in the upper region of the point cloud
            zLimit = [ptCloud_z.ZLimits(2)-obj.find_scaling_factor(ptCloud_z) ptCloud_z.ZLimits(2)]; % Redefine this parameter!!!!!! 
            roi = [ptCloud_z.XLimits(1) ptCloud_z.XLimits(2) ptCloud_z.YLimits(1) ptCloud_z.YLimits(2) zLimit(1) zLimit(2)];
            indices = findPointsInROI(ptCloud_z,roi);
            ptCloud_Ground = select(ptCloud_z,indices);
            model_arr3{2,2} = ptCloud_Ground;
        
            [model, ~, ~] = pcfitplane(ptCloud_Ground, maxDistance, 'referenceVector', referenceVector, 'maxAngularDistance' , maxAngularDistance);
            tform = normalRotation(model,referenceVector);
            planeParams = model.Parameters * tform.T;
            transformedPlane = planeModel(planeParams);
            model_arr3{2,1} = transformedPlane;
        
            % Remove Ceiling and Ground plane from point cloud
            denoised_ptCloud = obj.remove_ground_points(denoised_ptCloud, model_arr3);
        
            % Cluster point before further porcessing 
            minDistance = 0.5;
            minPoints = 100;
            [labels, numClusters] = pcsegdist(denoised_ptCloud, minDistance, 'NumClusterPoints',minPoints);
            idxValidPoints = find(labels);
            seg_PtCloud{1}{2} = labels(idxValidPoints);
            seg_PtCloud{1}{1} = select(denoised_ptCloud,idxValidPoints);
        
            % Find plane models in x-direction
            referenceVector = [1,0,0];
            ptCloud_x = denoised_ptCloud;
            model_arr = {};
            dist_v = [];
        
            % Test
            maxDistance = 0.1; % Old 0.1
            maxAngularDistance = 5;
            % Test
        
            for i = 1:numClusters
                idxValidPoints = find(labels == i);
                seg_ptCloud = select(ptCloud_x, idxValidPoints);
                [model, inlierIndices, ~] = pcfitplane(seg_ptCloud, maxDistance, 'referenceVector', referenceVector, 'maxAngularDistance' , maxAngularDistance);
                model_arr{i,2} = seg_ptCloud;
                if (model.Normal == [0 0 0])
                    model_arr{i,3} = [];
                    model_arr{i,1} = [];
                    continue;
                end
                model_arr{i,3} = select(seg_ptCloud, inlierIndices);
                tform = normalRotation(model,referenceVector);
                planeParams = model.Parameters * tform.T;
                transformedPlane = planeModel(planeParams);
                model_arr{i,1} = transformedPlane;
                dist_v = horzcat(dist_v, [model_arr{i,1}.Parameters(4); i]);
            end
        
            % Find plane models in y-direction
            referenceVector = [0,1,0];
            ptCloud_y = denoised_ptCloud;
            model_arr2 = {};
            dist_h = [];
        
            % Test
            maxDistance = 0.1; % Old 0.1
            maxAngularDistance = 5;
            % Test
        
            for i = 1:numClusters
                idxValidPoints = find(labels == i);
                seg_ptCloud = select(ptCloud_y, idxValidPoints);
                [model, inlierIndices, ~] = pcfitplane(seg_ptCloud, maxDistance, 'referenceVector', referenceVector, 'maxAngularDistance' , maxAngularDistance);
                model_arr2{i,2} = seg_ptCloud;
                if (model.Normal == [0 0 0])
                    model_arr2{i,3} = [];
                    model_arr2{i,1} = [];
                    continue;
                end
                model_arr2{i,3} = select(seg_ptCloud, inlierIndices);
                tform = normalRotation(model,referenceVector);
                planeParams = model.Parameters * tform.T;
                transformedPlane = planeModel(planeParams);
                model_arr2{i,1} = transformedPlane;
                dist_h = horzcat(dist_h, [model_arr2{i,1}.Parameters(4); i]);
            end
        
            plane_model_v = model_arr;
            plane_model_h = model_arr2; 
            plane_model_gc = model_arr3;
        end
        
        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function Name :   Find horizontal and vertical planes in point
        %                   cloud
        %
        % Description   :   The function searches for horizontal and
        %                   vertial planes in a point cloud. Afterwards all
        %                   planes are being rotated to get optimal vertial
        %                   and horizontal planes. The function returns the
        %                   plane equation and the points of the plane
        % 
        % Inputs        :   Point cloud (ptCloud)
        %
        % Outputs       :   1. Vertical planes (cell array - plane equation +
        %                      points on the plane)
        %                   2. Horizontal planes (cell array - plane equation +
        %                      points on the plane)
        %                   3. Distance in x-direction
        %                   4. Distance in y-direction
        % =========================================================================
        function [g_plane_model_v, g_plane_model_h, g_dist_v, g_dist_h] = global_plane(obj, denoised_ptCloud)
           
            % Filter rotated point cloud before searching
            rot_ptCloud2 = pcdenoise(denoised_ptCloud, 'NumNeighbors', 10, 'Threshold', 0.05);
        
            % Find plane models in x-direction
            referenceVector = [1,0,0];
            ptCloud_x = rot_ptCloud2;
            model_arr = {20,2};
            g_dist_v = [];
        
            % Set maximum distance and angular
            maxDistance = 0.1; 
            maxAngularDistance = 5;
          
            % Search in x-direction and rotated these planes
            for i = 1:20
                [model, inlierIndices, outlierIndices] = pcfitplane(ptCloud_x, maxDistance, 'referenceVector', referenceVector, 'maxAngularDistance' , maxAngularDistance);
                if (model.Normal == [0 0 0])
                    break;
                end
                model_arr{i,2} = select(ptCloud_x, inlierIndices);
                ptCloud_x = select(ptCloud_x,outlierIndices);
                tform = normalRotation(model,referenceVector);
                planeParams = model.Parameters * tform.T;
                transformedPlane = planeModel(planeParams);
                model_arr{i,1} = transformedPlane;
                g_dist_v = horzcat(g_dist_v, [model_arr{i,1}.Parameters(4); i]);
            end
        
            % Find plane models in y-direction
            referenceVector = [0,1,0];
            ptCloud_y = rot_ptCloud2;
            model_arr2 = {};
            g_dist_h = [];
        
            % Search in y-direction and rotated these planes
            for i = 1:20
                [model, inlierIndices, outlierIndices] = pcfitplane(ptCloud_y, maxDistance, 'referenceVector', referenceVector, 'maxAngularDistance' , maxAngularDistance);
                if (model.Normal == [0 0 0])
                    break;
                end
                model_arr2{i,2} = select(ptCloud_y, inlierIndices);
                ptCloud_y = select(ptCloud_y,outlierIndices);
                tform = normalRotation(model,referenceVector);
                planeParams = model.Parameters * tform.T;
                transformedPlane = planeModel(planeParams);
                model_arr2{i,1} = transformedPlane;
                g_dist_h = horzcat(g_dist_h, [model_arr2{i,1}.Parameters(4); i]);
            end
        
            % Return plane models and points of planes as cell array
            g_plane_model_v = model_arr;
            g_plane_model_h = model_arr2; 
        end
        
        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function Name :   Determination of a scale factor (ZLimits)
        %
        % Description   :   Function calculates a scale value according to 
        %                   the z-dimension of the point cloud
        % 
        % Inputs        :   Point cloud (ptCloud)
        %
        % Outputs       :   Scale factor
        % =========================================================================
        function [scale] = find_scaling_factor(obj, ptCloud)
            scale = norm(ptCloud.ZLimits) * 0.065; %0.065
        end

        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function Name :   Determination of a scale factor (XLimits) 
        %
        % Description   :   Function calculates a scale value according to 
        %                   the x-dimension of the point cloud
        % 
        % Inputs        :   Point cloud (ptCloud)
        %
        % Outputs       :   Scale factor
        % =========================================================================
        function [scale] = find_scaling_factor2(obj, ptCloud)
            scale = norm(ptCloud.XLimits) * 0.013; %0.015
        end

        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function Name :   Determination of a scale factor 2 (ZLimits) 
        %
        % Description   :   Function calculates a scale value according to 
        %                   the z-dimension of the point cloud for the
        %                   removment of the ceiling points
        % 
        % Inputs        :   Point cloud (ptCloud)
        %
        % Outputs       :   Scale factor
        % =========================================================================
        function [scale] = find_scaling_factor3(obj, ptCloud)
            scale = (norm(ptCloud.ZLimits)/4.6)^2;
        end
        
        %% ------------------------------------------------------------------------ 
        % =========================================================================
        % Function Name :   Select best suitable planes
        %
        % Description   :   The function checks if the generated plane fits
        %                   to the point cloud according to some metrics
        % 
        % Inputs        :   1. Global plane models
        %                   2. Local plane models
        %                   3. Global distances
        %                   4. Local distances
        %                   5. Margin - min. distance between two planes
        %
        % Outputs       :   1. Selected global plane models
        %                   2. Selected local plane models
        %                   3. All selected plane models
        %                   4. Distance values of the planes
        % =========================================================================
        function [g_plane_model, l_plane_model, plane_model, dist_out] = select_plane2(obj, plane_global, plane_local, g_dist, l_dist, margin)
            % Sort the local and global plane distances to find the outer
            % planes
            if ~isempty(g_dist)
                g_dist = sortrows(g_dist',1)';
            end

            l_dist = sortrows(l_dist',1)';
        
            sz_g = size(g_dist);
            sz_l = size(l_dist);
            id_g = 1;
            dist = []; 
            dist_out = [];
            g_plane_model{1,1} = 0;
            l_plane_model{1,1} = 0;
            
            % Select planes which are above a certain threshold value of
            % points
            for i = 1:sz_g(2) 
                if (plane_global{i,2}.Count > 400 && ~isempty(plane_global{i,1}))       %900
                    g_plane_model{id_g,1} = plane_global{i,1};
                    g_plane_model{id_g,2} = plane_global{i,2};
                    dist = horzcat(dist,[plane_global{i,1}.Parameters(4); 1; id_g]);
                    id_g = id_g + 1;
                end
            end
        
            id_l = 1;
        
            for i = 1:sz_l(2) 
                if (plane_local{i,2}.Count > 200 && ~isempty(plane_local{i,1}))         %600
                    l_plane_model{id_l,1} = plane_local{i,1};
                    l_plane_model{id_l,2} = plane_local{i,2};
                    l_plane_model{id_l,3} = plane_local{i,3};
                    dist = horzcat(dist, [plane_local{i,1}.Parameters(4); 2; id_l]);
                    id_l = id_l + 1;
                end
            end
            
            % Sort the planes according to their distance and remove those
            % which are lying to close together
            for i = 1:length(g_dist(1,:))
                if (g_dist(1,i) <= l_dist(1,i)) && (plane_global{g_dist(2, i),2}.Count > 100)
                    g_plane_model{id_g,1} = plane_global{g_dist(2, i),1};
                    g_plane_model{id_g,2} = plane_global{g_dist(2, i),2};
                    a = [plane_global{g_dist(2, i),1}.Parameters(4); 1 ; id_g];
                    dist = horzcat(dist, a);
                    id_g = id_g + 1;
                    break;
                elseif (plane_local{l_dist(2, i),2}.Count > 100)
                    l_plane_model{id_l,1} = plane_local{l_dist(2, i),1};
                    l_plane_model{id_l,2} = plane_local{l_dist(2, i),2};
                    l_plane_model{id_l,3} = plane_local{l_dist(2, i),3};
                    dist = horzcat(dist, [plane_local{l_dist(2, i),1}.Parameters(4); 2; id_l]);
                    id_l = id_l + 1;
                    break;
                end
            end
        
            for i = 1:length(g_dist(1,:))
                if (g_dist(1,end+1-i) <= l_dist(1,end+1-i))  && (plane_global{g_dist(2,end+1-i),2}.Count > 100)
                    g_plane_model{id_g,1} = plane_global{g_dist(2,end+1-i),1};
                    g_plane_model{id_g,2} = plane_global{g_dist(2,end+1-i),2};
                    dist = horzcat(dist, [plane_global{g_dist(2,end+1-i),1}.Parameters(4); 1; id_g]);
                    break;
                elseif (plane_local{l_dist(2, end+1-i),2}.Count > 100)
                    l_plane_model{id_l,1} = plane_local{l_dist(2, end+1-i),1};
                    l_plane_model{id_l,2} = plane_local{l_dist(2, end+1-i),2};
                    l_plane_model{id_l,3} = plane_local{l_dist(2, end+1-i),3};
                    dist = horzcat(dist, [plane_local{l_dist(2, end+1-i),1}.Parameters(4); 2; id_l]);
                    break;
                end
            end
        
            dist = sortrows(dist',1)';
            sz = size(dist);
            d = [];
            i = 1;
            id = 1;
        
            while (i <= sz(2))
        
                if i == sz(2)
                    diff = margin + 1;
                else
                    diff = abs(dist(1,i) - dist(1,i+1));
                end
        
                if diff > margin
                    if dist(2,i) == 1
                        plane_model{id,1} = g_plane_model{dist(3,i),1};
                        plane_model{id,2} = g_plane_model{dist(3,i),2};
                        dist_out = horzcat(dist_out, [plane_model{id,1}.Parameters(4); 1; id]);
                        id = id + 1;
                    else
                        plane_model{id,1} = l_plane_model{dist(3,i),1};
                        plane_model{id,2} = l_plane_model{dist(3,i),2};
                        plane_model{id,3} = l_plane_model{dist(3,i),3};
                        dist_out = horzcat(dist_out, [plane_model{id,1}.Parameters(4); 1; id]);
                        id = id + 1;
                    end
                    i = i + 1;
                else
                    if ((i+2) <= sz(2))
                        diff2 = abs(dist(1,i+1) - dist(1,i+2));
                    else
                        diff2 = margin + 1;
                    end
        
                    if diff2 <= margin
                        if dist(2,i) == 1
                            p1 = g_plane_model{dist(3,i),2}.Count;
                        else
                            p1 = l_plane_model{dist(3,i),2}.Count;
                        end
        
                        d = horzcat(d, [p1; dist(2:3,i)]);
                        i = i + 1;
                        continue;
                    else
                        if dist(2,i) == 1
                            p1 = g_plane_model{dist(3,i),2}.Count;
                        else
                            p1 = l_plane_model{dist(3,i),2}.Count;
                        end
                        d = horzcat(d, [p1; dist(2:3,i)]);
        
                        if dist(2,i+1) == 1
                            p1 = g_plane_model{dist(3,i+1),2}.Count;
                        else
                            p1 = l_plane_model{dist(3,i+1),2}.Count;
                        end
                        d = horzcat(d, [p1; dist(2:3,i)]);
                    end
        
                    [~,pos] = max(d(1,:));
                    ind_plane = d(3,pos);
        
                    if d(2,pos) == 1
                        plane_model{id,1} = g_plane_model{ind_plane,1};
                        plane_model{id,2} = g_plane_model{ind_plane,2};
                        dist_out = horzcat(dist_out, [plane_model{id,1}.Parameters(4); 1; id]);
                        id = id + 1;
                    else
                        plane_model{id,1} = l_plane_model{ind_plane,1};
                        plane_model{id,2} = l_plane_model{ind_plane,2};
                        plane_model{id,3} = l_plane_model{ind_plane,3};
                        dist_out = horzcat(dist_out, [plane_model{id,1}.Parameters(4); 1; id]);
                        id = id + 1;
                    end
                     i = i + 2;
                     d = [];
                end
            end
        end
        
        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function Name :   Inner point selection
        %
        % Description   :   The function removes all points which are lying
        %                   not in the inner part of a room with some
        %                   margin
        % 
        % Inputs        :   1. Point cloud (ptCloud)
        %                   2. Ground plane model
        %                   3. Margin value
        %
        % Outputs       :   1. Point cloud of the inner part
        % =========================================================================
        function [pgon1, pgon2, ptCloud_out] = remove_outer_points(obj, ptCloud, ground_plane, margin)
             % Remove all points from the filter point cloud that belong to the
             % walls, ground plane and the ceiling
             % First merge all plane point clouds

             pgon = polyshape(ground_plane(1,:),ground_plane(2,:));
             [x_poly,y_poly] = centroid(pgon);

             sz = size(pgon.Vertices);

             pgon1 = pgon;

             for i = 1:sz(1)
                 if pgon.Vertices(i,1) < x_poly
                    pgon.Vertices(i,1) = pgon.Vertices(i,1) + margin;
                 else
                    pgon.Vertices(i,1) = pgon.Vertices(i,1) - margin;
                 end

                 if pgon.Vertices(i,2) < y_poly
                    pgon.Vertices(i,2) = pgon.Vertices(i,2) + margin;
                 else
                    pgon.Vertices(i,2) = pgon.Vertices(i,2) - margin;
                 end
             end
        

             % for i = 1:sz(1)
             % 
             %     arr = [];
             % 
             %     x1 = pgon.Vertices(i,1) + margin;
             %     y1 = pgon.Vertices(i,2) + margin;
             % 
             %     dist = abs(x1-x_poly) + abs(y1-y_poly);
             %     arr = horzcat(arr, [x1;y1;dist]);
             % 
             %     x2 = pgon.Vertices(i,1) - margin;
             %     y2 = pgon.Vertices(i,2) + margin;
             % 
             %     dist = abs(x2-x_poly) + abs(y2-y_poly);
             %     arr = horzcat(arr, [x2;y2;dist]);
             % 
             %     x3 = pgon.Vertices(i,1) + margin;
             %     y3 = pgon.Vertices(i,2) - margin;
             % 
             %     dist = abs(x3-x_poly) + abs(y3-y_poly);
             %     arr = horzcat(arr, [x3;y3;dist]);
             % 
             %     x4 = pgon.Vertices(i,1) - margin;
             %     y4 = pgon.Vertices(i,2) - margin;
             % 
             %     dist = abs(x4-x_poly) + abs(y4-y_poly);
             %     arr = horzcat(arr, [x4;y4;dist]);
             % 
             %     [temp, order] = sort(arr(3,:));
             %     arr = arr(:,order);
             % 
             %     for j = 1:4
             %         if inpolygon(arr(1,j),arr(2,j),pgon.Vertices(:,1), pgon.Vertices(:,2))
             %            pgon.Vertices(i,1) = arr(1,j);
             %            pgon.Vertices(i,2) = arr(2,j);
             %            break;
             %         end
             %     end

                 % if inpolygon(x1,y1,pgon.Vertices(:,1), pgon.Vertices(:,2))
                 %    pgon.Vertices(i,1) = x1;
                 %    pgon.Vertices(i,2) = y1;
                 % elseif inpolygon(x2,y2,pgon.Vertices(:,1), pgon.Vertices(:,2))
                 %    pgon.Vertices(i,1) = x2;
                 %    pgon.Vertices(i,2) = y2;
                 % elseif inpolygon(x3,y3,pgon.Vertices(:,1), pgon.Vertices(:,2))
                 %    pgon.Vertices(i,1) = x3;
                 %    pgon.Vertices(i,2) = y3;
                 %  elseif inpolygon(x4,y4,pgon.Vertices(:,1), pgon.Vertices(:,2))
                 %    pgon.Vertices(i,1) = x4;
                 %    pgon.Vertices(i,2) = y4;
                 % end

                 % x = pgon.Vertices(i,1) + margin;
                 % y = pgon.Vertices(i,2);
                 % 
                 % [in,on] = inpolygon(x,y,pgon.Vertices(:,1), pgon.Vertices(:,2));
                 % 
                 % if in && not(on)
                 %    pgon.Vertices(i,1) = pgon.Vertices(i,1) + margin;
                 % else
                 %    pgon.Vertices(i,1) = pgon.Vertices(i,1) - margin;
                 % end
                 % 
                 % x = pgon.Vertices(i,1);
                 % y = pgon.Vertices(i,2) + margin;
                 % 
                 % [in,on] = inpolygon(x,y,pgon.Vertices(:,1), pgon.Vertices(:,2));
                 % 
                 % if in && not(on)
                 %   pgon.Vertices(i,2) = pgon.Vertices(i,2) + margin;
                 % else
                 %   pgon.Vertices(i,2) = pgon.Vertices(i,2) - margin;
                 % end
             % end

             pgon2 = pgon;

             x_ptCloud = ptCloud.Location(:,1);
             y_ptCloud = ptCloud.Location(:,2);
             z_ptCloud = ptCloud.Location(:,3);
             
             [id, ~] = inpolygon(x_ptCloud,y_ptCloud,pgon.Vertices(:,1), pgon.Vertices(:,2));
             corrd = horzcat(x_ptCloud(id), y_ptCloud(id), z_ptCloud(id));
             ptCloud_out = pointCloud(corrd);
        end
        
        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function Name :   Remove ground and ceiling points
        %
        % Description   :   Removes all points in point cloud with belong
        %                   to the ground and ceiling plane model
        % 
        % Inputs        :   1. Point cloud (ptCloud)
        %                   2. Plane models of the ground and ceiling
        %
        % Outputs       :   1. Point cloud without ground and ceiling
        %                      points
        % =========================================================================
        function filtered_ptCloud = remove_ground_points(obj, ptCloud, plane_gc)
             % Remove all points from the filter point cloud that belong to the
             % walls, ground plane and the ceiling
             % First merge all plane point clouds
             pt1 = plane_gc{1,2}.Location;
             pt2 = plane_gc{2,2}.Location;
             
             pt = vertcat(pt1,pt2);
             pt = unique(pt,'rows');
             ptCloudOut = pointCloud(pt);
        
             % Detect corresponding points and remove those points
             [~,ind] = ismember(ptCloudOut.Location, ptCloud.Location,'rows');
             ind(ind==0) = [];
             a = ptCloud.Location;
             a(ind,:) = [];
             filtered_ptCloud = pointCloud(a);
        end

        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function Name :   Remove ceiling points
        %
        % Description   :   Removes all points that belong to the ceiling
        %                   according to ROI
        % 
        % Inputs        :   1. Point cloud (ptCloud)
        %
        % Outputs       :   1. Point cloud without 
        % =========================================================================
        function filtered_ptCloud = remove_ceiling_points(obj, ptCloud)
             % Remove all points from the filter point cloud that belong to the
             % walls, ground plane and the ceiling
             % First merge all plane point clouds
             %pt1 = plane_gc{1,2}.Location;
             %pt2 = plane_gc{2,2}.Location;

             ptCloud_z = ptCloud;

             zLimit = [ptCloud_z.ZLimits(2)-obj.find_scaling_factor3(ptCloud_z) ptCloud_z.ZLimits(2)]; % Redefine this parameter!!!!!! 0.2 
             roi = [ptCloud_z.XLimits(1) ptCloud_z.XLimits(2) ptCloud_z.YLimits(1) ptCloud_z.YLimits(2) zLimit(1) zLimit(2)];
             indices = findPointsInROI(ptCloud_z,roi);
             ptCloudOut = select(ptCloud_z,indices);
        
             % Detect corresponding points and remove those points
             [~,ind] = ismember(ptCloudOut.Location, ptCloud.Location,'rows');
             ind(ind==0) = [];
             a = ptCloud.Location;
             a(ind,:) = [];
             filtered_ptCloud = pointCloud(a);
        end
        
        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function Name :   Calculate Intersection Points
        %
        % Description   :   Calculates the intersection points between a
        %                   vertical, horziontal and ground plane
        % 
        % Inputs        :   1. Vertical plane models
        %                   2. Horizontal plane models
        %                   3. Ground plane model
        %
        % Outputs       :   1. Intersection points (polygon format)
        %                   2. Points (x,y,z)
        % =========================================================================
        function [intersectionPoints, points] = calculate_Intersection_Points(obj, plane_v, plane_h, plane_g)
        
            sz = size(plane_v);
            num_Planes_v = sz(1);
        
            sz = size(plane_h);
            num_Planes_h = sz(1);
            
            dep = num_Planes_v * num_Planes_h;
            polyg = zeros(3,4,dep);

            points = [];
        
            % Loop through each pair of planes
            for i = 1:num_Planes_v
        
                % Caculate the first two intersection points
                a1 = plane_v{i,1}.Parameters(1:3);
                a2 = plane_h{1,1}.Parameters(1:3);
                a3 = plane_g{1,1}.Parameters(1:3);
                A = [a1; a2; a3];
                B = [plane_v{i,1}.Parameters(4); plane_h{1,1}.Parameters(4); plane_g{1,1}.Parameters(4)];
                vec1 = linsolve(A,B);

                points = horzcat(points, vec1);
        
                a3 = plane_g{2,1}.Parameters(1:3);
                A = [a1; a2; a3];
                B = [plane_v{i,1}.Parameters(4); plane_h{1,1}.Parameters(4); plane_g{2,1}.Parameters(4)];
                vec2 = linsolve(A,B);

        
                for j = 2:num_Planes_h
                    % Assign the first two vectors to the array
                    polyg(:,1,(i-1)*(num_Planes_h-1)+j-1) = vec1;
                    polyg(:,2,(i-1)*(num_Planes_h-1)+j-1) = vec2;
        
                    a1 = plane_v{i,1}.Parameters(1:3);
                    a2 = plane_h{j,1}.Parameters(1:3);
        
                    a3 = plane_g{2,1}.Parameters(1:3);
                    A = [a1; a2; a3];
                    B = [plane_v{i,1}.Parameters(4); plane_h{j,1}.Parameters(4); plane_g{2,1}.Parameters(4)];
                    vec2 = linsolve(A,B);
                    polyg(:,3,(i-1)*(num_Planes_h-1)+j-1) = vec2;
                    
                    a3 = plane_g{1,1}.Parameters(1:3);
                    A = [a1; a2; a3];
                    B = [plane_v{i,1}.Parameters(4); plane_h{j,1}.Parameters(4); plane_g{1,1}.Parameters(4)];
                    vec1 = linsolve(A,B);
                    polyg(:,4,(i-1)*(num_Planes_h-1)+j-1) = vec1;
                    points = horzcat(points, vec1);
        
               end
            end
        
            intersectionPoints = polyg;
        end
        
        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function Name :   Remove double polygons
        %
        % Description   :   Function detects and removes double polygons
        % 
        % Inputs        :   1. List of polygons
        %
        % Outputs       :   1. Filtered list of polygons
        % =========================================================================
        function [patches_out, A_ru2] = remove_dup_patches(obj, patches)
            [s1,s2,s3] = size(patches);
            % Reshape the slices into rows (by working down column-wise, and then across columns)
            A_r = reshape(patches,s1*s2,s3,1)';
            % Find the unique rows (which corresponds to unique slices of original array)
            A_ru = unique(A_r,'rows','stable');
            % Reshape the rows back to 3-d
            A_ru = reshape(A_ru',s1,s2,[]);
            a = all(A_ru(1,:,:) == A_ru(1,1,:));
            b = all(A_ru(2,:,:) == A_ru(2,1,:));
            ind = ~(a==b);
            A_ru2 = A_ru(:,:,ind);

            [~,~,s3] = size(A_ru2);
            A_r2 = fliplr(A_ru2(:,:,:));
     
            for i = 1:s3
                ind = ismember(A_ru2, A_r2(:,:,i));
                ind = all(ind(:,:,:));
                ind = all(ind(:,:,:));

                if nnz(ind) >= 2
                    A_ru2 = A_ru2(:,:,~ind);
                end
            end
            patches_out = A_ru2;
        end

        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function Name :   Polygon Plot 1
        %
        % Descprition   :   The function gets the vertical and horizontal
        %                   polygons and plot these polygons
        % 
        % Inputs        :   1. Vertical polygons
        %                   2. Horizontal polygons
        %
        % Outputs       :   -
        % =========================================================================
        function [] = plot_patches(obj, v_patches, h_patches)
            num_v = size(v_patches);
            num_v = num_v(3);
        
            num_h = size(h_patches);
            num_h = num_h(3);
        
            x = v_patches(1,:,1);
            y = v_patches(2,:,1);
            z = v_patches(3,:,1);
            patch(x,y,z, [0.5 0.5 0.5]);
        
            for i = 2:num_v
                hold on;
                x = v_patches(1,:,i);
                y = v_patches(2,:,i);
                z = v_patches(3,:,i);
                patch(x,y,z, [0.5 0.5 0.5]);
            end
        
            for i = 1:num_h
                hold on;
                x = h_patches(1,:,i);
                y = h_patches(2,:,i);
                z = h_patches(3,:,i);
                patch(x,y,z, [0.5 0.5 0.5]);
            end
        end

        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function Name :   Polygon Plot 2
        %
        % Descprition   :   The function gets polygons and plot these polygons
        % 
        % Inputs        :   1. Polygons
        %
        % Outputs       :   -
        % =========================================================================
       
        function [] = plot_patches2(obj, patches, handle)
            num_v = size(patches);
            num_v = num_v(3);
        
            x = patches(1,:,1);
            y = patches(2,:,1);
            z = patches(3,:,1);
            patch(x,y,z, [0.5 0.5 0.5], 'Parent', handle);
        
            for i = 2:num_v
                hold(handle,'on')
                x = patches(1,:,i);
                y = patches(2,:,i);
                z = patches(3,:,i);
                patch(x,y,z, [0.5 0.5 0.5], 'Parent', handle);
            end
        end
        %% ------------------------------------------------------------------------

        function [] = plot_patches3(obj, patches)
            num_v = size(patches);
            num_v = num_v(3);
        
            x = patches(1,:,1);
            y = patches(2,:,1);
            z = patches(3,:,1);
            patch(x,y,z, [0.5 0.5 0.5]);
        
            for i = 2:num_v
                hold on;
                x = patches(1,:,i);
                y = patches(2,:,i);
                z = patches(3,:,i);
                patch(x,y,z, [0.5 0.5 0.5]);
            end
        end
        % =========================================================================
        % Function Name :   Generate ground and ceiling polygons
        %
        % Descprition   :   The function generates a ground polygon
        % 
        % Inputs        :   1. Intersection points of the ground
        %                   2. Ground values 
        %                   3. Ceiling values
        %
        % Outputs       :   1. Points of the ground polygon
        % =========================================================================
        % =========================================================================
        % Function to create patches from ground points - current
        % =========================================================================
        
        function [intersectionPoints] = generate_patches(obj, ground_pt, ground_val, ceil_val)
  
            sz = size(ground_pt);
            num_points = sz(1);
            polyg = zeros(3,4,10);

            vec1 = [ground_pt(1,1:2), ground_val]';
            vec2 = [ground_pt(1,1:2), ceil_val]';
        
            % Loop through each pair of planes
            for i = 1:num_points-1
                polyg(:,1,i) = vec1;
                polyg(:,2,i) = vec2;

                vec1 = [ground_pt(i+1,1:2), ground_val]';
                vec2 = [ground_pt(i+1,1:2), ceil_val]';

                polyg(:,4,i) = vec1;
                polyg(:,3,i) = vec2;
            end
        
            intersectionPoints = polyg;
        end

        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function Name :   Detect outer structure
        %
        % Descprition   :   The function searches for planes that belong to
        %                   the outer structure of the room
        % 
        % Inputs        :   1. Point cloud (ptCloud)
        %
        % Outputs       :   1. Vertical plane models
        %                   2. Horizontal plane models
        %                   3. Distances in vertical direction
        %                   4. Distances in horizontal direction
        % =========================================================================
        function [g_plane_model_v, g_plane_model_h, g_dist_v, g_dist_h] = outer_planes(obj, denoised_ptCloud)
           
            model_arr = {};
            g_dist_v = [];
        
            % Add planes marked by the outest points
            p1 = planeModel([1,0,0,-denoised_ptCloud.XLimits(1)]);
            model_arr{1,1} = p1;
            g_dist_v = horzcat(g_dist_v, [p1.Parameters(4); 2; 1]);
            p1 = planeModel([1,0,0,-denoised_ptCloud.XLimits(2)]);
            model_arr{2,1} = p1;
            g_dist_v = horzcat(g_dist_v, [p1.Parameters(4); 2; 2]);

            model_arr2 = {};
            g_dist_h = [];
        
            % Add planes marked by the outest points
            p2 = planeModel([0,1,0,-denoised_ptCloud.YLimits(1)]);
            model_arr2{1,1} = p2;
            g_dist_h = horzcat(g_dist_h, [p2.Parameters(4); 2; 1]);
            p2 = planeModel([0,1,0,-denoised_ptCloud.YLimits(2)]);
            model_arr2{2, 1} = p2;
            g_dist_h = horzcat(g_dist_h, [p2.Parameters(4); 2; 2]);

            g_plane_model_v = model_arr;
            g_plane_model_h = model_arr2; 
        end

        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function Name :   Select outer planes
        %
        % Descprition   :   The function gets a bundle of planes and checks
        %                   which cloud belong to the outer structure of a
        %                   room.
        % 
        % Inputs        :   1. Plane models 1
        %                   2. Plane models 2
        %                   3. Distances 1
        %                   4. Distances 2
        %                   5. Margin values 
        %
        % Outputs       :   1. Cell array with plane models 
        % =========================================================================
        
        function [plane_model] = select_outer_plane(obj, plane_in1, plane_in2, dist_x1, dist_x2, margin_low)
           
            dist_in1 = sortrows(dist_x1',1)';
            dist_in2 = sortrows(dist_x2',1)';

            plane_model = plane_in1;
            sz = size(plane_model);
            i = 1;
            diff_min = abs(dist_in1(1,1)-dist_in2(1,1));
            diff_max = abs(dist_in1(1,end)-dist_in2(1,end));

            % Check if planes are lying to close together and keep only the
            % plane that is closest to the point cloud.

            if (diff_min < margin_low)
                plane_model{dist_in1(3,1),1} = plane_in2{dist_in2(3,1)};
            else 
                plane_model{sz(1)+i,1} = plane_in2{dist_in2(3,1)};
                i = i + 1;
            end

            if (diff_max < margin_low)
                plane_model{dist_in1(3,end),1} = plane_in2{dist_in2(3,end)};
            else 
                plane_model{sz(1)+i,1} = plane_in2{dist_in2(3,end)};
            end
        end

        %% ------------------------------------------------------------------------
        % =========================================================================
        % Function Name :   Point cloud segmentation
        %
        % Descprition   :   The function semgments a point cloud in
        %                   multiple cluster and stores it in a cell array
        % 
        % Inputs        :   1. Point cloud (ptCloud
        %                   2. Min distance between the points
        %                   3. Minimum points of a cluster
        %
        % Outputs       :   1. Cell array with point cloud clusters
        %                   2. Number of generated clusters 
        % =========================================================================
       
        function [segmentedPtCloud, numClusters] = segment_dataPt(obj, ptCloud, minDistance, minPoints)
            %minDistance = 0.2;
            %minPoints = 10;
            [labels,numClusters] = pcsegdist(ptCloud,minDistance,'NumClusterPoints',minPoints);

            for i = 1:numClusters
                idxValidPoints = find(labels == i);
                segmentedPtCloud{i,1} = select(ptCloud,idxValidPoints);
            end
          
        end
   end
end