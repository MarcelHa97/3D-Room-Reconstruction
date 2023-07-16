function construct_Objects(floor_z, segmentedPtCloud, numClusters, handle)
    for i = 1:numClusters    
    % Extract the points and their z-coordinates
    points = segmentedPtCloud{i, 1}.Points;
    z_coords = points(:, 3);
    
    % Create a logical mask to filter out points below the floor_z
    mask = z_coords >= floor_z;
    
    % Apply the mask to keep only the points above or equal to the floor_z
    filtered_points = points(mask, :);
    
    
    
    hold(handle,'on')    
    %plot(alphaShape(filtered_points(:,1),filtered_points(:,2),filtered_points(:,3),1) ,'Parent', handle)
    
    filtered_points = pointCloud(filtered_points);
           
    % Fit a box to the filtered points
    boxModel = pcfitcuboid(filtered_points);
       
    
    % Plot the best fit box
    
    plot(boxModel, 'Parent', handle);

    end

end