function [points_3D,filteredPoints] = get3dpoints(viewID_match,imageDir,intrinsics,viewtrue)

imds = imageDatastore(imageDir);

% Convert the images to grayscale.
images = cell(1, numel(imds.Files));
for i = 1:numel(imds.Files)
    I = readimage(imds, i);
    images{i} = im2gray(I);
end


I = images{1};


% set initial prevPoints and preFeatures
prevPoints   = detectSURFFeatures(I,NumOctaves=8);
prevFeatures = extractFeatures(I, prevPoints, Upright=true);

% create an imageviewset with view 1 of first image
vSet = imageviewset;
viewId = 1;
vSet = addView(vSet, viewId, rigidtform3d, Points=prevPoints);

for i = 2:numel(images)-1

    %if it failed it will be tried out with another image order
    if viewtrue ==1
        I = images{viewID_match(i,1)};
    else
        I = images{i};
    end

    % Detect, extract and feature matching process with current image
    currPoints   = detectSURFFeatures(I,NumOctaves=8);
    currFeatures = extractFeatures(I, currPoints, Upright=true);
    indexPairs   = matchFeatures(prevFeatures, currFeatures,MaxRatio=0.8,Unique=true,Method="Approximate",MatchThreshold=30);
    currentindexpairs = indexPairs;
    %if not enough matches found move to text image
    if size(indexPairs,1)<5
        continue
    end

    mp1 = prevPoints(indexPairs(:, 1));
    mp2 = currPoints(indexPairs(:, 2));

    % try to estimate essential matrix with different confidence and
    % maximal distance value if it failed

    try
        [estE,epipolarInliers] = estimateEssentialMatrix(mp1,mp2,intrinsics,'Confidence',60,'MaxDistance',0.5);
        mp1 = mp1(epipolarInliers, :);
        mp2 = mp2(epipolarInliers, :);
        relativePose = estrelpose(estE,intrinsics,mp1,mp2);
        % Get the table containing the previous camera pose.
        prevPose = poses(vSet, i-1).AbsolutePose;
   catch
            mp1 = prevPoints(indexPairs(:, 1));
            mp2 = currPoints(indexPairs(:, 2));
            [estE,epipolarInliers] = estimateEssentialMatrix(mp1,mp2,intrinsics,'Confidence',40,'MaxDistance',0.4);
            mp1 = mp1(epipolarInliers, :);
            mp2 = mp2(epipolarInliers, :);
            relativePose = estrelpose(estE,intrinsics,mp1,mp2);
            % Get the table containing the previous camera pose.
            try
            prevPose = poses(vSet, i-1).AbsolutePose;
            catch
                continue
            end
        
   end


    %  try to update current absolute pose 
    try
        currentPose = rigidtform3d(prevPose.A*relativePose.A);
    catch
        continue
    end

    % Add current view to the view set.
    vSet = addView(vSet, i, currentPose, Points=currPoints);

    ip1 = indexPairs(:, 2);
    ip2 = indexPairs(:, 1);

    indexPairs = [ip1 ip2];

    % create connection between two image views
    vSet = addConnection(vSet, i, i-1, relativePose, Matches=indexPairs(epipolarInliers,:));

    % point tracks and camera poses of vSet
    tracks = findTracks(vSet);
    cameraPoses = poses(vSet);

    % multiview triangulation and refining 
    points_3D = triangulateMultiview(tracks, cameraPoses, intrinsics);
    [points_3D, cameraPoses, Errors] = bundleAdjustment(points_3D, tracks, cameraPoses, intrinsics, FixedViewId=1, PointsUndistorted=true);

    % update the vSet
    vSet = updateView(vSet, cameraPoses);
    % update points
    prevFeatures = currFeatures;
    prevPoints   = currPoints;
end

% save only points with small error
goodIdx = (Errors < 5);
%filter outliers that are too far away from the main points cluster
points_3D = points_3D(goodIdx, :);
meanPoint = mean(points_3D, 1);
stdDev = std(points_3D, 0, 1);
validIndices = all(abs(points_3D - meanPoint) <= 3 * stdDev, 2);
filteredPoints = points_3D(validIndices, :);
end

