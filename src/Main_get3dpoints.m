function [points_3D,filteredPoints] = Main_get3dpoints(imageDir, cameraParam)
    %get intrinsic camera parameter from given txt file
    cameraParam = importdata(cameraParam,' ',3);
    camParam = cameraParam.data;

    % camera matrix K and camera intrinsics object
    [K,intrinsics] = intrinsicsParam(camParam);

    % first try without image sorting and if it fails, then sort images first
    try
        viewidtrue=0;
        viewID_match=0;
    [points_3D,filteredPoints] = get3dpoints(viewID_match,imageDir,intrinsics,viewidtrue);
    catch
    [features,validPoints, indexPairs, numImages, tforms,bestMatchForImage ] = calculate_features('SURF');
     [indexPairs,validPoints,viewID_match, matchedPoints1,matchedPoints2] = convert_data(bestMatchForImage,validPoints);
     viewidtrue=1;
      [points_3D,filteredPoints] = get3dpoints(viewID_match,imageDir,intrinsics,viewidtrue);
    end
  
function [K, intrin] = intrinsicsParam(camParam)
    
    % camera parameters from txt file
    Kmatrix.imageSize = [camParam(1) camParam(2)];
    Kmatrix.focalLength = [camParam(3) camParam(4)];
    Kmatrix.principalPoint = [camParam(5) camParam(6)];
    
    %create a camera intrinsics object
    intrin = cameraIntrinsics(Kmatrix.focalLength,Kmatrix.principalPoint,Kmatrix.imageSize);
    
    % Camera matrix
    K = [camParam(3) 0 camParam(5);
                0  camParam(4) camParam(6)
                0 0 1];
    
end

function [indexPairs,validPoints,viewID_match, matchedPoints1,matchedPoints2] = convert_data(bestMatchForImage,validPoints)
  
        validPoints = struct2cell(validPoints);
        bestMatchForImage = struct2cell(bestMatchForImage);
        indexPairs = cell(size(bestMatchForImage,3),1);
        matchedPoints=cell(size(bestMatchForImage,3),1);

        for i = 1:size(bestMatchForImage,3)
             viewID_match(i,1) = bestMatchForImage{1,:,i};
             viewID_match(i,2) = bestMatchForImage{2,:,i};
             indexPairs{i}=  bestMatchForImage{4,:,i};
             matchedPoints{i} = bestMatchForImage{5,:,i};
             matchedPoints1{i}=[matchedPoints{i}(:,1)  matchedPoints{i}(:,2)];
             matchedPoints2{i}=[matchedPoints{i}(:,3)  matchedPoints{i}(:,4)];
        end
        
end
end
