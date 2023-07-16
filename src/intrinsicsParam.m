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