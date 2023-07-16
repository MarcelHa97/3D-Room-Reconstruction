function [features,validPoints, indexPairs, numImages, tforms,bestMatchForImage ] = calculate_features(method,imageDir)
% Create a map between method names and their corresponding functions

featureDetectionFunctions = containers.Map({'Harris', 'FAST', 'MinEigen', 'Brisk', 'MSER', 'SURF', 'KAZE', 'ORB','SIFT'}, {@detectHarrisFeatures, @detectFASTFeatures, @detectMinEigenFeatures, @detectBRISKFeatures, @detectMSERFeatures, @detectSURFFeatures, @detectKAZEFeatures, @detectORBFeatures,@detectSIFTFeatures});

% Check if the given method is valid
if ~featureDetectionFunctions.isKey(method)
    error('Invalid method. Available methods are Brisk, MSER, SIFT, ORB, SURF, FAST, Harris, MinEigen');
end

% Get a list of JPG files in the image directory
imageFiles = dir(fullfile(imageDir,'*.jpg'));  % Replace 'jpg' with the file type if it's different

% Define the number of images to process
numImages = numel(imageFiles);
border = 50;

% Initialize 'images' cell array
images = cell(1, numImages);
matchedPoints = cell(1,1);

% Initialize 'features' and 'validPoints' as structs
features = struct;
validPoints = struct;
MatchedPointsRaw = cell(numImages,numImages);

% Read the images into a cell array
for i = 1:numImages
    images{i} = imread(fullfile(imageDir, imageFiles(i).name));
end

% Define the feature detection function
detectFeatures = featureDetectionFunctions(method);
bestMatchForImage = struct('image', {}, 'bestMatch', '', 'numMatches', 0,'indexPairs',{},'matchedPoints',{});
I = images{1};
roi = [border, border, size(I, 2)- 2*border, size(I, 1)- 2*border];
for i = 1:numImages
    % Read the image
    img = images{i};

    % Convert the image to grayscale if it's not
    if ndims(img) == 3
        img = rgb2gray(img);
    end

    % Extract features
    points = detectFeatures(img,NumOctaves=8,ROI=roi);
    strongestPoints = points;

    % Create valid struct field name from image file name
    validStructFieldName = matlab.lang.makeValidName(imageFiles(i).name);

    % Extract features for the next steps
    [features.(validStructFieldName), validPoints.(validStructFieldName)] = extractFeatures(img, strongestPoints);
end

% Initialize
tforms = struct;  % Store geometric transformations
indexPairs = struct;
i=1;
usedimages =1;

% Compute the correspondences between all images
while true
    best_matches = 0;
    best_idx = 0;
    % Extract the features for the image i
    validStructFieldName1 = matlab.lang.makeValidName(imageFiles(i).name);
    features1 = features.(validStructFieldName1);

    % Initialize struct for the current image
    currentBestMatch = struct('image',i, 'bestMatch', 0, 'numMatches', 0,'indexPairs',{},'matchedPoints',{});

    for j = 1:numImages
        if i ~= j  % Ensure we're not matching an image with itself
            % Extract the features for the image j
            validStructFieldName2 = matlab.lang.makeValidName(imageFiles(j).name);
            features2 = features.(validStructFieldName2);

            % Match the features
            indexPairsRaw = matchFeatures(features1, features2,Unique=true);

            % Get the matching feature points
            matchedPoints1 = validPoints.(validStructFieldName1)(indexPairsRaw(:, 1), :);
            matchedPoints2 = validPoints.(validStructFieldName2)(indexPairsRaw(:, 2), :);
            matchedPoints = [matchedPoints2.Location matchedPoints1.Location];

            try
            [tform, inlierIdx] = estimateGeometricTransform(matchedPoints1, matchedPoints2, 'projective','Confidence', 0.70,'MaxNumTrials', 2000);  
            catch
                continue
            end

            % Compute the Euclidean distances between matched points
            numMatches=size(inlierIdx,1);
            distances = sqrt(sum((matchedPoints1.Location - matchedPoints2.Location).^2, 2));

            % Set a threshold for the Euclidean distance
            threshold = 90000;  % You can adjust this value based on your needs

            % Get the indices of matches where the Euclidean distance is below the threshold
            goodMatchIndices = distances < threshold;

            % Keep only the good matches
            indexPairsField = [validStructFieldName1 '_to_' validStructFieldName2];
            indexPairs.(indexPairsField) = indexPairsRaw(goodMatchIndices, :);
            % save index pairs and matched points
            if isempty(MatchedPointsRaw{j,i})
                MatchedPointsRaw{j,i} = matchedPoints;
                IndexPairsRaw{j,i} = indexPairsRaw;
            end
            % update the best matches 
            if numMatches ~=0
                if numMatches > best_matches
                    if ~any(usedimages==j)
                        best_idx = j;
                        best_matches = numMatches;
                        best_index_pairs = indexPairsRaw(goodMatchIndices, :);

                    else
                        continue
                    end
                end
            end
        end
    end
    if numel(usedimages) == numImages
        break
    end
    usedimages=[usedimages; best_idx];
    matchedPoints = MatchedPointsRaw{best_idx,i};
    best_image_match = struct('image', i, 'bestMatch', best_idx, 'numMatches', best_matches, 'indexPairs',{best_index_pairs}, 'matchedPoints',{matchedPoints});
    bestMatchForImage = [bestMatchForImage, best_image_match];
    i=best_idx;

end
end

