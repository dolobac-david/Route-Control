% Module for building 3D sparse model of scene based on previously selected keyframes and 
% their estimated camera locations. 

function [ptCloud,vSet,wpSet] = sfmStructure(route)

close all;
clc;
addpath modules\sfm_structure\functions

wait = waitbar(0,"Wait...");

% Load the images.
imageDir = fullfile("data/images/"+ route);
imds = imageDatastore(imageDir);

% Load the imageviewset and indices of keyframes.
load("data/vSet_odometry/vSet_odometry_"+ route,"vSet");
load("data/key_frames_indices/key_frames_indices_"+ route,"keyFramesIdxVec");

% Camera parameters.
load('data/calibrationSession.mat');
intrinsics = calibrationSession.CameraParameters.Intrinsics;

% Clear the old matches between the points.
for i=1:height(vSet.Connections)
    vSet = deleteConnection(vSet, i, i+1);
end

% Detect denser points in the keyframes.
nKeyFrames = numel(keyFramesIdxVec);
for i=1:nKeyFrames
    I = readimage(imds, keyFramesIdxVec(i));
    [features, points, I] = detectAndExtractFeatures(I, intrinsics);

    % Store the points in the view set.
    vSet = updateView(vSet, i, Points=points, Features=features);
end

% Match keypoints of every keyframe with next n keyframes.
nNextFrames = 2;
for i=1:nKeyFrames-1
    prevI = readimage(imds, keyFramesIdxVec(i));
    prevI = undistortImage(prevI, intrinsics);
    prevFeatures = vSet.Views.Features{i};
    prevPoints = vSet.Views.Points{i};

    startFrame = i+1;
    if nKeyFrames - startFrame >= nNextFrames
        endFrame = startFrame + nNextFrames-1;
    else
        endFrame = nKeyFrames;
    end

    for j=startFrame:endFrame
        currI = readimage(imds, keyFramesIdxVec(j));
        currI = undistortImage(currI, intrinsics);
        currFeatures = vSet.Views.Features{j};
        currPoints = vSet.Views.Points{j};

        indexPairs = matchFeatures(prevFeatures, currFeatures, MaxRatio=1, Unique=true, ...
            MatchThreshold = 100);
   
        if height(indexPairs) == 0
            continue;
        end

        % Select matched points.
        matchedPoints1 = prevPoints(indexPairs(:, 1));
        matchedPoints2 = currPoints(indexPairs(:, 2));
    
% %          Visualize epipolar inliers.
%         figure(2)
%         showMatchedFeatures(prevI, currI, prevPoints(indexPairs(:,1)), currPoints(indexPairs(:,2)));
%         title('Initial correspondences, frames ' + string(keyFramesIdxVec(i)) + ' and ' + string(keyFramesIdxVec(j)));
    
        % Exclude epipolar outliers using RANSAC.
        [inlierIdx, essentialMatrixError] = estimateEpipolarInliers(matchedPoints1, matchedPoints2, ...
            intrinsics);

        if essentialMatrixError
            continue;
        end

        indexPairs = indexPairs(inlierIdx,:);
    
%         disp("Number of epipolar inliers, frame " + string(keyFramesIdxVec(i)) + " and " + ...
%             string(keyFramesIdxVec(j)) + newline + newline + string(nnz(inlierIdx)));
    
%         % Visualize epipolar inliers.
%         figure(3)
%         showMatchedFeatures(prevI, currI, prevPoints(indexPairs(:,1)), currPoints(indexPairs(:,2)));
%         title('Epipolar inliers, frames ' + string(keyFramesIdxVec(i)) + ' and ' + string(keyFramesIdxVec(j)));
    
        % Exclude remaining outliers with points distance between two views.
        pointsDistance = vecnorm((currPoints(indexPairs(:,2)).Location - ...
            prevPoints(indexPairs(:,1)).Location),2,2);
        pointsDistMean = mean(pointsDistance);
        idxDistPoints = indexPairs(~(pointsDistance > 2.5*pointsDistMean),:);
    
        if height(idxDistPoints) == 0
            continue;
        end

%         % Visualize correspondences
%         figure(4)
%         showMatchedFeatures(prevI, currI, prevPoints(idxDistPoints(:,1)), ...
%             currPoints(idxDistPoints(:,2)));
%         title('Epipolar inliers filtered with points distance, frames ' + ...
%             string(keyFramesIdxVec(i)) + ' and ' + string(keyFramesIdxVec(j)));
    
        % Check the parallax between the two views and exclude points with
        % the insufficient parallax.
        [parallaxDegrees, indexPairsParallax] = checkParallax(intrinsics, ...
            vSet.Views.AbsolutePose(i), vSet.Views.AbsolutePose(j), ...
            prevPoints(idxDistPoints(:,1)), currPoints(idxDistPoints(:,2)), idxDistPoints);
    
        if height(indexPairsParallax) == 0
            continue;
        end

%         % Visualize correspondences.
%         figure(5)
%         showMatchedFeatures(prevI, currI, prevPoints(indexPairsParallax(:,1)), ...
%             currPoints(indexPairsParallax(:,2)));
%         title('Epipolar inliers filtered with points distance and parallax, frames ' + ...
%             string(keyFramesIdxVec(i)) + ' and ' + string(keyFramesIdxVec(j)));
    
        % Store the matches suitable for triangulation.
        vSet = addConnection(vSet, i, j, Matches=indexPairsParallax);
    end
    waitbar(i/nKeyFrames,wait,"Building 3D model:"+ newline +string((i/nKeyFrames*100) + " %"));
end

% Find point tracks across all views.
tracks = findTracks(vSet);

% Get the table containing camera poses for all views.
camPoses = poses(vSet);

% Triangulate locations for the 3-D world points.
[xyzPoints, reprojErrors, inFrontIdx] = triangulateMultiview(tracks, camPoses, intrinsics);

% Exclude points with high reprojection error and points located behind the
% camera.
idx = reprojErrors < 1 & inFrontIdx;

% Refine the 3-D world points.
xyzPointsRefined = bundleAdjustment(xyzPoints(idx,:), tracks(idx), camPoses, intrinsics, ...
        PointsUndistorted=true, AbsoluteTolerance=1e-12, RelativeTolerance=1e-12, ...
        MaxIterations=10000, FixedViewID=1, Verbose = true);

% Color the points.
colorVec = pointCloudColor(tracks, keyFramesIdxVec, imds, intrinsics);
ptCloud = pointCloud(xyzPointsRefined, "Color", colorVec(idx,:));

% Visualize the point cloud and estimated camera poses.
locations = vertcat(camPoses.AbsolutePose.Translation);

figure(6)
pcshow(ptCloud, VerticalAxis='y', VerticalAxisDir='down', AxesVisibility="on", ...
    MarkerSize=45,Projection="perspective");
hold on
plotCamera(camPoses, Size=1, Color="red");
plot3(locations(:,1),locations(:,2),locations(:,3),"g-","LineWidth",1,"Marker","o", ...
    "MarkerEdgeColor","m");
grid on
xlabel('X') 
ylabel('Y') 
zlabel('Z') 
hold off

% Store the 3D points and indices to their corresponding 2D features into
% the worldpointset object.
wpSet = worldpointset;
wpSet = addWorldPoints(wpSet, xyzPointsRefined);

tracksRefined = tracks(idx);
for i=1:width(tracksRefined)
    for j=1:width(tracksRefined(i).ViewIds)
        wpSet = addCorrespondences(wpSet,tracksRefined(i).ViewIds(j),i,tracksRefined(i).FeatureIndices(j));
    end
end
close(wait);
end