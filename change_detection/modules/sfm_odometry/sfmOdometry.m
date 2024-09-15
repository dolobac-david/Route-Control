% Module for estimating camera poses of chosen keyframes from input image sequence. 

% Tunable parameters. 
% SURF threshold - lower it is, more points are detected, vice versa.
%
% min paralax (degrees) - minimal angle between two consecutive frames,
% lower angle means more keyframes taken and more time to process, vice versa.

function [vSet,keyFramesIdxVec] = sfmOdometry(route, SURFthreshold, minParallax)
    
close all;
clc;
addpath modules/sfm_odometry/functions

% Load input images of scene.
imageDir = fullfile("data/images/" + route);
imds = imageDatastore(imageDir);

% Camera parameters.
load('data/calibrationSession.mat');
intrinsics = calibrationSession.CameraParameters.Intrinsics;

allPoints = zeros(numel(imds.Files),1);

% Determine number of uniformly distributed points - numPoints, selected from the
% detected keypoints.
wait = waitbar(0,"Determining number of feature points to be extracted");
for i = 1:numel(imds.Files)
    % Detect, extract and match features
    currI = readimage(imds, i);
    [~, ~, ~, ~, allPoints] = detectAndExtractFeatures(currI, intrinsics, allPoints,i, 500, SURFthreshold);
    
    waitbar(i/numel(imds.Files),wait,"Determining number of feature points to be extracted:" +newline +string((i/numel(imds.Files)*100) + " %"));
end
close(wait)
meanAllPoints = mean(allPoints);
numPoints = round(meanAllPoints/3);

prevI = readimage(imds, 1);
[prevFeatures, prevPoints, prevI, threshold, allPoints] = detectAndExtractFeatures(prevI, intrinsics, allPoints, 1, numPoints, SURFthreshold);

% Create an empty imageviewset object to manage the data associated with each
% view.
vSet = imageviewset;

% Add the first view. Place the camera associated with the first view
% in the origin, oriented along the Z-axis.
lastKeyFrameId = 1;
keyFramesIdxVec(lastKeyFrameId) = 1;
vSet = addView(vSet, lastKeyFrameId, rigidtform3d, Points=prevPoints, ...
    Features=prevFeatures);

% Estimate the pose of second view. 
wait = waitbar(0,"Estimating pose of second view");
for i = 2:numel(imds.Files)
    % Detect, extract and match features
    currI = readimage(imds, i);
    [currFeatures, currPoints, currI, threshold, allPoints] = detectAndExtractFeatures(currI, intrinsics, allPoints,i, numPoints, SURFthreshold);
    indexPairs = matchFeatures(prevFeatures, currFeatures, "Unique", true, "MaxRatio", 0.9);

%     % Visualize correspondences.
%     figure(2)
%     showMatchedFeatures(prevI, currI, prevPoints(indexPairs(:,1)), ...
%         currPoints(indexPairs(:,2)));
%     title('Frames ' + string(keyFramesIdxVec(1)) + ' and ' + string(i));

    % Estimate the pose of the current view relative to the previous view.
    [relPose, inlierIdx, essentialMatrixError,E] = estimateRelativePose(prevPoints(indexPairs(:,1)), ...
        currPoints(indexPairs(:,2)), intrinsics);

    % Skip to next frame, if essential matrix is not valid.
    if essentialMatrixError
        continue;
    end

    % Exclude epipolar outliers.
    indexPairs = indexPairs(inlierIdx, :);

%     % Visualize correspondences
%     figure(3)
%     showMatchedFeatures(prevI, currI, prevPoints(indexPairs(:,1)), ...
%         currPoints(indexPairs(:,2)));
%     hold on;
%     title('Epipolar inliers, frames ' + string(keyFramesIdxVec(1)) + ' and ' + string(i));

    % Check the pixel distance between the points from two views. 
    [enoughDistance, pointsDistance, minDistance] = calcPointsDistance(prevPoints(indexPairs(:,1)), ...
        currPoints(indexPairs(:,2)));

    if ~enoughDistance
        continue;
    end

    % Check the parallax between the two views.
    [parallaxDegrees, isParallaxValid, reprErr, inFront] = checkParallax(intrinsics, ...
        vSet.Views.AbsolutePose(lastKeyFrameId), relPose, ...
        prevPoints(indexPairs(:,1)), currPoints(indexPairs(:,2)), minParallax);

    if ~isParallaxValid || ~all(inFront & (reprErr < 1))
        continue;
    end

%     title('Map initialized with frames ' + string(keyFramesIdxVec(1)) + ' and ' + string(i));
%     hold off;

%     disp("Reprojection error of points > 1 (parallax)")
%     nnz(~(reprErr < 1))
%     disp("Points behind camera (parallax)")
%     disp(~(nnz(inFront)))

    lastKeyFrameId = lastKeyFrameId + 1;
    keyFramesIdxVec(lastKeyFrameId) = i;

    % Add the current view to the view set.
    vSet = addView(vSet, lastKeyFrameId, relPose, Points=currPoints, ...
        Features=currFeatures);
    
    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, lastKeyFrameId-1, lastKeyFrameId, Matches=indexPairs);

    prevFeatures = currFeatures;
    prevPoints = currPoints;
    break;
end
close(wait)

% Estimating poses of the rest of the views using Perspective-3-Point algorithm.
wait = waitbar(0,"Selecting keyframes and estimating their poses");
for i=keyFramesIdxVec(lastKeyFrameId)+1:numel(imds.Files)
    currI = readimage(imds, i);
    [currFeatures, currPoints, currI, threshold, allPoints] = detectAndExtractFeatures(currI, intrinsics, allPoints,i, numPoints, SURFthreshold);

    % Match points between the previous and the current image.
    indexPairs = matchFeatures(prevFeatures, currFeatures, "Unique", true, "MaxRatio", 0.9);

% %   Visualize correspondences.
%     figure(4)
%     showMatchedFeatures(prevI, currI, prevPoints(indexPairs(:,1)), ...
%         currPoints(indexPairs(:,2)));
%     title('Frames ' + string(keyFramesIdxVec(end)) + ' and ' + string(i));

    % Eliminate outliers from feature matches with RANSAC.
    [~, inlierIdx, essentialMatrixError,E] = estimateRelativePose(prevPoints(indexPairs(:,1)), ...
        currPoints(indexPairs(:,2)), intrinsics);

    % Skip to next frame, if essential matrix is not valid
    if essentialMatrixError
        continue;
    end

    indexPairs = indexPairs(inlierIdx, :);

%     % Visualize correspondences
%     figure(5)
%     showMatchedFeatures(prevI, currI, prevPoints(indexPairs(:,1)), ...
%         currPoints(indexPairs(:,2)));
%     title('Epipolar inliers, frames ' + string(keyFramesIdxVec(end)) + ' and ' + string(i));

    % Exclude remaining outliers with pixel points distance between two views.
    [enoughDistance, pointsDistance, minDistance] = calcPointsDistance(prevPoints(indexPairs(:,1)), ...
        currPoints(indexPairs(:,2)));
    pointsDistMean = mean(pointsDistance);
    idxDistPoints = indexPairs(~(pointsDistance > 2.5*pointsDistMean),:);

%     % Visualize correspondences
%     figure(6)
%     showMatchedFeatures(prevI, currI, prevPoints(idxDistPoints(:,1)), ...
%         currPoints(idxDistPoints(:,2)));
%     hold on;
%     title('Epipolar inliers filtered with points distance, frames ' + string(keyFramesIdxVec(end)) + ' and ' + string(i));

    if ~enoughDistance
        continue;
    end

    % Establish 3D to 2D correspondences between the previous pair of
    % keyframes and current frame.
    [worldPoints, imagePoints, reprojErrors3Dto2D, validIdx3Dto2D, idx1, idx2, idxTriplet] = find3Dto2DCorrespondences(vSet, intrinsics, idxDistPoints, currPoints);
%     disp('Number of world points for PnP, frame ' + string(keyFramesIdxVec(lastKeyFrameId)) + ' and ' + string(i));
%     size(worldPoints,1)
%     disp("Reprojection error of world points for PnP < 1")
%     nnz(reprojErrorsPnP < 1)
%     disp("Valid points for PnP")
%     disp(nnz(validIdxPnP))

    % Estimate the absolute camera pose for the current view.
    idx3Dto2D = reprojErrors3Dto2D < 1 & validIdx3Dto2D;

    % Check if the 2D-3D correspondences aren't empty vector (results in
    % estworldpose runtime error)
    if nnz(idx3Dto2D) < 1
        disp("No 3D-2D correspondences find for P3P, frames " + ...
            string(keyFramesIdxVec(end)) + " and " + string(i));
        continue;
    end

    [absPose, inliersPNP, status] = estworldpose(imagePoints(idx3Dto2D,:), worldPoints(idx3Dto2D,:), ...
        intrinsics);
%     disp('Number of inliers PNP');
%     nnz(inliersPNP)

    if status ~= 0
        continue;
    end

    % Check the parallax between the two consecutive views.
    [parallaxDegrees, isParallaxValid, reprErr, inFront] = checkParallax(intrinsics, ...
        vSet.Views.AbsolutePose(lastKeyFrameId), absPose, ...
        prevPoints(idxDistPoints(:,1)), currPoints(idxDistPoints(:,2)), minParallax);

    if ~isParallaxValid
        continue;
    end

%     disp("Reprojection error of points > 1 (parallax) ")
%     nnz(~(reprErr < 1))
%     disp("Points behind camera (parallax)")
%     disp(~(nnz(inFront)))

%     % Visualize 3D-2D point correspondences accross three consecutive views that were 
%     % used in P3P method. 
%     points1 = vSet.Views.Points{end-1}(idx1(idx3Dto2D(inliersPNP)));
%     points2 = vSet.Views.Points{end}(idx2(idx3Dto2D(inliersPNP)));
%     pointsTriplets = currPoints(idxTriplet(idx3Dto2D(inliersPNP)));

%     figure(7)
%     showMatchedFeatures(undistortImage(readimage(imds, keyFramesIdxVec(end-1)), intrinsics), ...
%         prevI, points1, points2);
%     title('PnP inliers, frames ' + string(keyFramesIdxVec(end-1)) + ' and ' + string(keyFramesIdxVec(end)) + ' number of inliers ' + nnz(inliersPNP));

%     figure(8)
%     showMatchedFeatures(prevI, currI, points2, pointsTriplets);
%     title('PnP inliers, frames ' + string(keyFramesIdxVec(end)) + ' and ' + string(i)+ ' number of inliers ' + nnz(inliersPNP));

    lastKeyFrameId = lastKeyFrameId + 1;
    keyFramesIdxVec(lastKeyFrameId) = i;

    % Add the current view to the view set.
    vSet = addView(vSet, lastKeyFrameId, absPose, Points=currPoints, Features=currFeatures);

    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, lastKeyFrameId-1, lastKeyFrameId, Matches=idxDistPoints);

    % Find point tracks spanning multiple views.
    tracks = findTracks(vSet);

    % Get camera poses for all views.
    camPoses = poses(vSet);

    % Triangulate initial locations for the 3-D world points.
    [xyzPoints, reprojErrors, inFrontIdx] = triangulateMultiview(tracks, camPoses, intrinsics);
%     disp("Number of xyz points")
%     size(xyzPoints,1)
%     disp("Reprojection error xyz points < 1 (triangulation)")
%     nnz(reprojErrors < 1)
%     disp("Valid xyz points (triangulation)")
%     disp(nnz(validIdx))

    % Exclude points with high reprojection errors and points that project
    % behind the camera.
    idx = reprojErrors < 1 & inFrontIdx;
    [~, camPoses] = bundleAdjustment(xyzPoints(idx,:), tracks(idx), camPoses, intrinsics, ...
        PointsUndistorted=true, AbsoluteTolerance=1e-12, RelativeTolerance=1e-12, ...
        MaxIterations=200, FixedViewID=1, Verbose = false);

    % Update view set.
    vSet = updateView(vSet, camPoses);

    prevI = currI;
    prevFeatures = currFeatures;
    prevPoints   = currPoints;

    % Visualize estimated camera poses
    locations = vertcat(poses(vSet).AbsolutePose.Translation);

    figure(9)
    pcshow(locations, VerticalAxis='y', VerticalAxisDir='down', AxesVisibility="on", ...
        MarkerSize=45,Projection="perspective");
    hold on
    plotCamera(poses(vSet), Size=1, Color="red");
    plot3(locations(:,1),locations(:,2),locations(:,3),"g-","LineWidth",1,"Marker","o", ...
        "MarkerEdgeColor","m");
    grid on
    xlabel('X') 
    ylabel('Y') 
    zlabel('Z') 
    hold off

    waitbar(i/numel(imds.Files),wait,"Selecting keyframes and estimating their poses:"+newline+string((i/numel(imds.Files)*100) + " %"));
end
close(wait);

% errorSum = trajectoryError(vSet);
% xDiff = trajectoryDifferenceX(vSet);
% meanAllPoints = mean(allPoints);
% minAllPoints = min(allPoints);
% maxAllPoints = max(allPoints);
% keyFramesPoints = allPoints(keyFramesIdxVec);
% meanKeyFramesPoints = mean(allPoints(keyFramesIdxVec));
% minKeyFramesPoints = min(allPoints(keyFramesIdxVec));
% maxKeyFramesPoints = max(allPoints(keyFramesIdxVec));
end