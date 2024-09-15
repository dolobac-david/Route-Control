% Module for registering two point clouds. 

function tformSVD = pcRegistration(route1,route2)

close all;
clc;
addpath  modules\pc_registration\functions

wait = waitbar(0,"Registering two point clouds");

% Load the images of route 1.
imageDir = fullfile("data/images/"+route1);
imds1 = imageDatastore(imageDir);

% Load the images of route 2.
imageDir = fullfile("data/images/"+route2);
imds2 = imageDatastore(imageDir);

% Camera parameters.
load('data/calibrationSession.mat');
intrinsics = calibrationSession.CameraParameters.Intrinsics;

% Load the indices of keyframes of both routes.
load("data/key_frames_indices/key_frames_indices_"+route1+".mat","keyFramesIdxVec");
keyFramesIdxVecFixed = keyFramesIdxVec;
load("data/key_frames_indices/key_frames_indices_"+route2+".mat","keyFramesIdxVec");
keyFramesIdxVecMoved = keyFramesIdxVec;

% Load the point clouds.
load("data/point_clouds/denoised/pointCloud_"+route1+".mat","ptCloud");
ptCloudFixed = ptCloud;
load("data/point_clouds/denoised/pointCloud_"+route2+".mat","ptCloud");
ptCloudMoved = ptCloud;

% Load the viewset objects with all the 2D points, their features and views that 
% were used in building 3D structures.
load("data/vSet_structure/vSet_structure_"+route1+".mat","vSet");
vSetFixed = vSet;
load("data/vSet_structure/vSet_structure_"+route2+".mat","vSet");
vSetMoved = vSet;

% Load the worldpointset objects with 3D points and their indices for their corresponding 2D
% features.
load("data/wpSet_structure/denoised/wpSet_structure_"+route1+".mat","wpSet");
wpSetFixed = wpSet;
load("data/wpSet_structure/denoised/wpSet_structure_"+route2+".mat","wpSet");
wpSetMoved = wpSet;

% Iterate through keyframes of route 1 and select the 2D features and
% 2D points from i-th keyframe that correspond to 3D worldpoints of point cloud 1.
worldPointPairs = [];
for i=1:width(wpSetFixed.ViewIds)
    [features1, points1, I1, worldPointIndices1] = select2DFeaturesInView(wpSetFixed, i, ...
        vSetFixed, imds1, keyFramesIdxVecFixed, intrinsics);

    % Iterate through keyframes of route 2 and select the 2D features and
    % 2D points from j-th keyframe that correspond to 3D worldpoints of point cloud 2.
    for j=1:width(wpSetMoved.ViewIds)
%         % Show points of i-th frame from route 1.
%         figure;
%         imshow(im2gray(I1));
%         hold on;
%         plot(points1);
%         title('Points, frame ' + string(keyFramesIdxVecFixed(i)) + ", point cloud 1");
%         hold off;

        [features2, points2, I2, worldPointIndices2] = select2DFeaturesInView(wpSetMoved, j, vSetMoved, ...
            imds2, keyFramesIdxVecMoved, intrinsics);

%         % Show points from j-th frame from route 2.
%         figure;
%         imshow(im2gray(I2));
%         hold on;
%         plot(points2);
%         title('Points, frame ' + string(keyFramesIdxVecMoved(j)) + ", point cloud 2");
%         hold off;

        % Match the 2D features of the i-th view from route 1 with the 
        % features of j-th view from route 2.
        indexPairs = matchFeatures(features1, features2, "Unique", true, "MaxRatio", 0.6, ...
        "MatchThreshold",1);

        % Select putatively matched 3D points from point clouds.
        worldPoints1 = wpSetFixed.WorldPoints(worldPointIndices1(indexPairs(:,1)),:);
        worldPoints2 = wpSetMoved.WorldPoints(worldPointIndices2(indexPairs(:,2)),:);

        % Select putatively matched 2D points from point clouds.
        matchedPoints1 = points1(indexPairs(:,1));
        matchedPoints2 = points2(indexPairs(:,2));

        % Compute translation, rotation and scale between putatively matched 3D points and 
        % exclude outlier correspondences wrt distance of transformed points2 to points1.
        distMean = 1e6;
        enoughPoints = true;
        while distMean > 1
            % Make sure there are at least 3 point pairs.
            if height(worldPoints1) < 3 || height(worldPoints2) < 3
                enoughPoints = false;
                disp("Not enough points");
                break;
            end

%             % Visualize 2D feature matches.
%             figure;
%             showMatchedFeatures(I1, I2, matchedPoints1, matchedPoints2);
%             title('Putatively matched points, frames ' + string(keyFramesIdxVecFixed(i)) + ' and ' + string(keyFramesIdxVecMoved(j)));
%             legend("Matched points 1","Matched points 2");
%     
%             % Visualize 2D feature matches.
%             figure;
%             showMatchedFeatures(im2gray(I1), im2gray(I2), matchedPoints1, matchedPoints2, "montage");
%             title('Putatively matched points, frames ' + string(keyFramesIdxVecFixed(i)) + ' and ' + string(keyFramesIdxVecMoved(j)));
%             legend("Matched points 1","Matched points 2");
% 
%             % Visualize 3D points matches.
%             figure;
%             pcshowpair(pointCloud(worldPoints1), pointCloud(worldPoints2), VerticalAxis='y', VerticalAxisDir='down', ...
%                 AxesVisibility="on", MarkerSize=45, Projection="perspective");
%             hold on;
%             for k=1:height(worldPoints1)
%                 plot3([worldPoints1(k,1);worldPoints2(k,1)], ...
%                     [worldPoints1(k,2);worldPoints2(k,2)],[worldPoints1(k,3);worldPoints2(k,3)],"Color","red");
%             end
%             grid on
%             xlabel('X') 
%             ylabel('Y') 
%             zlabel('Z') 
%             title('Points before alignment')
%             legend({'80 fixed','81 moving'},'TextColor','w')
%             legend('Location','southoutside')
%             hold off;

            % Compute transformation.
            [R,t,s,tformSVD,invalidR] = tformAbsoluteOrient(worldPoints2,worldPoints1);
    
            % Check the validity of rotation matrix.
            if invalidR
                disp("Invalid rotation matrix");
                break;
            end

            % Transform the points.
            inliersPtCloudSVD2 = transformPointsForward(tformSVD,worldPoints2);
            inliersPtCloudSVD1 = worldPoints1;

%             % Visualize transformed 3D points.
%             figure;
%             pcshowpair(pointCloud(inliersPtCloudSVD1), pointCloud(inliersPtCloudSVD2), VerticalAxis='y', VerticalAxisDir='down', ...
%                 AxesVisibility="on", MarkerSize=45, Projection="perspective");
%             hold on;
%             for k=1:height(inliersPtCloudSVD1)
%                 plot3([inliersPtCloudSVD1(k,1);inliersPtCloudSVD2(k,1)], ...
%                     [inliersPtCloudSVD1(k,2);inliersPtCloudSVD2(k,2)],[inliersPtCloudSVD1(k,3);inliersPtCloudSVD2(k,3)],"Color","red");
%             end
%             grid on
%             xlabel('X') 
%             ylabel('Y') 
%             zlabel('Z') 
%             title('Points aligned')
%             legend({'80 fixed','81 moving'},'TextColor','w')
%             legend('Location','southoutside')
%             hold off;

            % Compute distances between fixed points1 and transformed
            % points2.
            distancesTransfSVD = vecnorm((inliersPtCloudSVD1 - inliersPtCloudSVD2),2,2);
            distMean = mean(distancesTransfSVD);

            % Select inliers for next transformation estimate.
            if distMean > 1
                distIdx = distancesTransfSVD < distMean;

                indexPairs = indexPairs(distIdx,:);
                worldPoints1 = worldPoints1(distIdx,:);
                worldPoints2 = worldPoints2(distIdx,:);
    
                matchedPoints1 = matchedPoints1(distIdx);
                matchedPoints2 = matchedPoints2(distIdx);
            end
        end

        % Store the correct correspondences.
        if enoughPoints && ~invalidR
            worldPointPairs = [worldPointPairs;
                [worldPointIndices1(indexPairs(:,1)) worldPointIndices2(indexPairs(:,2))] distancesTransfSVD];
        end
        close all;
    end
end

clearvars -except wait worldPointPairs wpSetFixed wpSetMoved ptCloudFixed ptCloudMoved
%             close all;
clc;

wait = waitbar(0,"Registering two point clouds");
% Find multiple matches of same points from point cloud 1 (first column of correspondence 
% matrix worldPointPairs) and leave only the strongest match - the match with minimal distance.
[~, idxUnique] = unique(worldPointPairs(:,1), 'stable');
uniqueVals = worldPointPairs(idxUnique,:);

for i=1:height(uniqueVals)
    idx = worldPointPairs(:,1) == uniqueVals(i,1);
    duplicates = worldPointPairs(idx,:);
    [~, idxMin] = min(duplicates(:,3));
    uniqueVals(i,:) = duplicates(idxMin,:);
end

% Do the same for multiple occurences of points2 in second column.
[~, idxUnique2] = unique(uniqueVals(:,2), 'stable');
uniqueVals2 = uniqueVals(idxUnique2,:);

for i=1:height(uniqueVals2)
    idx2 = uniqueVals(:,2) == uniqueVals2(i,2);
    duplicates = uniqueVals(idx2,:);
    [~, idxMin] = min(duplicates(:,3));
    uniqueVals2(i,:) = duplicates(idxMin,:);
end

% Select matched 3D points from two point clouds.
worldPoints1 = wpSetFixed.WorldPoints(uniqueVals2(:,1),:);
worldPoints2 = wpSetMoved.WorldPoints(uniqueVals2(:,2),:);

% Estimate final transformation between two point clouds, exclude further correspondences 
% that do not satisfy minimal distance condition.
done = false;
while true
    [R,t,s,tformSVD,invalidR] = tformAbsoluteOrient(worldPoints2,worldPoints1);
    
    if invalidR
        error("Invalid rotation matrix");
    end
    
    inliersPtCloudSVD2 = transformPointsForward(tformSVD,worldPoints2);
    inliersPtCloudSVD1 = worldPoints1;

    % Visualize transformed 3D points.
    figure;
    pcshowpair(pointCloud(inliersPtCloudSVD1), pointCloud(inliersPtCloudSVD2), VerticalAxis='y', VerticalAxisDir='down', ...
        AxesVisibility="on", MarkerSize=45, Projection="perspective");
    hold on;
    for k=1:height(inliersPtCloudSVD1)
        plot3([inliersPtCloudSVD1(k,1);inliersPtCloudSVD2(k,1)], ...
            [inliersPtCloudSVD1(k,2);inliersPtCloudSVD2(k,2)],[inliersPtCloudSVD1(k,3);inliersPtCloudSVD2(k,3)],"Color","red");
    end
    grid on
    xlabel('X') 
    ylabel('Y') 
    zlabel('Z') 
    title('Points aligned')
    legend({'Fixed','Moved'},'TextColor','w')
    legend('Location','southoutside')
    hold off;
    
    distancesTransfSVD = vecnorm((inliersPtCloudSVD1 - inliersPtCloudSVD2),2,2);
    distMean = mean(distancesTransfSVD);

    if done
        break;
    end

    if distMean > 1
        distIdx = distancesTransfSVD < distMean;
    else
        distIdx = distancesTransfSVD < 1;
        done = true;
    end

    worldPoints1 = worldPoints1(distIdx,:);
    worldPoints2 = worldPoints2(distIdx,:);
end

transfWorldPointsSVD = transformPointsForward(tformSVD, ptCloudMoved.Location);
ptCloudMoved = pointCloud(transfWorldPointsSVD, Color=ptCloudMoved.Color);

% Visualize transformed 3D points.
figure;
pcshowpair(pointCloud(ptCloudFixed.Location), pointCloud(ptCloudMoved.Location), VerticalAxis='y', VerticalAxisDir='down', ...
    AxesVisibility="on", MarkerSize=45, Projection="perspective");
hold on;
grid on
xlabel('X') 
ylabel('Y') 
zlabel('Z') 
title("Coarse registration of two point clouds")
legend({'Fixed','Moved'},'TextColor','w')
legend('Location','southoutside')

close(wait)
end