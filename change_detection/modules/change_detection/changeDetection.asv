% Module performing change detection on 3D data.

% Tunable parameters. 
% grid size - size of cubic voxels uniformly distributed throughout the
% bounding box which encapsulates investigated space
%
% k - size of neigborhood used for normal estimation. Lower it is, the normals
% are more distinctive and preserving details of surface, higher value
% means that details may be lost and surface approximation will be more
% generalized, potentially distorted by points that are not part of local
% surface (f.e. belong to different object) but still get in the neigborhood.
function changeDetection(route1,route2,gridSize,k)

close all;
clc;
addpath modules\change_detection\functions

wait = waitbar(0,"Wait...");

% Load the images of route 1.
imageDir = fullfile("data/images/"+route1);
imds1 = imageDatastore(imageDir);

% Load the images of route 2.
imageDir = fullfile("data/images/"+route2);
imds2 = imageDatastore(imageDir);

% Camera parameters.
load('data/calibrationSession.mat');
intrinsics = calibrationSession.CameraParameters.Intrinsics;

% Load the keyframes.
load("data/key_frames_indices/key_frames_indices_"+route1+".mat","keyFramesIdxVec");
keyFramesIdxVecFixed = keyFramesIdxVec;
load("data/key_frames_indices/key_frames_indices_"+route2+".mat","keyFramesIdxVec");
keyFramesIdxVecMoved = keyFramesIdxVec;

% Load the point clouds.
load("data/point_clouds/denoised/pointCloud_"+route1+".mat","ptCloud");
ptCloudFixed = ptCloud;
load("data/point_clouds/denoised/pointCloud_"+route2+".mat","ptCloud");
ptCloudMoved = ptCloud;

% Load the transformation for alignment of point clouds.
load("data/transformations/tformSVD_"+route2+"_to_"+route1+".mat","tformSVD");

% Load the worldpointset objects with 3D points and indices for their corresponding 2D
% features.
load("data/wpSet_structure/denoised/wpSet_structure_"+route1+".mat");
wpSetFixed = wpSet;
load("data/wpSet_structure/denoised/wpSet_structure_"+route2+".mat");
wpSetMoved = wpSet;

% Load the viewset objects with all the 2D points, their features and views that 
% were used in building 3D structures.
load("data/vSet_structure/vSet_structure_"+route1+".mat","vSet");
vSetFixed = vSet;
load("data/vSet_structure/vSet_structure_"+route2+".mat","vSet");
vSetMoved = vSet;

pointCloudShow(ptCloudFixed,"Global map",1,0);
pointCloudShow(ptCloudMoved,"Current location in global map",1,0);

% % Visualize unaligned point clouds.
% pointCloudShowPair(ptCloudFixed, ptCloudMoved, "Point clouds before alignment", 1,{'Fixed','Moved'}, 0);

transfWorldPointsSVD = transformPointsForward(tformSVD, ptCloudMoved.Location);
ptCloudMoved = pointCloud(transfWorldPointsSVD, Color=ptCloudMoved.Color);

% % Visualize transformed 3D points.
% pointCloudShowPair(ptCloudFixed, ptCloudMoved, 'Coarse registration SVD',1,{'Fixed','Moved'}, 0);

% Voxelize the point clouds with the resulution of gridSize and bin the data points.
[bins1, bins2, binLocations, ptCloudFixedCropped, idxOfInvestigatedPoints] = voxelize(gridSize, ptCloudFixed, ptCloudMoved);

% pointCloudShowPair(ptCloudFixedCropped, ptCloudMoved, 'Current location registered in cropped original point cloud',1,{'Fixed','Moved'},1);

% Pad the voxelized space.
[bins1Padded, bins2Padded, binLocationsPadded] = padding(bins1, bins2, binLocations);

% Exclude an empty space.
% Empty space log 0, occupied space log 1.
% Points of first point cloud.
A = ~cellfun("isempty", bins1Padded);
% Points of second point cloud.
B = ~cellfun("isempty", bins2Padded);

% Surface change detection.
% Investigate only non-empty voxels.
initialSurfaceChangeMask = A | B;
initialSurfaceChangeLocations = binLocationsPadded(initialSurfaceChangeMask);

% % Estimate point cloud normals.
% % k - size of neighborhood used for normal estimation.
% kValues = 3:100;
% nNormals = numel(kValues);
% evalVec = zeros(nNormals,3);
% for i=1:nNormals
%     k = kValues(i);
%     ptCloudNormals(ptCloudFixedCropped, k);
%     ptCloudNormals(ptCloudMoved, k);
%     
%     % Construct orientation histograms for estimated normals in every voxel.
%     nHistBins = [8 4]; % [azimuth [-180,180], elevation [-90 90]]
%     orientationHistograms1 = createOH(initialSurfaceChangeMask, initialSurfaceChangeLocations, ptCloudFixedCropped, ...
%         bins1Padded, gridSize, nHistBins);
%     orientationHistograms2 = createOH(initialSurfaceChangeMask, initialSurfaceChangeLocations, ptCloudMoved, ...
%         bins2Padded, gridSize, nHistBins);
%     
%     % Compare histograms and mark voxels with surface change flag.
%     minSimilarity = 0.5;
%     [surfaceChangeMask, similarityVec] = matchSurface(ptCloudFixedCropped, ptCloudMoved, initialSurfaceChangeMask, ...
%         initialSurfaceChangeLocations, orientationHistograms1, orientationHistograms2, gridSize, ...
%         bins1Padded, bins2Padded, nHistBins, minSimilarity);
%     similarityMean = mean(similarityVec);
%     nnz(surfaceChangeMask);
%     
%     % Refine the detection of surface change by taking into account voxel neighborhood.
%     finalSurfaceChangeMask = detectSurfaceChange(surfaceChangeMask, bins1Padded);
%     finalSurfaceChangeLocations = binLocationsPadded(finalSurfaceChangeMask == 1);
%     finalNoSurfaceChangeLocations = binLocationsPadded(finalSurfaceChangeMask == 2);
%     
%     % Select changed points and their indices from the investigated point clouds. 
%     [ptCloud1SurfaceChange, pointsIdx1SurfaceChange] = selectPoints(ptCloudFixedCropped, bins1Padded, finalSurfaceChangeMask);
%     [ptCloud2SurfaceChange, pointsIdx2SurfaceChange] = selectPoints(ptCloudMoved, bins2Padded, finalSurfaceChangeMask);
%                     
%     % Evaluation
%     [sensitivity,specificity,~,~,~] = evaluateChangeDetection(route1,route2,ptCloudFixedCropped, ...
%             pointsIdx1SurfaceChange,ptCloudMoved,pointsIdx2SurfaceChange);
%     
%     evalVec(i,:) = [k sensitivity specificity]; 
%     waitbar(i/nNormals,wait,string((i/nNormals*100) + " %"));
% end
% 
% % Choose the best value of neigborhood.
% [~, idx] = max(evalVec(:,2) + evalVec(:,3));
% optimalK = evalVec(idx,1);

% Estimate normals for both point clouds.
ptCloudNormals(ptCloudFixedCropped, k);
ptCloudNormals(ptCloudMoved, k);

% Construct orientation histograms for estimated normals in every voxel.
nHistBins = [8 4]; % [azimuth [-180,180], elevation [-90 90]]
orientationHistograms1 = createOH(initialSurfaceChangeMask, initialSurfaceChangeLocations, ptCloudFixedCropped, ...
    bins1Padded, gridSize, nHistBins);
orientationHistograms2 = createOH(initialSurfaceChangeMask, initialSurfaceChangeLocations, ptCloudMoved, ...
    bins2Padded, gridSize, nHistBins);

% Evaluation of change detection with different values of
% threshold (minimal similarity).
minSimilarity = 0.5;
minSimilarityValues = 0:0.1:1;
nSimilarity = numel(minSimilarityValues);
for i=1:nSimilarity
    [surfaceChangeMask, similarityVec] = matchSurface(ptCloudFixedCropped, ptCloudMoved, initialSurfaceChangeMask, ...
        initialSurfaceChangeLocations, orientationHistograms1, orientationHistograms2, gridSize, ...
        bins1Padded, bins2Padded, nHistBins, minSimilarityValues(i));

    % Refine the detection of surface change by taking into account voxel neighborhood.
    finalSurfaceChangeMask = detectSurfaceChange(surfaceChangeMask, bins1Padded);
    finalSurfaceChangeLocations = binLocationsPadded(finalSurfaceChangeMask == 1);
    finalNoSurfaceChangeLocations = binLocationsPadded(finalSurfaceChangeMask == 2);

    % Select changed points and their indices from the investigated point clouds. 
    [ptCloud1SurfaceChange, pointsIdx1SurfaceChange] = selectPoints(ptCloudFixedCropped, bins1Padded, finalSurfaceChangeMask);
    [ptCloud2SurfaceChange, pointsIdx2SurfaceChange] = selectPoints(ptCloudMoved, bins2Padded, finalSurfaceChangeMask);
    
    if minSimilarityValues(i) == minSimilarity
        % Visualize points that represent types of detected changes.
        pointCloudShowPair(ptCloudFixedCropped, ptCloud1SurfaceChange,'Changed points in global map (to remove)', 1,{'Źiadne zmeny','Zmeny'},1)
        pointCloudShowPair(ptCloudMoved, ptCloud2SurfaceChange,'Changed points from current location (add to global map)', 1,{'Źiadne zmeny','Zmeny'},1)
    
%         % Update the cropped point cloud with detected changes (remove/add points).
%         % Remove
%         ptCloudFixedCroppedUpdated = removePoints(ptCloudFixedCropped, pointsIdx1SurfaceChange');    
%         pointCloudShow(ptCloudFixedCroppedUpdated, "Cropped original point cloud with removed changes", 1,1);
        
%         % Add
%         ptCloudFixedCroppedUpdated = addPoints(ptCloudMoved, ptCloudFixedCroppedUpdated, pointsIdx2SurfaceChange');
%         pointCloudShow(ptCloudFixedCroppedUpdated, "Cropped original point cloud with added changes", 1,1);
        
%         % Update the global point cloud with detected changes (removal and addition of points).
%         ptCloudFixedUpdated = removePoints(ptCloudFixed, idxOfInvestigatedPoints);
%         ptCloudFixedUpdated = addPoints(ptCloudFixedCroppedUpdated, ptCloudFixedUpdated, 1:ptCloudFixedCroppedUpdated.Count);
%         pointCloudShow(ptCloudFixedUpdated, "Global map updated", 1,1);
    end

    % Evaluation
    [sensitivity,specificity,FPR,valuesGT, valuesPredict, accuracy] = evaluateChangeDetection(route1,route2,ptCloudFixedCropped, ...
        pointsIdx1SurfaceChange,ptCloudMoved,pointsIdx2SurfaceChange);

    TPRVec(i) = sensitivity;
    FPRVec(i) = FPR;

    if minSimilarityValues(i) == minSimilarity
        % Compute confusion matrix.
        C = confusionmat(valuesGT, valuesPredict);
        figure
        confusionchart(C,["Žiadna zmena", "Zmena"]);
        title("Matica zámen pre prah = "+string(minSimilarity)+", TPR = " ...
            +string(sensitivity)+", TNR = "+string(specificity) + " acc = " + ...
            string(accuracy) + " FPR = " + string(FPR))
    end
end

% ROC curve plot.
figure
plot(FPRVec,TPRVec)
AUC = trapz(FPRVec, TPRVec);
legend("AUC = " + string(AUC),"Location","southeast")
xlabel("FPR - False positive rate")
ylabel("TPR - True positive rate")

close(wait);
end