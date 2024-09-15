%% Detect changed voxels by comparing their surface similarity.
function [changeMask,similarityVec] = matchSurface(ptCloudFixed, ptCloudMoved, initialChangeMask, changeLocations, ...
    orientationHistograms1, orientationHistograms2, gridSize, bins1, bins2, nHistBins, minSimilarity)

voxelIdx = find(initialChangeMask);
pointsIdx1 = bins1(voxelIdx);
pointsIdx2 = bins2(voxelIdx);

[rows, cols, pages] = size(initialChangeMask);
changeMask = zeros(rows, cols, pages);
similarityVec = zeros(1,numel(orientationHistograms1));
% Compare similarity of corresponding histograms.
% pointCloudShowPair(ptCloudFixed, ptCloudMoved, 'Normals in voxels',0);
for i=1:numel(orientationHistograms1)
%     drawVoxels(changeLocations{i}, gridSize, lines(1));
%     quiver3(ptCloudFixed.Location(pointsIdx1{i},1), ptCloudFixed.Location(pointsIdx1{i},2), ...
%         ptCloudFixed.Location(pointsIdx1{i},3), ptCloudFixed.Normal(pointsIdx1{i},1), ...
%         ptCloudFixed.Normal(pointsIdx1{i},2), ptCloudFixed.Normal(pointsIdx1{i},3),Color="magenta");
%     quiver3(ptCloudMoved.Location(pointsIdx2{i},1), ptCloudMoved.Location(pointsIdx2{i},2), ...
%         ptCloudMoved.Location(pointsIdx2{i},3), ptCloudMoved.Normal(pointsIdx2{i},1), ...
%         ptCloudMoved.Normal(pointsIdx2{i},2), ptCloudMoved.Normal(pointsIdx2{i},3),Color="green");

    % Compare histograms of current voxel only if there are points from point cloud 1 and 2 present. 
    if numel(pointsIdx1{i}) > 0 && numel(pointsIdx2{i}) > 0
        % Intersection of normalized histograms.
        normalizedHist1 = orientationHistograms1{i} / sum(orientationHistograms1{i},"all");
        normalizedHist2 = orientationHistograms2{i} / sum(orientationHistograms2{i},"all");
        
        histIntersection = min(normalizedHist1, normalizedHist2);
        similarity = sum(histIntersection, "all");
    % If theres only points 1 or 2 in current voxel, set similarity of surface to 0.  
    else
        similarity = 0;
    end
    similarityVec(i) = similarity;
    
    % Green, flag 1 - change
    if similarity < minSimilarity || minSimilarity == 1
        changeMask(voxelIdx(i)) = 1;
%         drawVoxels(changeLocations{i}, gridSize, "green");

    % Red, flag 2 - no change.
    else
        changeMask(voxelIdx(i)) = 2;
%         drawVoxels(changeLocations{i}, gridSize, "red");
    end

%     figure
%     histogram2('XBinEdges',-180:360/nHistBins(1):180, 'YBinEdges', ...
%         -90:180/nHistBins(2):90,'BinCounts', orientationHistograms1{i},DisplayStyle='tile',FaceColor="flat");
%     colorbar
%     xlabel('azimuth') 
%     ylabel('elevation') 
%     zlabel('count') 
% 
%     figure
%     histogram2('XBinEdges',-180:360/nHistBins(1):180, 'YBinEdges', ...
%         -90:180/nHistBins(2):90,'BinCounts', orientationHistograms2{i},DisplayStyle='tile',FaceColor="flat");
%     colorbar
%     xlabel('azimuth') 
%     ylabel('elevation') 
%     zlabel('count') 
% 
%     close;
%     close;
end
end