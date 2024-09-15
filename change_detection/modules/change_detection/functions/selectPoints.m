%% Select points from the point cloud given voxels/bins and mask.
function [ptCloudOut, pointsIdxVec] = selectPoints(ptCloud, bins, mask)

pointsIdx = bins(mask == 1);
pointsIdxVec = [];
for i=1:height(pointsIdx)

    if isempty(pointsIdx{i})
        continue;
    end
    
    pointsIdxVec = [pointsIdxVec pointsIdx{i}'];
end

ptCloudOut = select(ptCloud,pointsIdxVec);
end