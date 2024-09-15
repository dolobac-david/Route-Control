%% Remove points encapsulated in given voxels from point cloud.
function ptCloudOut = removePointsInVoxels(ptCloudIn, binsPadded, voxels)

pointIdxCell = binsPadded(voxels);
pointIdxVec = [];
for i=1:height(pointIdxCell)
    pointIdxVec = [pointIdxVec pointIdxCell{i}'];
end

ptCloudOut = removePoints(ptCloudIn, pointIdxVec);
end