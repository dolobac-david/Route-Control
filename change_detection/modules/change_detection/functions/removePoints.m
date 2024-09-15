%% Remove points from point cloud given their indices.
function ptCloudOut = removePoints(ptCloudIn, pointsIdx2Remove)

newPoints = ptCloudIn.Location;
newColors = ptCloudIn.Color;
newPoints(pointsIdx2Remove,:) = [];
newColors(pointsIdx2Remove,:) = [];
ptCloudOut = pointCloud(newPoints, Color=newColors);
end