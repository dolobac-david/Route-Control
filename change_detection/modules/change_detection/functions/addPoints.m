%% Add points from one point cloud to another.
function ptCloudTo = addPoints(ptCloudFrom, ptCloudTo, pointsIdx2Add)

newPoints = ptCloudFrom.Location(pointsIdx2Add,:);
newColors = ptCloudFrom.Color(pointsIdx2Add,:);
ptCloudTo = pointCloud([ptCloudTo.Location; newPoints], Color = [ptCloudTo.Color; newColors]);
end