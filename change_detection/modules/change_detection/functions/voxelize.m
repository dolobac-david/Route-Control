%% Voxelize point cloud data.
% Compute the number of bins (integer) and set the boundaries in each dimension so that the 
% length of every bin accross x,y,z equals gridSize.
% Result is a space uniformly divided into cubes with the length of gridSize.
% Cubes represent the bins in which the points are stored.
function [bins1, bins2, binLocations, ptCloudFixedCropped, idxOfInvestigatedPoints] = voxelize(gridSize, ptCloudFixed, ptCloudMoved)

% Get the boundaries of the registered point clouds.
XLimit1 = ptCloudMoved.XLimits(1);
XLimit2 = ptCloudMoved.XLimits(2);
YLimit1 = ptCloudMoved.YLimits(1);
YLimit2 = ptCloudMoved.YLimits(2);
ZLimit1 = ptCloudMoved.ZLimits(1);
ZLimit2 = ptCloudMoved.ZLimits(2); 

% X
intLimitX2 = ceil(XLimit2);
nBinsX = (intLimitX2 - floor(XLimit1)) / gridSize;

while mod(nBinsX,1) ~= 0
    intLimitX2 = intLimitX2 + 1;
    nBinsX = (intLimitX2 - floor(XLimit1)) / gridSize;
end

% Y
intLimitY2 = ceil(YLimit2);
nBinsY = (intLimitY2 - floor(YLimit1)) / gridSize;

while mod(nBinsY,1) ~= 0
    intLimitY2 = intLimitY2 + 1;
    nBinsY = (intLimitY2 - floor(YLimit1)) / gridSize;
end

% Z
intLimitZ2 = ceil(ZLimit2);
nBinsZ = (intLimitZ2 - floor(ZLimit1)) / gridSize;

while mod(nBinsZ,1) ~= 0
    intLimitZ2 = intLimitZ2 + 1;
    nBinsZ = (intLimitZ2 - floor(ZLimit1)) / gridSize;
end

roi = [floor(XLimit1) intLimitX2 floor(YLimit1) intLimitY2 floor(ZLimit1) intLimitZ2];
idxOfInvestigatedPoints = findPointsInROI(ptCloudFixed, roi);

ptCloudFixedCropped = select(ptCloudFixed, idxOfInvestigatedPoints);

[bins1, binLocations] = pcbin(ptCloudFixedCropped, [nBinsX nBinsY nBinsZ], [floor(XLimit1) intLimitX2; ...
    floor(YLimit1) intLimitY2; floor(ZLimit1) intLimitZ2]);

bins2 = pcbin(ptCloudMoved, [nBinsX nBinsY nBinsZ], [floor(XLimit1) intLimitX2; ...
    floor(YLimit1) intLimitY2; floor(ZLimit1) intLimitZ2]);
end