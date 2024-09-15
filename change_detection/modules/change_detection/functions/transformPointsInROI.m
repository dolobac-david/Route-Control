% Rigid transformation of selected object.
function [ptCloud,indices] = transformPointsInROI(ROI, ptCloud, eulerAngles, t)

indices = selectFromROI(ptCloud, ROI);
pointsToMove = select(ptCloud,indices);

tform = rigidtform3d(eulerAngles,t);

pointsMoved = transformPointsForward(tform, pointsToMove.Location);

origPoints = ptCloud.Location;
origPoints(indices,:) = pointsMoved;
ptCloud = pointCloud(origPoints,Color= ptCloud.Color);
end

