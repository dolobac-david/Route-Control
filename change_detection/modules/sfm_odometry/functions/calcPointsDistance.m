%% Calculate euclidean distance (in pixel coordinates) between points in two frames.
function [enoughDistance, pointsDistance, minDistance] = calcPointsDistance(points1, points2)

minDistance = 15;
pointsDistance = vecnorm((points2.Location - points1.Location),2,2);

if all(pointsDistance > minDistance)
    enoughDistance = true;
else 
    enoughDistance = false;
end
end