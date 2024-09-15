%% Triangulate points from previous two views, then find correspondences with 2D points in current view.
function [worldPoints, imagePoints, reprojErrors3Dto2D, inFront3Dto2D, idx1, idx2, idxTriplet] = find3Dto2DCorrespondences(vSet, intrinsics, matchIdx, currPoints)

camPoses = poses(vSet);

% Compute the camera projection matrix for the next-to-the-last view.
camMatrix1 = cameraProjection(intrinsics, pose2extr(camPoses.AbsolutePose(end-1)));

% Compute the camera projection matrix for the last view.
camMatrix2 = cameraProjection(intrinsics, pose2extr(camPoses.AbsolutePose(end)));

% Find indices of points visible in all three views.
matchIdxPrev = vSet.Connections.Matches{end};
[~, ia, ib] = intersect(matchIdxPrev(:, 2), matchIdx(:, 1));
idx1 = matchIdxPrev(ia, 1);
idx2 = matchIdxPrev(ia, 2);
idxTriplet = matchIdx(ib, 2);

% Triangulate the points.
points1 = vSet.Views.Points{end-1};
points2 = vSet.Views.Points{end};
[worldPoints, reprojErrors3Dto2D, inFront3Dto2D] = triangulate(points1(idx1,:), points2(idx2,:), camMatrix1, camMatrix2);
imagePoints = currPoints(idxTriplet).Location;
end