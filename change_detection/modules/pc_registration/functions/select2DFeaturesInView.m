%% Select 2D features and 2D points in current view that correspond to 3D worldpoints.
function [features, points, I, worldPointIndices]  = select2DFeaturesInView(wpSet, viewId, vSet, imds, keyFramesIdxVec, intrinsics)

[worldPointIndices, featureIndices] = findWorldPointsInView(wpSet,viewId);
features = vSet.Views.Features{viewId}(featureIndices,:);
points = vSet.Views.Points{viewId}(featureIndices);
I = readimage(imds, keyFramesIdxVec(viewId));
I = undistortImage(I, intrinsics);
end