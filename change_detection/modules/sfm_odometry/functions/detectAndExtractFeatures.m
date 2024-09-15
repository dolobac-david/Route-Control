%% Features and keypoints extraction.
function [features, points, I, threshold, allPoints] = detectAndExtractFeatures(I, intrinsics, allPoints, i, numPoints, threshold)

% Undistort the image
I = undistortImage(I, intrinsics);

points = detectSURFFeatures(im2gray(I), 'MetricThreshold', threshold);
allPoints(i) = height(points);

points   = selectUniform(points, numPoints, size(I));

[features, points] = extractFeatures(im2gray(I), points);
end

