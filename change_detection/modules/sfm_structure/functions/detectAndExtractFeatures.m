%% Features and keypoints extraction.
function [features, points, I] = detectAndExtractFeatures(I, intrinsics)

% Undistort the image
I = undistortImage(I, intrinsics);

% Detect keypoints
points = detectSURFFeatures(im2gray(I), 'MetricThreshold', 0); 
[features, points] = extractFeatures(im2gray(I), points);
end