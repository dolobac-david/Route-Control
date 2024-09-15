%% Check the parallax of triangulated points between two frames.
% Parallax angle is in degrees.
function [parallaxDegrees, isParallaxValid,reprojectionErrors,isInFront] = checkParallax(intrinsics, pose1, pose2, inlierPoints1, ...
    inlierPoints2, minParallax)

camMatrix1 = cameraProjection(intrinsics, pose2extr(pose1));
camMatrix2 = cameraProjection(intrinsics, pose2extr(pose2));  

[xyzPoints, reprojectionErrors, isInFront] = triangulate(inlierPoints1, inlierPoints2, camMatrix1, ...
    camMatrix2);

% % Filter points by view direction and reprojection error.
% minReprojError = 1;
% inlierTriangulationIdx  = isInFront & (reprojectionErrors < minReprojError);
% xyzPoints  = xyzPoints(inlierTriangulationIdx, :);

% A good two-view with significant parallax
ray1       = xyzPoints - pose1.Translation;
ray2       = xyzPoints - pose2.Translation;
cosAngle   = sum(ray1 .* ray2, 2) ./ (vecnorm(ray1, 2, 2) .* vecnorm(ray2, 2, 2));

% Check parallax in degrees
parallaxDegrees = acosd(cosAngle);
isParallaxValid = all(parallaxDegrees > minParallax & parallaxDegrees < 90);

% % Check parallax in radians
% isParallaxValid = all(cosAngle < cosd(minParallax) & cosAngle > 0);
end