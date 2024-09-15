%% Relative pose of camera extracted from essential matrix; also determining epipolar inliers through RANSAC.
function [relPose, inlierIdx, essentialMatrixError,E] = estimateRelativePose(matchedPoints1, matchedPoints2, intrinsics)

if ~isnumeric(matchedPoints1)
    matchedPoints1 = matchedPoints1.Location;
end

if ~isnumeric(matchedPoints2)
    matchedPoints2 = matchedPoints2.Location;
end

for i = 1:100
    % Estimate the essential matrix.    
    [E, inlierIdx, status] = estimateEssentialMatrix(matchedPoints1, matchedPoints2,...
        intrinsics);
    
    % Make sure there is enough matched points and inliers
    if status == 1
        essentialMatrixError = true;
        relPose = rigidtform3d;
        return;
    elseif status == 2
        continue;
    end

    % Get the epipolar inliers.
    inlierPoints1 = matchedPoints1(inlierIdx, :);
    inlierPoints2 = matchedPoints2(inlierIdx, :);

    % Compute the camera pose from the essenial matrix
    [relPose, validPointFraction] = estrelpose(E, intrinsics, inlierPoints1, inlierPoints2);

    if validPointFraction > 0.95
       essentialMatrixError = false;
       return;
    end
end

% After 100 attempts validPointFraction is still too low.
essentialMatrixError = true;
relPose = rigidtform3d;
end