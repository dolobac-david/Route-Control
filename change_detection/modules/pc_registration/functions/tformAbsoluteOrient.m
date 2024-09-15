%% Compute similarity transformation between two sets of 3D points using
% singular value decomposition.
% R - rotation matrix, t - translation vector, s - scale scalar,
% x - points to transform, y - fixed points.
function [R,t,s,tform,invalidR] = tformAbsoluteOrient(x,y)

% Flag for validity of rotation matrix.
invalidR = false;

% Centroids of point clouds.
y0 = mean(y);
x0 = mean(x);

% Cross covariant matrix.
H = (x-x0)'*(y-y0);

% Rotation matrix.
[U,~,V] = svd(H);
R = V * U';

% Check if R is rotation matrix.
if(int8(det(R)) ~= 1)
    invalidR = true;
    t = zeros(1,3);
    s = 0;
    tform = simtform3d;
    return;
end

% Scale factor.
s = sqrt(sum(vecnorm((y-y0),2,2).^2) / sum(vecnorm((x-x0),2,2).^2));

% Translation.
t = y0' - s*R*x0';

tform = simtform3d(s,R,t');
end