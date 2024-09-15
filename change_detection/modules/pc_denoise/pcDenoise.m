% Module for denoising point cloud data. 

% Tunable parameters. 
% distance threshold - minimal average distance of point to its k-nearest
% neighbors for which the point is considered outlier
%
% k - number of neighbors, lower it is more sensitive the filter is to noise,
% higher values mean more robustness to noise but also more computation
% time

function [ptCloudDenoised,wpSetDenoised] = pcDenoise(route,distThreshold, k)
clc;

load("data/point_clouds/raw/pointCloud_"+route+".mat","ptCloud");
load("data/wpSet_structure/raw/wpSet_structure_"+route+".mat","wpSet");

% Remove outliers from the point clouds with statistical filtering.
[ptCloudDenoised, idxDenoised, idxNoise] = pcdenoise(ptCloud, NumNeighbors=k, Threshold=distThreshold);

% Select the denoised subset of 3D points.
ptCloudDenoised = pointCloud(ptCloudDenoised.Location, Color=ptCloud.Color(idxDenoised,:));
wpSetDenoised = removeWorldPoints(wpSet,idxNoise);

figure
pcshowpair(pointCloud(ptCloud.Location), pointCloud(ptCloudDenoised.Location), VerticalAxis='y', VerticalAxisDir='down', AxesVisibility="on", ...
    MarkerSize=45,Projection="perspective");
hold on
grid on
xlabel('X') 
ylabel('Y') 
zlabel('Z') 
title("Point cloud " + route + " for k = " + k);
legend({'Raw','Denoised'},'TextColor','w')
legend('Location','southoutside')
hold off

figure
pcshow(ptCloudDenoised, VerticalAxis='y', VerticalAxisDir='down', AxesVisibility="on", ...
    MarkerSize=45,Projection="perspective");
hold on
grid on
xlabel('X') 
ylabel('Y') 
zlabel('Z') 
title("Denoised point cloud " + route + " for k = " + k)
hold off

end