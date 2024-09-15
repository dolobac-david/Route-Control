clear;
close all;
clc;
addpath modules\change_detection\functions

route1 = "80";
route2 = "82";

% Load the point clouds.
load("data/point_clouds/denoised/pointCloud_"+route1+".mat","ptCloud");
ptCloudFixed = ptCloud;
load("data/point_clouds/denoised/pointCloud_"+route2+".mat","ptCloud");
ptCloudMoved = ptCloud;

% Load the transformation for alignment of point clouds.
load("data/transformations/tformSVD_"+route2+"_to_"+route1+".mat","tformSVD");

transfWorldPointsSVD = transformPointsForward(tformSVD, ptCloudMoved.Location);
ptCloudMoved = pointCloud(transfWorldPointsSVD, Color=ptCloudMoved.Color);

% Visualize transformed 3D points.
pointCloudShowPair(ptCloudFixed, ptCloudMoved, 'Coarse registration SVD',1,{'Fixed','Moved'}, 0);

pointCloudShow(ptCloudFixed,"Global map",1,0);
pointCloudShow(ptCloudMoved,"Current location in global map",1,0);

% Manually select indices of ground true points that represent true changes.
scenario = route1+"_and_"+route2;

% 80 and 82 scenario
% Original route 80
objectROI1Original = [14.8, 20.8, -2, 5, 10, 15];
% Current route 82
objectROI1Current = [];

% % 95 and 99_1 scenario
% % Original route 95
% objectROI1Original = [2, 4.3, -2, 1, 3, 5];
% % Current route 99_1
% objectROI1Current = [1.8, 4.8, -0.5, 1.1, 3.5, 5];

% % 100 and 101 scenario
% % Original route 100
% objectROI1Original = [2.2, 5, -2, 1.3, 4, 6];
% % Current route 101 
% objectROI1Current = [6.1, 8.6, -2, 1.3, 3.1, 5.2];

% % 102 and 103 scenario
% % Original route 102
% objectROI1Original = [];
% % Current route 103
% objectROI1Current = [7, 9, -3, 1.3, 4.9, 6];

% % 104 and 105 scenario
% % Original route 104
% objectROI1Original = [5, 12, -3, 4, 13, 18];
% % Current route 105 
% objectROI1Current = [5.2, 12, -1, 5, 14, 18];

% % 106 and 107 scenario
% % Original route 106
% objectROI1Original = [20, 26, -7.5, 4, 10, 19];
% % Current route 107 
% objectROI1Current = [18, 31, -6, 0, 22.5, 29];

% % 109 and 110 scenario
% % Original route 109
% objectROI1Original = [16, 21, -5, 4, 10, 14];
% objectROI2Original = [24.5, 31, 0.5, 4.8, 10, 15];
% % Current route 110
% objectROI1Current = [16, 22.5, 0, 4.5, 10, 15.5];
% objectROI2Current = [25.5, 30, -5, 4.8, 8, 12.5];

% % 111 and 112 scenario
% % Original route 111
% objectROI1Original = [9, 15.5, -2, 4, 12, 18];
% objectROI2Original = [20, 24, -6, 5, 10.5, 16.5];
% % Current route 112
% objectROI1Current = [10, 14.5, -6, 5, 11, 18];
% objectROI2Current = [19, 25.5, -1, 5, 11.5, 17];

% Saving ROIs.
% Original route
selectFromROI(ptCloudFixed, objectROI1Original,1);
objectsROIsOriginal = [objectROI1Original];
save("data/evaluation_ground_true/"+scenario+"/ROIs"+ route1 +".mat","objectsROIsOriginal");

% Current route
selectFromROI(ptCloudMoved, objectROI1Current,1);
objectsROIsCurrent = [objectROI1Current];
save("data/evaluation_ground_true/"+scenario+"/ROIs"+ route2 +".mat","objectsROIsCurrent");