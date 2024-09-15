%% Create orientation histograms of point normals encapsulated in given voxels. 
function orientationHistograms = createOH(voxelMask, voxelLocations, ptCloud, binsPadded, gridSize, ...
    nHistBins)

% Choose voxels and get the indices of their points.
voxelIdx = find(voxelMask);
pointsIdx = binsPadded(voxelIdx);

% % Visualize investigated voxels.
% pointCloudShow(ptCloud, 'No structural change in voxels',1,0);
% drawVoxels(voxelLocations, gridSize, lines(1));

% Compute the azimuth and elevation angles of normals and create their histogram for
% every given voxel.
n = numel(voxelIdx);
orientationHistograms = cell(1,n);
% x = ptCloud.Location(:,1);
% y = ptCloud.Location(:,2);
% z = ptCloud.Location(:,3);
for i=1:n
%     drawVoxels(voxelLocations{i}, gridSize, "yellow");
    s = ptCloud.Normal(pointsIdx{i},:);
%     u = s(:,1);
%     v = s(:,2);
%     w = s(:,3);

%     ptCloudVisualize = pointCloud(ptCloud.Location(pointsIdx{i},:),Color=ptCloud.Color(pointsIdx{i},:));
%     pointCloudShow(ptCloudVisualize, "Normals of points",1,0)
%     quiver3(x(pointsIdx{i}),y(pointsIdx{i}),z(pointsIdx{i}),u,v,w);

%     elevation1 = acosd(s(:,3)./vecnorm(s,2,2));
%     elevation2 = atand(s(:,3) ./ sqrt(s(:,1).^2 + s(:,2).^2));
%     elevation3 = atan2d(s(:,3),sqrt(s(:,1).^2 + s(:,2).^2));
%     azimuth1 = atand(s(:,2)./s(:,1));
    [a,e] = cart2sph(s(:,1),s(:,2),s(:,3));

    % phi
    azimuth = rad2deg(a);
    % theta
    elevation = rad2deg(e);
    
    N = histcounts2(azimuth, elevation, nHistBins, XBinLimits=[-180 180], YBinLimits=[-90 90]);
    orientationHistograms{i} = N;

%     figure
%     h = histogram2(azimuth,elevation,NumBins=[8 4],XBinLimits=[-180 180], YBinLimits=[-90 90], ...
%         DisplayStyle="bar3",FaceColor="flat",ShowEmptyBins="on");
%     colorbar
%     xlabel('azimuth') 
%     ylabel('elevation') 
%     zlabel('count') 
% 
%     close;
end
%hold off;
end