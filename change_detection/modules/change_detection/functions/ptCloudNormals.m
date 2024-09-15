%% Estimate normals of given point cloud.
function ptCloudNormals(ptCloud, f)

% Compute normals for the whole point cloud
% normals = pcnormals(ptCloud,round(ptCloud.Count/f)); 
normals = pcnormals(ptCloud,f); 
x = ptCloud.Location(:,1);
y = ptCloud.Location(:,2);
z = ptCloud.Location(:,3);
u = normals(:,1);
v = normals(:,2);
w = normals(:,3);

% pointCloudShow(ptCloud, 'Normals of given points',1,1);
% quiver3(x,y,z,u,v,w);

% Unify the direction of surface normals wrt camera position at [0,0,0].
cameraPosition = [0,0,0];
for i=1:height(normals)
    vector1 = cameraPosition - [x(i) y(i) z(i)];
    vector2 = [u(i) v(i) w(i)];

    % Change the direction of normal vector if it points inward of the surface wrt the camera 
    % location.
    angle = atan2(norm(cross(vector1,vector2)),vector1*vector2');
    if angle > pi/2 || angle < -pi/2
       u(i) = -u(i);
       v(i) = -v(i);
       w(i) = -w(i);
   end
end
normals = [u v w];
ptCloud.Normal = normals;
% pointCloudShow(ptCloud, 'Normals with outward orientation',1,1);
% quiver3(x,y,z,u,v,w);
end
