%% Draw the cuboid voxels.
function drawVoxels(locations, gridSize, color)

if iscell(locations)
    n = numel(locations);
else
    n=1;
end

for i=1:n
    if n == 1
        cubeBounds = locations;
    else
        cubeBounds = locations{i};
    end

    if isempty(cubeBounds)
        continue;
    end
    
    xctr = cubeBounds(1,1) + gridSize/2;
    yctr = cubeBounds(2,1) + gridSize/2;
    zctr = cubeBounds(3,1) + gridSize/2;
    showShape("cuboid",[xctr yctr zctr gridSize gridSize gridSize 0 0 0],Color=color);
end
end