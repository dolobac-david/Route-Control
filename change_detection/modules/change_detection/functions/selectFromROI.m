%% Select indices of points from the region of interest.
function indices = selectFromROI(ptCloud, roi, figuresOn)

% roi = [xmin xmax ymin ymax zmin zmax];
if isempty(roi)
    indices = [];
else
    indices = findPointsInROI(ptCloud,roi);
    xmin= roi(1);
    xmax= roi(2);
    ymin= roi(3);
    ymax= roi(4);
    zmin= roi(5);
    zmax= roi(6);
    
    if figuresOn
        roiShowShape = [(xmax-xmin)/2+xmin (ymax-ymin)/2+ymin (zmax-zmin)/2+zmin xmax-xmin ymax-ymin zmax-zmin 0 0 0];
        pointCloudShow(ptCloud, "Region of interest",1,1);
        showShape("cuboid",roiShowShape);
        
        ptCloudSelected = select(ptCloud,indices);
        pointCloudShow(ptCloudSelected, "Selected points",1,1);
        pointCloudShowPair(ptCloud,ptCloudSelected,"Selected points",0,{},0)
        showShape("cuboid",roiShowShape);
    end
end
end