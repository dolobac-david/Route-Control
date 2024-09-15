% Select indices of points that are manually marked as ground true
% change.
function idxGT = selectGTPoints(objectsROIs, ptCloud, figuresOn)

n = height(objectsROIs);
idxGT = [];
if ~isempty(objectsROIs)
    for i=1:n
        currROI = objectsROIs(i,:);
        idxGTCurrentObj = selectFromROI(ptCloud, currROI,figuresOn);
        idxGT = [idxGT;idxGTCurrentObj];
    end
end
end

