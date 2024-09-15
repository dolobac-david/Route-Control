%% Detect final surface change considering the local neigborhood of given voxels.
function finalSurfaceChangeMask = detectSurfaceChange(surfaceChangeMask, bins)

% Investigate the neighbourhood (26 nearby voxels) of current voxel and decide about change.
surfaceChangeIdx = find(surfaceChangeMask);

[rows, cols, pages] = size(bins);
finalSurfaceChangeMask = zeros(rows, cols, pages);
for i=1:height(surfaceChangeIdx)

    % Get the position of voxel in global grid.
    [row, col, page] = ind2sub(size(surfaceChangeMask), surfaceChangeIdx(i));

    % Get the neighborhood of current voxel.
    supervoxelSurfaceChange = surfaceChangeMask(row-1:row+1, col-1:col+1, page-1:page+1);
    
    % Supervoxel contains only changed voxels - flag 1. (no unchanged voxels with flag 2 present) 
    if numel(find(supervoxelSurfaceChange == 1)) > 0 && numel(find(supervoxelSurfaceChange == 2)) == 0
        finalSurfaceChangeMask(row-1:row+1, col-1:col+1, page-1:page+1) = supervoxelSurfaceChange;    
    end
end

% Rest of the voxels mark as unchanged with flag 2.
nonChangeIdx = setdiff(surfaceChangeIdx, find(finalSurfaceChangeMask));
finalSurfaceChangeMask(nonChangeIdx) = 2;
end