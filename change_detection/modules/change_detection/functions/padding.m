%% Pad the voxelized space.
function [bins1Padded, bins2Padded, binLocationsPadded] = padding(bins1, bins2, binLocations)

% Padding of space grid for neighbourhood investigation.
[rows, cols, pages] = size(bins1);
bins1Padded = cell(rows+2, cols+2, pages+2);
bins2Padded = cell(rows+2, cols+2, pages+2);
binLocationsPadded = cell(rows+2, cols+2, pages+2);

[rowsPad, colsPad, pagesPad] = size(bins1Padded);
bins1Padded(2:rowsPad-1, 2:colsPad-1, 2:pagesPad-1) = bins1;
bins2Padded(2:rowsPad-1, 2:colsPad-1, 2:pagesPad-1) = bins2;
binLocationsPadded(2:rowsPad-1, 2:colsPad-1, 2:pagesPad-1) = binLocations;
end