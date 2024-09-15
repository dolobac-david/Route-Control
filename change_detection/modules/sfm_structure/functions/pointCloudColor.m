%% Get the color of reconstructed 3D points from the pictures.
function colorVec = pointCloudColor(tracks, keyFramesIdxVec, imds, intrinsics)

colorVec = uint8(zeros(width(tracks),3));
numPixels = intrinsics.ImageSize(1) * intrinsics.ImageSize(2);
I_vec = cell(1,width(keyFramesIdxVec));

% Load the keyframes.
for i=1:width(keyFramesIdxVec)
    I = undistortImage(readimage(imds, keyFramesIdxVec(i)), intrinsics);
    I_vec{1,i} = I;
end

C = cell(width(tracks),width(keyFramesIdxVec));
for i=1:width(tracks)
    % Get the index of keyframe of a world point.
    viewId = tracks(i).ViewIds(1);

    % Get the x,y location of a point in image and store it in cell array, points from same 
    % image go to same column, each row corresponds to one individual point.
    C{i,viewId} = tracks(i).Points(1,:);
end

for i=1:width(keyFramesIdxVec)
    % Get the xy locations of all points in current keyframe. 
    I = I_vec{1,i};
    idxC = ~cellfun(@isempty,C(:,i));
    if nnz(idxC) == 0
        break;
    end
    locationXY = cell2mat(C(idxC, i));

    % Reshape image matrix to get the color of every pixel.
    allColors = reshape(I, [numPixels, 3]);
    colorIdx = sub2ind([intrinsics.ImageSize(1), intrinsics.ImageSize(2)], round(locationXY(:,2)), round(locationXY(:,1)));
    
    % Store the color of the points in vector.
    colorVec(idxC,:) = allColors(colorIdx, :);
end
end