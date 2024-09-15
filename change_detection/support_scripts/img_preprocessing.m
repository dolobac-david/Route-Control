clear;
close all;
clc;

a=dir('images/*.avi');

for i=114:114
    read_video(['MOVI' sprintf('%04d',i) '.avi'], i);
end

% rgbDirectory = 80;
% labConversion(rgbDirectory);
%% Process the video frame by frame.
function [] = read_video(video_file, idx)
    mkdir("data/images/" + num2str(idx));
    video_original = VideoReader(video_file);
    
    wait = waitbar(0,"Wait...");
    for i=1:video_original.NumFrames
        img = readFrame(video_original);
        imwrite(img,['data/images/' num2str(idx) '\' sprintf('%04d',i) '.jpg']);

        waitbar(i/video_original.NumFrames,wait,string((i/video_original.NumFrames*100) + " %"));
    end
    close(wait);
end

%% Convert the RGB frames to LAB color space and perform histogram equalization,
% then convert back to RGB.
function [] = labConversion(rgbDirectory)

% Load the RGB frames.
imageDir = fullfile(string(rgbDirectory));
imds = imageDatastore(imageDir);

% Create folder for Lab frames.
if ~exist(num2str(rgbDirectory) + "_histeq", 'dir')
    mkdir(num2str(rgbDirectory) + "_histeq");
end

% Perform histogram equalization on L channel for every frame, then convert outcomes back 
% to RGB color space.
wait = waitbar(0,"Wait...");
for i=1:numel(imds.Files)
    labImage = rgb2lab(readimage(imds, i));
    L = labImage(:,:,1);
    figure(1);
    imhist(L/100);
    lEq = histeq(L/100);
    figure(2);
    imhist(lEq);
    lEq = lEq * 100;
    labImage(:,:,1) = lEq;
    rgbImageEnhanced = lab2rgb(labImage);

    figure(3)
    imshow(readimage(imds, i));

    figure(4);
    imshow(rgbImageEnhanced);

    imwrite(rgbImageEnhanced,[num2str(rgbDirectory) '_histeq' '\' sprintf('%04d',i) '.jpg']);

    waitbar(i/numel(imds.Files),wait,string((i/numel(imds.Files)*100) + " %"));
end
close(wait);
end