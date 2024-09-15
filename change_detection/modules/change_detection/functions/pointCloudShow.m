%% Visualize point cloud data.
function pointCloudShow(ptCloud, titleText, newFig, offsetOn)

axOffset = 0.8;

if newFig
    figure;
end
pcshow(ptCloud, VerticalAxis='y', VerticalAxisDir='down', ...
    AxesVisibility="on", MarkerSize=45, Projection="perspective");
hold on;
grid on
xlabel('X') 
ylabel('Y') 
zlabel('Z') 
if offsetOn
    xlim([ptCloud.XLimits(1)-axOffset ptCloud.XLimits(2)+axOffset])
    ylim([ptCloud.YLimits(1)-axOffset ptCloud.YLimits(2)+axOffset])
    zlim([ptCloud.ZLimits(1)-axOffset ptCloud.ZLimits(2)+axOffset])
end
title(titleText)
end