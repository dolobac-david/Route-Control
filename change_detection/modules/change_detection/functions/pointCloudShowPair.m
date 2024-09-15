%% Visualize pair of point cloud data.
function pointCloudShowPair(ptCloud1, ptCloud2, titleText, legendOn, legendText, offsetOn)

axOffset = 0.8;

figure;
pcshowpair(pointCloud(ptCloud1.Location), pointCloud(ptCloud2.Location), VerticalAxis='y', VerticalAxisDir='down', ...
    AxesVisibility="on", MarkerSize=45, Projection="perspective");
hold on;
grid on
xlabel('X') 
ylabel('Y') 
zlabel('Z') 
if offsetOn
    xlim([ptCloud2.XLimits(1)-axOffset ptCloud2.XLimits(2)+axOffset])
    ylim([ptCloud2.YLimits(1)-axOffset ptCloud2.YLimits(2)+axOffset])
    zlim([ptCloud2.ZLimits(1)-axOffset ptCloud2.ZLimits(2)+axOffset])
end
title(titleText)
if legendOn
    legend(legendText,'TextColor','w')
    legend('Location','southoutside')
end
end