function savePCDenoise(route,ptCloudDenoised,wpSetDenoised)

ptCloud = ptCloudDenoised;
wpSet = wpSetDenoised;

save("data/point_clouds/denoised/pointCloud_"+ route+".mat","ptCloud");
save("data/wpSet_structure/denoised/wpSet_structure_"+ route+".mat","wpSet");
end

