function  saveStructure(route,ptCloud,vSet,wpSet)

save("data/point_clouds/raw/pointCloud_"+route +".mat", "ptCloud");
save("data/vSet_structure/vSet_structure_"+route +".mat", "vSet");
save("data/wpSet_structure/raw/wpSet_structure_"+route +".mat", "wpSet");
end