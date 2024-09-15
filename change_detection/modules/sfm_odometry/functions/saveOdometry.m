% Save data from odometry module.
function saveOdometry(route, vSet, keyFramesIdxVec)

save("data/vSet_odometry/vSet_odometry_" + route + ".mat", "vSet");
save("data/key_frames_indices/key_frames_indices_" + route + ".mat", "keyFramesIdxVec"); 
end

