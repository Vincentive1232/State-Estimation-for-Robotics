function Pose_k = Convert2Pose(translation, Theta)
    Pose_k = v2skew_se3(translation, Theta);
    Pose_k = expm(Pose_k);
end