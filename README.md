# SEQ_ORB_SLAM2
# 1. Background
Fork from [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2), add the sequence class to speed up loop detection.

Merge a number of continuously keyframes into a sequence, and merge the keypoint sets from each keyframe to calculate the sequence keypoint sets(also calculate the BoW).
Here is the brief image for the final result(tested on KITTI).
![test](/res/result.png)

# 2. loop detection


