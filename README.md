# SEQ_ORB_SLAM2
# 1. Background
Fork from [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2), add the sequence class to speed up loop detection.

Merge a number of continuously keyframes into a sequence, and merge the keypoint sets from each keyframe to calculate the sequence keypoint sets(also calculate the BoW).

Here is the brief image for the final result(tested on KITTI).
![test](/res/result.png)

# 2. loop detection
For each sequence, calculate the BoW score with all previous sequences, verify if there is a matched sequence.

Here is the output result for loop detection.
![sequence match](/res/localmapping.png)

# 3. Result
Compare the time usage for loop clusure thread between SEQ_ORB_SLAM2 and the ORB_SLAM2, and the result shown as the following graph.
![time usage compare](/res/time_usage.png)

