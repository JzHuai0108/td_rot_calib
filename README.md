# td_rot_calib
Matlab functions to estimate time offset and relative rotation.
Tested on matlab 2024a

Implementation of the correlation algorithm in our [snail radar paper](https://arxiv.org/pdf/2407.11705).

## Entrypoint functions

estimateRotAndTdByPose2.m: given two pose files.
estimateRotAndTdByPoseAndVel.m: given a pose file and a velocity file.
estimateRotAndTdByPose.m: given an imu file and a pose file.
estimateRotAndTd.m: given two imu files.

## Function list
.
├── alignVector3Sequences.m
├── associate2.m % faster associate
├── associate.m % tum associate
├── computeAngularRateFromAttitude.m
├── convertPose2IMUData.m
├── convertPose2LocalVelocity.m
├── cubicfit.m
├── estimateDelayOfSensorReading.m
├── estimateRelativeRotation.m
├── estimateRelativeRotationRobust.m
├── estimateRotAndTdByPose2.m
├── estimateRotAndTdByPoseAndVel.m
├── estimateRotAndTdByPose.m
├── estimateRotAndTd.m
├── findApexByFit2.m
├── findTimeOffset.m
├── readme.md
├── removeRepetitiveEntriesAndReorder.m
├── tests
└── writeRotAndTd.m

