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
```
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
```

## Use example for lidar imu temporal and extrinsic rotation calibration

### Use kiss-icp to process the lidar data

*Clone* [kissicp](git@github.com:JzHuai0108/kiss-icp), checkout branch: inhouse-seqs

*Build* refer to [kissicp readme](https://github.com/JzHuai0108/kiss-icp/blob/inhouse-seqs/README.md)

Basically,
```
cd src/kiss-icp
make editable
kiss_icp_pipeline --help
```

*Run* the [script](https://github.com/JzHuai0108/kiss-icp/blob/inhouse-seqs/config/run_vlp16_kuangye.sh), note to change the *bag path* and *result_dir* in the script.

### Extract the imu data from the rosbag

*Clone* [vio_common](https://github.com/JzHuai0108/vio_common)

*Run*

```
python3 python/bag_to_imu.py bag /imu/data
```

### Td and rot calib with correlation

Open matlab, change working dir to td_and_rot calib folder

Also find the kiss-icp odometry output xxx_tum.txt and the xxx_imu_data.txt.

Run 

```
i_R_p_init = [ 0, 0, 1; 0, 1, 0; -1, 0, 0]; % lidar pose relative to IMU frame rough guess.
iwinsize = 9; % imu smoothing window size
pwinsize = 1; % lidar odometry smoothing window size

[i_R_p, time_offset, td_list] = estimateRotAndTdByPose('xxx_imu_data.txt', 'xxx_tum.txt', outputdir, i_R_p_init, iwinsize, pwinsize)

```

The resulting time offset is *time_offset* which equals to the fourth value of td_list.
