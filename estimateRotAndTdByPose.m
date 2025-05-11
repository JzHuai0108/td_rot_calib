function [i_R_p, time_offset, td_list] = estimateRotAndTdByPose(imufile, posefile, outputdir, i_R_p_init, iwinsize, pwinsize)
% imufile each line: time[sec], gx, gy, gz[rad/sec], ax, ay, az[m/s^2]
% posefile each line: time tx ty tz qx qy qz qw
% t: position of sensor in world frame, 
% q: rotation to world from sensor frame.

% Author: Jianzhu Huai
% Date: 2024

if ~exist('iwinsize', 'var')
    iwinsize = 1;
end
if ~exist('pwinsize', 'var')
    pwinsize = 1;
end

if nargin < 4
    i_R_p_init = [0.0300840665779303        -0.999542073967788       0.00325442872203546;
             0.999544576986591         0.030076123204073      -0.00246280931267187;
           0.00236380092897233       0.00332703789963079          0.99999167159731]; % zed_R_lidar
    
    % i_R_p_init = [-0.00665358785409892        -0.999969094299433       0.00418810394319119;
    %          0.999355203065517      -0.00679716297882199       -0.0352558744228315;
    %         0.0352832520404078       0.00395082540876196         0.999369542813889]; % bynavimu_R_lidar
    fprintf('Using default i_R_p_init\n');
    disp(i_R_p_init);
end

poses = readmatrix(posefile, 'ConsecutiveDelimitersRule', 'join');
poses = removeRepetitiveEntriesAndReorder(poses);

omega_i = readmatrix(imufile, 'ConsecutiveDelimitersRule', 'join');
norm1 = norm(omega_i(1, 2:4));
norm2 = norm(omega_i(1, 4:6));
if norm1 > 9
    fprintf('Swap 2:4 and 5:7 columns for data from the imufile %s\n', imufile);
    tmp = omega_i(:, 2:4);
    omega_i(:, 2:4) = omega_i(:, 5:7);
    omega_i(:, 5:7) = tmp;
end
% now omega_i: time, gx, gy, gz, ax, ay, az

suppressLargeValues = true;
omega_p = convertPose2IMUData(poses, suppressLargeValues);
tmp = omega_p(:, 2:4);
omega_p(:, 2:4) = omega_p(:, 5:7);
omega_p(:, 5:7) = tmp;
% now omega_p: time, gx, gy, gz, ax, ay, az

close all;
fprintf('Aligning %s and %s\n', posefile, imufile);
[time_offset, td_list] = alignVector3Sequences(omega_i, omega_p, 2:4, i_R_p_init, 5, outputdir, iwinsize, pwinsize);
fprintf('pose data original time + %.9f sec = pose data time in IMU clock.\n', time_offset);
omega_p(:, 1) = omega_p(:, 1) + time_offset;

[i_R_p, bias, matchesEM]= estimateRelativeRotationRobust(omega_i, omega_p, 2:4);
disp('Estimated i_R_p by angular rate: may be unreliable in degenerate motion.');
if ~isempty(i_R_p)
    fprintf('i_R_p:\n')
    fprintf('%.9f %.9f %.9f\n%.9f %.9f %.9f\n%.9f %.9f %.9f\n', ...
        i_R_p(1, 1), i_R_p(1, 2), i_R_p(1, 3), ...
        i_R_p(2, 1), i_R_p(2, 2), i_R_p(2, 3), ...
        i_R_p(3, 1), i_R_p(3, 2), i_R_p(3, 3));
    fprintf('bias: %.6f %.6f %.6f\n', bias);
end
end
