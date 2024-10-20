function [a_R_b, time_offset, td_list] = estimateRotAndTd(imuafile, imubfile, outputdir, a_R_b_init)
% imufile each line: time[sec], gx, gy, gz[rad/sec], ax, ay, az[m/s^2]

% Author: Jianzhu Huai
% Date: 2014

if nargin < 4
    a_R_b_init = eye(3);
end

omega_b = readmatrix(imubfile);
omega_a = readmatrix(imuafile);

close all;
fprintf('Aligning %s and %s\n', imuafile, imubfile);
[time_offset, td_list] = alignVector3Sequences(omega_a, omega_b, 2:4, a_R_b_init, 5, outputdir);
fprintf('imu b original time + %.9f sec = imu b time in imu a clock.\n', time_offset);
omega_b(:, 1) = omega_b(:, 1) + time_offset;

[a_R_b, matchesEM]= estimateRelativeRotation(omega_a, omega_b, 2:4);
disp('Estimated a_R_b by angular rate: may be unreliable in degenerate motion.');
fprintf('a_R_b:\n')
fprintf('%.9f %.9f %.9f\n%.9f %.9f %.9f\n%.9f %.9f %.9f\n', ...
    a_R_b(1, 1), a_R_b(1, 2), a_R_b(1, 3), ...
    a_R_b(2, 1), a_R_b(2, 2), a_R_b(2, 3), ...
    a_R_b(3, 1), a_R_b(3, 2), a_R_b(3, 3));
end
