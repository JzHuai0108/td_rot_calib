function [p_R_v, time_offset, td_list, stdeps] = estimateRotAndTdByPoseAndVel(...
    posefile, velfile, outputdir, p_R_v_init, p_t_v_init, pwinsize, vwinsize)
% posefile each line: time[sec], x, y, z[m], qx, qy, qz, qw (position of sensor p in a world frame A, rotation to world A from sensor p frame)
% velfile each line: time[sec], vx, vy, vz[m/s] (velocities of the local v frame relative to a world frame B, expressed in the local v frame)
% outputdir: output directory
% p_R_v_init: initial guess of p_R_v, optional. Default is identity.
% p_t_v_init: initial guess of p_t_v, position of v sensor wrt p sensor
% frame.

% Author: Jianzhu Huai
% Date: 2024

if ~exist('pwinsize', 'var')
    pwinsize = 1;
end
if ~exist('vwinsize', 'var')
    vwinsize = 1;
end
if nargin < 5
   p_t_v_init = [0; 0; 0]; 
end
if nargin < 4
    p_R_v_init = [1 0 0;
                  0 1 0;
                  0 0 1]; % x36dimu_R_ars548 or x36dimu_R_oculii
end

poses = readmatrix(posefile);
poses = removeRepetitiveEntriesAndReorder(poses);
vels_p = convertPose2LocalVelocity(poses, p_t_v_init); % P_v_{WV}

vels = readmatrix(velfile, 'ConsecutiveDelimitersRule', 'join'); % V_v_{WV}
vels = removeRepetitiveEntriesAndReorder(vels);

close all;
fprintf('Aligning %s and %s\n', posefile, velfile);
[time_offset, td_list] = alignVector3Sequences(vels_p, vels, 2:4, p_R_v_init, 5, outputdir, pwinsize, vwinsize);
fprintf('local vel data original time + %.9f sec = local vel time in pose sensor clock.\n', time_offset);
vels(:, 1) = vels(:, 1) + time_offset;

[p_R_v, bias, matchesEM, stdeps]= estimateRelativeRotationRobust(vels_p, vels, 2:4, 0.01/5);
if ~isempty(p_R_v)
    disp('Estimated p_R_v from matched local velocity: may be unreliable in degenerate motion.');
    fprintf('p_R_v:\n')
    fprintf('%.9f %.9f %.9f\n%.9f %.9f %.9f\n%.9f %.9f %.9f\n', ...
        p_R_v(1, 1), p_R_v(1, 2), p_R_v(1, 3), ...
        p_R_v(2, 1), p_R_v(2, 2), p_R_v(2, 3), ...
        p_R_v(3, 1), p_R_v(3, 2), p_R_v(3, 3));
    fprintf('bias: %.6f %.6f %.6f\n', bias);
end
end
