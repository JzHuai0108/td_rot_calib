
ratesa = rand(3000, 3) - 0.5;
a_R_b = rotmat(quaternion(rand(1, 4) - 0.5), "point");
noise = 0.01;
rotvec = rand(1, 3) * noise;
rotnoise = rotvec2mat3d(rotvec);
a_R_b_noisy = a_R_b * rotnoise;
ratesb = ratesa * a_R_b_noisy;
dt = 0.05;
times = 0:dt:dt * (3000-1);
deltat = 1.5;
ratesa = [times', ratesa];
ratesb = [times' + deltat, ratesb];

close all;
[time_offset, ~] = alignVector3Sequences(ratesa, ratesb, 2:4, a_R_b_noisy, 5, '');

assert(norm(time_offset + deltat) < 1e-5, "Failed to estimate time offset");
ratesb(:, 1) = ratesb(:, 1) + time_offset;

[a_R_b_est, matchesEM] = estimateRelativeRotation(ratesa, ratesb, 2:4);

assert(norm(a_R_b_est - a_R_b) < noise, "Failed to estimate relative rotation");
