quat = rand(1, 4) - 0.5;
quat = quat / norm(quat);
aRb = rotmat(quaternion(quat), "point");
N = 3000;
omegab = rand(N, 3) - 0.5;
omegaa = omegab * transpose(aRb) + rand(N, 3) * 0.01;
times = 0:0.02:0.02 * (N-1);
times = transpose(times);

[a_R_b, matchesEM]= estimateRelativeRotation([times, omegaa], [times, omegab], 2:4);

assert(norm(a_R_b - aRb) < 5e-4);