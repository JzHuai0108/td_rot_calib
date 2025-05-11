a= rand(100, 3);

b_R_a= eul2rotm(rand(1,3));

b= a*b_R_a' + rand(100, 3) * 0.01;
% add some outliers
b(1:10, :)= b(1:10, :) + rand(10, 3) * 0.1;

a=[(1:size(a,1))', a];
b=[(1:size(b,1))', b];

[b_R_a_hat, bias, matchesab]= estimateRelativeRotationRobust(b, a, 2:4, 0.005);
disp('reference and estimated values of b_R_a');
disp([b_R_a, b_R_a_hat]);

diff = norm(b_R_a - b_R_a_hat);
assert(diff < 1e-2);
