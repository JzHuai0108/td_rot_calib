function [m_R_e, matchesME, stdeps]= estimateRelativeRotationRobust(mdata, edata, channels, maxTimeDiff)
% estimate the relative rotation between two imu sensors E and M
% both imuDataE and imuDataM may contain accelerometer and/or gyro data
% the first column of imuDataE and imuDataM are timestamps
% channels represent which data channel are used for svd decomposition,e.g,
% 2:4 or 5:7
% return estimated rotation from E sensor frame to M sensor frame
% and the corresponding indices of time-wise matched entries

% Author: Jianzhu Huai
% Date: 2024

[m_R_e_init, matchesME] = estimateRelativeRotation(mdata, edata, channels, maxTimeDiff);
if isempty(m_R_e_init)
    m_R_e = [];
    stdeps = [];
    return;
end
fprintf('Initial m_R_e\n');
disp(m_R_e_init);

% use truncated least squares to solve for rotation given initial estimate
tol = 1e-5;
A = zeros(size(matchesME,1) * 3, 3);
b = zeros(size(matchesME,1) * 3, 1);

m_R_e = m_R_e_init;
std_noise = 0.03;
idx = 1;
for i = 1:30
    idx = 1;
    for count=1:size(matchesME,1)
        we=edata(matchesME(count,2), channels);
        wm=mdata(matchesME(count,1), channels);
        res = m_R_e * we' - wm';
        coeffs = m_R_e * we';
        crossMat = @(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
        crossD = crossMat(coeffs);
        if norm(res, 2) > max(min(norm(we, 2), norm(wm, 2)) * 0.3, 3 * std_noise)
            continue;
        else
            A(idx:idx+2, :) = crossD;
            b(idx:idx+2) = res;
            idx = idx + 3;
        end
    end
    eps = A(1:idx-1, :) \ b(1:idx-1);
    fprintf('iter %d, eps: %.8f %.8f %.8f, using %d out of %d matches\n', i, eps(1), eps(2), eps(3), (idx - 1)/3, size(matchesME,1));
    crosseps = [0 -eps(3) eps(2); eps(3) 0 -eps(1); -eps(2) eps(1) 0];
    deltaR = expm(crosseps);
    m_R_e = deltaR * m_R_e;
    if norm(eps, 2) < tol
        break;
    end
end

ATA = transpose(A(1:idx-1, :)) * A(1:idx-1, :);
coveps = std_noise * std_noise * inv(ATA);
stdeps = sqrt(diag(coveps));
fprintf('std eps: %f %f %f\n', stdeps(1), stdeps(2), stdeps(3));
end
