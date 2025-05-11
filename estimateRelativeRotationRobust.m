function [m_R_e, b_est, matchesME, stdeps]= estimateRelativeRotationRobust(mdata, edata, channels, maxTimeDiff)
% estimate the relative rotation between two imu sensors E and M
% both imuDataE and imuDataM may contain accelerometer and/or gyro data
% the first column of imuDataE and imuDataM are timestamps
% channels represent which data channel are used for svd decomposition,e.g,
% 2:4 or 5:7
% return estimated rotation from E sensor frame to M sensor frame
% and the corresponding indices of time-wise matched entries

% Author: Jianzhu Huai
% Date: 2024
if nargin == 3
    [m_R_e_init, matchesME] = estimateRelativeRotation(mdata, edata, channels);
else
    [m_R_e_init, matchesME] = estimateRelativeRotation(mdata, edata, channels, maxTimeDiff);
end
if isempty(m_R_e_init)
    m_R_e = [];
    stdeps = [];
    return;
end
fprintf('Initial m_R_e\n');
disp(m_R_e_init);

% use truncated least squares to solve for rotation and bias given initial
% estimate.
% measurement equation, r = exp(theta) * m_R_e * (omega_e - b) - omega_m
estimate_bias = true;
tol       = 1e-5;
nMatches  = size(matchesME,1);
A         = zeros(nMatches*3, 6);
rhs       = zeros(nMatches*3, 1);

m_R_e     = m_R_e_init;
b_est     = zeros(3,1);
std_noise = 0.03;

for iter = 1:30
    idx = 1;
    for count = 1:nMatches
        we = edata(matchesME(count,2), channels);
        wm = mdata(matchesME(count,1), channels);

        % residual including bias
        res    = m_R_e*we' - wm' - m_R_e*b_est;
        coeffs = m_R_e*(b_est - we');

        % cross-product matrix
        crossMat = @(x)[0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
        crossD   = crossMat(coeffs);

        % truncation threshold
        thresh = max(min(norm(we,2), norm(wm,2))*0.3, 3*std_noise);
        if norm(res,2) > thresh
            continue;
        end

        A(idx:idx+2, 1:3) = -crossD;
        if estimate_bias
            A(idx:idx+2, 4:6) = m_R_e;
        end
        rhs(idx:idx+2)   = res;
        idx = idx + 3;
    end

    M  = A(1:idx-1, :);
    r  = rhs(1:idx-1);
    x  = M \ r;                % least-squares solve
    eps = x(1:3);
    delta_b = x(4:6);

    fprintf('iter %d: eps = [%.6f %.6f %.6f], Δb = [%.6f %.6f %.6f], inliers = %d/%d\n', ...
            iter, eps, delta_b, (idx-1)/3, size(matchesME,1));

    % update rotation and bias
    crosseps = [  0      -eps(3)  eps(2);
               eps(3)     0     -eps(1);
              -eps(2)  eps(1)      0   ];
    m_R_e = expm(crosseps) * m_R_e;
    b_est = b_est + delta_b;

    if norm(eps,2)<tol && norm(delta_b,2)<tol
        break;
    end
end

% covariance and standard deviations
ATA     = M.' * M;
if estimate_bias
    cov_x   = std_noise^2 * inv(ATA);
    stdeps  = sqrt(diag(cov_x(1:3, 1:3)));
    std_b   = sqrt(diag(cov_x(4:6, 4:6)));
else
    cov_x   = std_noise^2 * inv(ATA(1:3, 1:3));
    stdeps  = sqrt(diag(cov_x(1:3, 1:3)));
    std_b   = zeros(3, 1);
end
fprintf('Refined m_R_e\n');
disp(m_R_e);
fprintf('Refined bias\n');
disp(b_est');
fprintf('std(eps): [%.6f %.6f %.6f]\n', stdeps);
fprintf('std(b):   [%.6f %.6f %.6f]\n', std_b);

end
