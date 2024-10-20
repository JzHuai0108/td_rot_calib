
function [omega_is_s, fitTimes] = computeAngularRateFromAttitude(times, w_q_s, difference_option)
% output the angular rate of sensor wrt the inertial frame expressed in the
% sensor frame
% times timestsmp for each entry of w_q_s
% w_q_s: array of quaternions from sensor frame to world frame, each row (wxyz)
% difference_option: 0, forward difference, i.e., fitTimes(i-1) == times(i-1)
% 1, center difference, i.e., fitTimes(i-1) == (times(i) + times(i-1))/2
% interval, 2, backward difference, i.e., fitTimes(i-1) == times(i)
% output omega_{is}^{s} and new timestamps for each angular rate
% see also convertPose2IMUData

% Author: Jianzhu Huai
% Date: 2014

    if nargin < 3
        difference_option = 1;
    end
    omega_is_s = zeros(size(w_q_s, 1)-1, 3);
    fitTimes = zeros(size(w_q_s, 1)-1, 1);
    for i=2:size(w_q_s, 1)
        qsk2skm1 = compact(conj(quaternion(w_q_s(i-1, :))) * quaternion(w_q_s(i, :)));
        omega_is_s(i-1,:) = rotvec(quaternion(qsk2skm1))'/(times(i)- times(i-1));
        switch difference_option
            case 0
                fitTimes(i-1) = times(i-1);
            case 1
                fitTimes(i-1) = (times(i) + times(i-1))/2;
            case 2
                fitTimes(i-1) = times(i);
            otherwise
                fitTimes(i-1) = (times(i) + times(i-1))/2;
        end
    end
end