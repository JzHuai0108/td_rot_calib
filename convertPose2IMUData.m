function IMUData = convertPose2IMUData(PoseData, suppressLargeValues)
% given poses and timestamps, compute \ddot{x}_{is}^s and angular rate \omega_{is}^s
% pose data Each row is timestamp, position of sensor in world frame, 
% rotation to world from sensor frame in quaternion xyzw format.
% IMU data each row: time, axyz, gxyz.
% see also computeAngularRateFromAttitude

% Author: Jianzhu Huai
% Date: 2014

if nargin < 2
    suppressLargeValues = false;
end
accelTol = 20;
gyroTol = 4 * pi;
IMUData = PoseData(1:end-1,1:7);
gw = [0, 0, -9.806]; % the world frame has z up
for j=1: size(PoseData, 1)-1
    qi2w = [PoseData(j, 8), PoseData(j, 5:7)];
    qip12w = [PoseData(j+1, 8), PoseData(j+1, 5:7)];
    Ri2w = rotmat(quaternion(qi2w), 'point');
    Rip12w = rotmat(quaternion(qip12w), 'point');

    si_q_sip1 = conj(quaternion(qi2w)) * quaternion(qip12w);
    dt = PoseData(j+1,1)-PoseData(j,1);
    [qa, ~, ~, ~] = parts(si_q_sip1);
    if qa < 0
        si_q_sip1 = -si_q_sip1;
    end
    rv = rotvec(si_q_sip1);
    omega_wc_c = rv'/dt;
    if max(abs(omega_wc_c)) > gyroTol
        fprintf('Warn: Large angular rate at %.9f: %.4f %.4f %.4f\n', IMUData(j,1), omega_wc_c(1), omega_wc_c(2), omega_wc_c(3));
    end
    IMUData(j,5:7) = omega_wc_c;
    if j>1
        vi= (PoseData(j, 2:4) - PoseData(j-1, 2:4))/(PoseData(j, 1) - PoseData(j-1, 1));
        vip1= (PoseData(j+1, 2:4) - PoseData(j, 2:4))/(PoseData(j+1, 1) - PoseData(j, 1));
        ai= (vip1- vi)/(PoseData(j+1, 1) - PoseData(j-1, 1))*2;
        accel = (ai - gw)*Ri2w;
        if max(abs(accel)) > accelTol
            fprintf('Warn: Large local acceleration at %.9f: %.4f %.4f %.4f\n', IMUData(j, 1), accel(1), accel(2), accel(3));
        end
        IMUData(j,2:4) = accel;
    else
        IMUData(j,2:4) = 0;
    end
    IMUData(j,1) = (PoseData(j+1, 1) + PoseData(j, 1))/2;
end

if suppressLargeValues
    for i=1:size(IMUData, 1)
        for j = 2:4
            if abs(IMUData(i, j)) > gyroTol
                IMUData(i, j) = 0;
            end
        end
        for j = 5:7
            if abs(IMUData(i, j)) > accelTol
                IMUData(i, j) = 0;
            end
        end
    end
end
end