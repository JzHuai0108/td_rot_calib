function localVel = convertPose2LocalVelocity(poseData, L_t_B_init)
% given poses W_T_L and timestamps, compute L_v_{WB} using central difference
% L_t_B_init is the body frame position in the L frame, 3x1 or 1x3 vector.
%
% pose data, each row: timestamp, position of sensor in world frame, 
% rotation to world from sensor frame in quaternion xyzw format.
%
% return local vel L_v_{WB}, each row: time, vel in local frame.
% see also computeAngularRateFromAttitude

% Author: Jianzhu Huai
% Date: 2023

localVel = poseData(1:end-1,1:4);
tol = 30;
gyroTol = 4 * pi;
for j = 1:size(poseData, 1)-1
    qi2w = [poseData(j, 8), poseData(j, 5:7)];
    qip12w = [poseData(j+1, 8), poseData(j+1, 5:7)];
    dt = poseData(j+1,1)-poseData(j,1);

    si_q_sip1 = conj(quaternion(qi2w)) * quaternion(qip12w);
    [qa, ~, ~, ~] = parts(si_q_sip1);
    if qa < 0
        si_q_sip1 = -si_q_sip1;
    end
    rv = rotvec(si_q_sip1);
    omega_wl_l = rv/dt;
    if max(abs(omega_wl_l)) > gyroTol
        fprintf('Warn: Large angular rate at %.9f: %.4f %.4f %.4f\n', IMUData(j,1), omega_wl_l(1), omega_wl_l(2), omega_wl_l(3));
    end

    vhalf = (poseData(j+1, 2:4) - poseData(j, 2:4)) / dt; % W_v_{WL}
    qhalf = slerp(quaternion(qi2w), quaternion(qip12w), 0.5); % W_q_L
    Rhalf = rotmat(qhalf, 'point');
    Lv = vhalf * Rhalf + cross(omega_wl_l, L_t_B_init); % L_v_{WB}
    localVel(j,2:4) = Lv;
    localVel(j,1) = (poseData(j+1, 1) + poseData(j, 1))/2;
    if j > 1 && norm(Lv) > tol
        fprintf('Warn: Large velocity at %.9f: %.4f %.4f %.4f\n', localVel(j,1), Lv(1), Lv(2), Lv(3));
        localVel(j, 2:4) = localVel(j-1, 2:4);
    end
end

end