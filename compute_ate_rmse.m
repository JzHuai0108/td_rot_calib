function out = compute_ate_rmse(est_sel, gt_sel, allow_scale)
% allow_scale: true => similarity (Sim3); false => rigid (SE3, s=1)

    P_est = est_sel(:,2:4)';  % 3xN
    P_gt  = gt_sel(:,2:4)';   % 3xN

    if nargin < 3, allow_scale = true; end
    [R, t, s] = umeyama_alignment(P_est, P_gt, allow_scale);

    P_est_aligned = s * (R * P_est) + t;
    diffs  = P_gt - P_est_aligned;
    errors = sqrt(sum(diffs.^2,1))';
    rmse   = sqrt(mean(errors.^2));

    out = struct('rmse', rmse, 'errors', errors, ...
                 'R', R, 't', t, 's', s, ...
                 'P_est_aligned', P_est_aligned', 'P_gt', P_gt');
end


