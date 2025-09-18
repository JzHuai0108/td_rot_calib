function eval_ate(est_file, gt_file, use_secs, correct_scale)
% eval_ate(est_file, gt_file, use_secs, correct_scale)
% - est_file, gt_file: paths to text files [t x y z ...] (t in seconds)
% - use_secs:
%       [] / omitted      -> use all matches
%       scalar  T > 0     -> use [0, T] s from first matched GT time in
%       motion
%       vector [t0, t1]   -> use [t0, t1] s from first matched GT time
% - correct_scale: true => Sim(3); false => SE(3)

    if nargin < 3 || isempty(use_secs),  use_secs = []; end
    if nargin < 4 || isempty(correct_scale), correct_scale = true; end

    % Read and sanity check
    est = readmatrix(est_file);  % [t x y z ...]
    gt  = readmatrix(gt_file);   % [t x y z ...]
    assert(size(est,2) >= 4 && size(gt,2) >= 4, 'Need at least 4 columns [t x y z ...].');

    % Ensure time-sorted
    if any(diff(est(:,1)) < 0), est = sortrows(est,1); end
    if any(diff(gt(:,1))  < 0), gt  = sortrows(gt,1);  end

    % Associate by nearest time (≤ 5 ms)
    matches = associate2(est, gt, 0.0, 0.005);
    if size(matches,1) < 5
        error('Not enough matches between %s and %s (N=%d).', est_file, gt_file, size(matches,1));
    end

    % Compute alignment + errors using requested time window
    res = compute_ate_rmse(est(matches(:,1),:), gt(matches(:,2),:), use_secs, correct_scale);

    % Print
    [~, est_base] = fileparts(est_file);
    [~,  gt_base] = fileparts(gt_file);
    metric_type   = ternary(correct_scale, 'Sim3', 'SE3');
    win_str       = window_to_str(res.align_window);
    fprintf('%s  vs  %s | %s ATE RMSE = %.4f m  (N=%d)%s\n', ...
        est_base, gt_base, metric_type, res.rmse, numel(res.errors), win_str);
    fprintf('R =\n'); disp(res.R);
    fprintf('t = [% .6f % .6f % .6f]^T\n', res.t(:));
    fprintf('s = %.8f\n', res.s);

    % Align full estimated trajectory with found similarity
    P_est_all         = est(:,2:4);
    P_est_all_aligned = apply_similarity(P_est_all, res.R, res.t, res.s);
    P_gt_all          = gt(:,2:4);

    % Plot
    figure('Name','Aligned Trajectories','Color','w'); hold on; axis equal; grid on;
    plt1 = plot3(P_gt_all(:,1),  P_gt_all(:,2),  P_gt_all(:,3),  'g--', 'LineWidth', 1.5, 'DisplayName','GT (all)');
    plt2 = plot3(P_est_all_aligned(:,1), P_est_all_aligned(:,2), P_est_all_aligned(:,3), 'b-.',  'LineWidth', 1.5, 'DisplayName','EST aligned (all)');

    % Mark GT start/end
    gt_start = P_gt_all(1,:); 
    gt_end   = P_gt_all(end,:);
    plt3 = scatter3(gt_start(1), gt_start(2), gt_start(3), 72, 'go', 'DisplayName','GT start');
    plt4 = scatter3(gt_end(1),   gt_end(2),   gt_end(3),   72, 'ks', 'DisplayName','GT end');

    legend([plt1, plt2, plt3, plt4], 'Location','best');
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    ttl = sprintf('Aligned Trajectories — %s ATE RMSE = %.3f m (N=%d)%s', metric_type, res.rmse, numel(res.errors), win_str);
    title(ttl);
    view(3);
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end

function s = window_to_str(use_secs)
    if isempty(use_secs), s = '';
    elseif isscalar(use_secs)
        s = sprintf('  [window: 0–%.2fs]', use_secs);
    else
        a = use_secs(1); b = use_secs(2);
        if b < a, tmp=a; a=b; b=tmp; end
        s = sprintf('  [window: %.2f–%.2fs]', a, b);
    end
end
