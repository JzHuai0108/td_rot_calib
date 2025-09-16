est = readmatrix('C:\Users\elev854\Downloads\run_20250910_152028\run_20250910_152028\traj.txt');  % [t tx ty tz ...]
gt  = readmatrix('C:\Users\elev854\Downloads\run_20250910_152028\run_20250910_152028\traj_gt_interp.txt');    % [t tx ty tz ...]

matches = associate2(est, gt, 0, 0.005);
% Sim(3) ATE (with scale):
res = compute_ate_rmse(est(matches(:,1),:), gt(matches(:,2),:), true);
disp(res.R);
disp(res.t);
disp(res.s);

% SE(3) ATE (no scale):
res_se3  = compute_ate_rmse(est(matches(:,1),:), gt(matches(:,2),:), false);

fprintf('Sim3 ATE RMSE = %.4f m  (N=%d)\n', res.rmse, numel(res.errors));

% Align the full estimated trajectory using the similarity found on matches
P_est_all = est(:,2:4);
P_est_all_aligned = apply_similarity(P_est_all, res.R, res.t, res.s);
P_gt_all  = gt(:,2:4);

% Prepare matched (highlight) subsets after alignment
P_est_match_aligned = res.P_est_aligned;  % already aligned in res (Nx3)
P_gt_match          = res.P_gt;           % Nx3

% ---- Plot ----
figure; hold on; axis equal; grid on;
plt1 = plot3(P_gt_all(:,1), P_gt_all(:,2), P_gt_all(:,3), '-', 'LineWidth', 1.5);
plt2 = plot3(P_est_all_aligned(:,1), P_est_all_aligned(:,2), P_est_all_aligned(:,3), '-', 'LineWidth', 1.5);

% Highlight matched points
% scatter3(P_gt_match(:,1), P_gt_match(:,2), P_gt_match(:,3), 24, 'k', 'filled');         % GT matches
% scatter3(P_est_match_aligned(:,1), P_est_match_aligned(:,2), P_est_match_aligned(:,3), 24, 'r', 'filled'); % EST matches

% ---- NEW: mark start/end on GT trajectory ----
gt_start = P_gt_all(1,:);                    % start point (GT)
gt_end   = P_gt_all(end,:);                  % end point (GT)
scatter3(gt_start(1), gt_start(2), gt_start(3), 80, 'g', 'filled', 'Marker', 'o'); % green circle
scatter3(gt_end(1),   gt_end(2),   gt_end(3),   80, 'm', 'filled', 'Marker', 's'); % magenta square
text(gt_start(1), gt_start(2), gt_start(3), '  GT start', 'Color', 'g', 'FontWeight','bold');
text(gt_end(1),   gt_end(2),   gt_end(3),   '  GT end',   'Color', 'm', 'FontWeight','bold');

% (Optional) also mark start/end on aligned EST trajectory
% est_start = P_est_all_aligned(1,:);
% est_end   = P_est_all_aligned(end,:);
% scatter3(est_start(1), est_start(2), est_start(3), 70, 'g', 'o');
% scatter3(est_end(1),   est_end(2),   est_end(3),   70, 'm', 's');
% text(est_start(1), est_start(2), est_start(3), '  EST start', 'Color', [0 0.5 0]);
% text(est_end(1),   est_end(2),   est_end(3),   '  EST end',   'Color', [0.6 0 0.6]);

legend([plt1, plt2], {'GT (all)', 'EST aligned (all)'}, 'Location','best');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title(sprintf('Aligned Trajectories (ATE RMSE = %.3f m, N=%d)', res.rmse, numel(res.errors)));
view(3);
