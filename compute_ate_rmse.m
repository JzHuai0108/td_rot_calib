function out = compute_ate_rmse(est_sel, gt_sel, use_secs, allow_scale)
% out = compute_ate_rmse(est_sel, gt_sel, use_secs, allow_scale)
% - est_sel, gt_sel: matched rows (same K) from full trajectories, [t x y z ...]
% - use_secs:
%       [] / omitted      -> use ALL matches *minus* stationary start/end
%       scalar  T > 0     -> use [t_move, t_move + T], where t_move = GT motion start
%       scalar  T = 0     -> use the first pose at t_move
%       vector [t0, t1]   -> use [t0, t1] relative to FIRST matched GT time (legacy)
% - allow_scale: true => Similarity (Sim3); false => Rigid (SE3, s=1).
%
% Returns struct:
%   rmse, errors (K_used x 1), R (3x3), t (3x1), s (scalar),
%   P_est_aligned (K x 3) for matched pairs (aligned), P_gt (K x 3),
%   align_window: [t_start t_end] RELATIVE to first matched GT time.

    if nargin < 3, use_secs = []; end
    if nargin < 4 || isempty(allow_scale), allow_scale = true; end

    K = size(est_sel,1);
    assert(size(est_sel,2) >= 4 && size(gt_sel,2) >= 4, 'Need [t x y z ...].');
    assert(K == size(gt_sel,1), 'est_sel and gt_sel must have same number of rows.');

    % Extract matched 3D points and GT times
    P_est = est_sel(:,2:4);   % Kx3
    P_gt  = gt_sel(:,2:4);    % Kx3
    t_gt  = gt_sel(:,1);      % Kx1 (seconds)
    t0    = t_gt(1);

    % Motion detection params (tweak if needed)
    speed_thresh = 0.1;   % m/s
    min_run      = 3;     % consecutive samples above threshold

    % ---------- Build masks & alignment window ----------
    if isempty(use_secs)
        % Trim stationary start/end and use only moving span for BOTH alignment & errors
        [t_move_start, t_move_end] = find_motion_window(t_gt, P_gt, speed_thresh, min_run);
        if isempty(t_move_start) || isempty(t_move_end)
            % No motion detected: fall back to all matches
            overall_mask = true(K,1);
            align_window_abs = [t_gt(1), t_gt(end)];
        else
            overall_mask = (t_gt >= t_move_start) & (t_gt <= t_move_end);
            align_window_abs = [t_move_start, t_move_end];
        end
        mask_align = overall_mask;
        mask_error = overall_mask;   % <-- key change: errors exclude stationary ends
    elseif isscalar(use_secs)
        % Motion-start-based window: [t_move, t_move + T] for alignment;
        % Errors are evaluated on ALL matches (unchanged behavior).
        T = use_secs;
        t_move_start = find_motion_start_time(t_gt, P_gt, speed_thresh, min_run);
        if isempty(t_move_start), t_move_start = t_gt(1); end
        mask_align = (t_gt >= t_move_start) & (t_gt <= (t_move_start + T));
        if ~any(mask_align)
            warning('compute_ate_rmse: empty [t_move, t_move+T] window; falling back to all matches for alignment.');
            mask_align = true(K,1);
            align_window_abs = [t_gt(1), t_gt(end)];
        else
            align_window_abs = [t_move_start, t_move_start + T];
            disp(any(mask_align));
        end
        mask_error = true(K,1);  % keep full-trajectory errors
    else
        % Explicit relative window [a,b] from first matched GT time
        a = use_secs(1); b = use_secs(2);
        if b < a, tmp=a; a=b; b=tmp; end
        mask_align = (t_gt - t0) >= a & (t_gt - t0) <= b;
        if ~any(mask_align)
            warning('compute_ate_rmse: empty [t0,t1] window; falling back to all matches for alignment.');
            mask_align = true(K,1);
            align_window_abs = [t_gt(1), t_gt(end)];
        else
            align_window_abs = [t0 + a, t0 + b];
        end
        mask_error = true(K,1);  % keep full-trajectory errors
    end

    % ---------- Alignment on the selected subset ----------
    X = P_est(mask_align,:);   % Nx3
    Y = P_gt(mask_align,:);    % Nx3
    [R, t, s] = umeyama_alignment(X', Y', allow_scale);  % X',Y' => 3xN

    % Align all matched estimated points
    P_est_aligned = (s * (R * P_est') + t)';  % Kx3

    % Errors & RMSE on requested error mask
    diffs  = P_gt(mask_error,:) - P_est_aligned(mask_error,:);  % K_used x 3
    errors = sqrt(sum(diffs.^2, 2));                            % K_used x 1
    rmse   = sqrt(mean(errors.^2));

    out = struct('rmse', rmse, 'errors', errors, ...
                 'R', R, 't', t, 's', s, ...
                 'P_est_aligned', P_est_aligned, 'P_gt', P_gt, ...
                 'align_window', align_window_abs - t0);  % report relative to first matched GT time
end

% ---------- Helper: detect motion start & end from GT trajectory ----------
function [t_start, t_end] = find_motion_window(t, P, speed_thresh, min_run)
% Returns the first and last GT times where motion is sustained:
%   - t_start: first time with >= min_run consecutive speeds > threshold
%   - t_end:   last time with >= min_run consecutive speeds > threshold
% If none found, both are [].

    dt = diff(t); 
    dt(dt <= 0) = eps;
    dP = diff(P);                           % (K-1)x3
    speed = sqrt(sum(dP.^2, 2)) ./ dt;      % (K-1)x1, m/s
    moving = speed > speed_thresh;

    if ~any(moving), t_start=[]; t_end=[]; return; end

    box = ones(min_run,1);
    convRes = conv(double(moving), box, 'valid');   % length K-1 - min_run + 1

    % First run
    idx_first = find(convRes >= min_run, 1, 'first');
    % Last run
    idx_last  = find(convRes >= min_run, 1, 'last');

    if isempty(idx_first) || isempty(idx_last)
        t_start=[]; t_end=[]; return;
    end

    % Map to GT times: start at idx_first, end at idx_last+min_run
    % (clamp end within bounds)
    t_start = t(idx_first);
    end_idx = min(idx_last + min_run, numel(t));
    t_end   = t(end_idx);
end

function t_move = find_motion_start_time(t, P, speed_thresh, min_run)
% First time t(k) with >= min_run consecutive samples of speed > threshold
    dt = diff(t);
    dt(dt <= 0) = eps;
    dP = diff(P);
    speed = sqrt(sum(dP.^2, 2)) ./ dt;
    moving = speed > speed_thresh;

    if ~any(moving), t_move = []; return; end

    box = ones(min_run,1);
    convRes = conv(double(moving), box, 'valid');
    idx = find(convRes >= min_run, 1, 'first');
    if isempty(idx)
        t_move = [];
    else
        t_move = t(idx); % conservative
    end
end
