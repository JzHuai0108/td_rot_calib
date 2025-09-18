function out = compute_ate_rmse(est_sel, gt_sel, use_secs, allow_scale)
% out = compute_ate_rmse(est_sel, gt_sel, use_secs, allow_scale)
% - est_sel, gt_sel: matched rows (same K) from full trajectories, [t x y z ...]
% - use_secs:
%       [] / omitted      -> use ALL matches for alignment
%       scalar  T > 0     -> use [t_move, t_move + T] where t_move = GT motion start
%       vector [t0, t1]   -> use [t0, t1] relative to FIRST matched GT time (legacy behavior)
% - allow_scale: true => Similarity (Sim3); false => Rigid (SE3, s=1).
%
% Returns struct:
%   rmse, errors (Kx1), R (3x3), t (3x1), s (scalar),
%   P_est_aligned (Kx3) for matched pairs, P_gt (Kx3),
%   align_window: [t_start t_end] in absolute GT time used for alignment.

    if nargin < 3, use_secs = []; end
    if nargin < 4 || isempty(allow_scale), allow_scale = true; end

    K = size(est_sel,1);
    assert(size(est_sel,2) >= 4 && size(gt_sel,2) >= 4, 'Need [t x y z ...].');
    assert(K == size(gt_sel,1), 'est_sel and gt_sel must have same number of rows.');

    % Extract matched 3D points and GT times
    P_est = est_sel(:,2:4);   % Kx3
    P_gt  = gt_sel(:,2:4);    % Kx3
    t_gt  = gt_sel(:,1);      % Kx1 (seconds)

    % ---------- Choose alignment mask ----------
    if isempty(use_secs)
        mask = true(K,1);
        align_window = [t_gt(1), t_gt(end)];
    elseif isscalar(use_secs)
        % Motion-start-based window: [t_move, t_move + T]
        T = use_secs;
        % Tunables for motion detection:
        speed_thresh = 0.1;   % m/s, tweak if needed
        min_run      = 3;      % consecutive samples above threshold
        t_move = find_motion_start_time(t_gt, P_gt, speed_thresh, min_run);
        if isempty(t_move)
            % Fallback if we cannot detect motion start
            t_move = t_gt(1);
        end
        mask = (t_gt >= t_move) & (t_gt <= (t_move + T));
        if ~any(mask)
            warning('compute_ate_rmse: empty [t_move, t_move+T] window; falling back to all matches.');
            mask = true(K,1);
            align_window = [t_gt(1), t_gt(end)];
        else
            align_window = [t_move, t_move + T];
        end
    else
        % Legacy behavior for explicit range [t0, t1] relative to first matched GT time
        a = use_secs(1); b = use_secs(2);
        if b < a, tmp=a; a=b; b=tmp; end
        t0 = t_gt(1);
        mask = (t_gt - t0) >= a & (t_gt - t0) <= b;
        if ~any(mask)
            warning('compute_ate_rmse: empty [t0,t1] window; falling back to all matches.');
            mask = true(K,1);
            align_window = [t_gt(1), t_gt(end)];
        else
            align_window = [t0 + a, t0 + b];
        end
    end

    % ---------- Alignment on the selected subset ----------
    X = P_est(mask,:);   % Nx3
    Y = P_gt(mask,:);    % Nx3
    [R, t, s] = umeyama_alignment(X', Y', allow_scale);  % X',Y' => 3xN

    % Align all matched estimated points
    P_est_aligned = (s * (R * P_est') + t)';  % Kx3

    % Errors & RMSE over ALL matched pairs
    diffs  = P_gt - P_est_aligned;           % Kx3
    errors = sqrt(sum(diffs.^2, 2));         % Kx1
    rmse   = sqrt(mean(errors.^2));

    out = struct('rmse', rmse, 'errors', errors, ...
                 'R', R, 't', t, 's', s, ...
                 'P_est_aligned', P_est_aligned, 'P_gt', P_gt, ...
                 'align_window', align_window - t_gt(1));
end

% ---------- Helper: detect motion start from GT trajectory ----------
function t_move = find_motion_start_time(t, P, speed_thresh, min_run)
% Returns the first time t(k) where there are 'min_run' consecutive samples
% with speed > speed_thresh. If none found, returns [].
    dt = diff(t); 
    dt(dt <= 0) = eps;
    dP = diff(P);                           % (K-1)x3
    speed = sqrt(sum(dP.^2, 2)) ./ dt;      % (K-1)x1, m/s
    moving = speed > speed_thresh;

    if ~any(moving)
        t_move = [];
        return;
    end

    % Find first index with min_run consecutive 'true'
    box = ones(min_run,1);
    convRes = conv(double(moving), box, 'valid');   % length K-1 - min_run + 1
    idx = find(convRes >= min_run, 1, 'first');
    if isempty(idx)
        t_move = [];
    else
        % Time at the START of that run; choose the corresponding GT timestamp
        t_move = t(idx);   % conservative (could also use t(idx+ceil(min_run/2)))
    end
end
