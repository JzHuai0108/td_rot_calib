
function matches = associate2(est, gt, time_offset, max_diff)
% Nearest-neighbor time association with tolerance.
% est, gt: [N x 4], [M x 4], time in col 1 (seconds)
    te = est(:,1) + time_offset;
    tg = gt(:,1);

    i = 1; j = 1;
    matches = zeros(0,2);
    while i <= numel(te) && j <= numel(tg)
        dt = te(i) - tg(j);
        if abs(dt) <= max_diff
            matches(end+1,:) = [i, j]; %#ok<AGROW>
            i = i + 1; j = j + 1;
        elseif dt < 0
            i = i + 1;
        else
            j = j + 1;
        end
    end
end
